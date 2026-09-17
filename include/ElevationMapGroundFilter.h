#pragma once
#ifndef ELEVATION_MAP_GROUND_FILTER_H
#define ELEVATION_MAP_GROUND_FILTER_H

#include "Type.h"
#include "track.h"   // SimpleTracker, TrackedObstacle (用于跨帧 ID 跟踪)
#include "ReadYamlFile.h"

#include <vector>
#include <queue>
#include <cfloat>
#include <cmath>
#include <algorithm>
#include <unordered_map>
#include <memory>
#include <utility>  // 包含 std::pair
#include <Eigen/Dense>  // OBB 计算需要 SelfAdjointEigenSolver

namespace Lidar_Low_Detection
{

// 前向声明，避免循环依赖
class DebugViewer;

// ============================================================================
// 一、数据结构定义
// ============================================================================

/**
 * @brief Grid配置参数（可Config化，方便后续从YAML读取）
 *
 * 设计思想：
 * - 将参数与算法逻辑分离，后续可通过 YAML / ROS Param 动态加载
 * - 每个参数都有默认值，确保构造函数即可用
 * - slope_threshold 和 height_diff_threshold 是 Region Growing 的核心超参
 */
struct ElevationGridConfig
{
    // ---- ROI 范围 (m) ----
    float roi_x_min = 0.0f;
    float roi_x_max = 0.0f;
    float roi_y_min = 0.0f;
    float roi_y_max = 0.0f;

    //车身参数以及外扩范围
    float car_half_x = 0.0f;
    float car_half_y = 0.0f;
    float body_filter_x_threshold = 0.0f;
    float body_filter_y_threshold = 0.0f;

    // ---- Grid 分辨率 (m) ----
    float grid_resolution = 0.1f;  //10CM长宽的格子

    // ---- Ground 判定阈值 ----
    float slope_threshold      = 0.15f;   // 坡度阈值: |Δz|/distance, 对应约 8.5° 坡
    float height_diff_threshold = 0.20f;   // 直接高度差阈值 (m), 防止垂直墙壁被纳入地面
    int   min_points_per_cell   = 3;       // 有效 Grid 最少点数, 过滤噪点

    // ---- Sensor 高度 (m), 用于 Seed 选取 ----
    float sensor_height = 1.1f;

    // ========================================================================
    // 以下为 Voxel Occupancy Analysis（体素占据分析）新增参数
    // ========================================================================

    // ---- Z 方向分层分辨率 (m) ----
    // 用于构建 layer_histogram，统计每个 cell 在 Z 方向的点云分布
    // 典型值 0.05m (5cm)，与低矮障碍物高度 (10~20cm) 匹配
    float layer_resolution = 0.05f;

    
    // ---- 低矮障碍物判定阈值 (基于 top_height, 替代上面的 height_range 判定) ----
    // top_height = max_z - ground_reference_z
    //   表示障碍物顶部距离参考地面的真实高度，是区分"低矮障碍物"与"高大障碍物"的核心指标。
    //
    // 分两段距离判定:
    //   近距离 (x < near_range_boundary): top_height ∈ [top_height_min_near, top_height_max_near]
    //     推荐 5~40cm, 因为近距离点云密集, 可检测更低矮的障碍物(如 5cm 石块)
    //   中距离 (x >= near_range_boundary): top_height ∈ [top_height_min_far, top_height_max_far]
    //     推荐 10~50cm, 因为中距离点云稀疏, 太小的高度不可靠
    float obstacle_top_height_min_near = 0.10f;   // 近距离最小 top_height (m)
    float obstacle_top_height_max_near = 0.40f;   // 近距离最大 top_height (m)
    float obstacle_top_height_min_far  = 0.0f;   // 中距离最小 top_height (m)
    float obstacle_top_height_max_far  = 0.50f;   // 中距离最大 top_height (m)
    float near_range_boundary          = 2.0f;    // 近/中距离分界 (m)

    // ---- Sensor 标称高度 (仅 fallback 使用) ----
    // 当某列完全没有 Ground Grid 时，回退使用标称高度作为 ground_reference_z
    float sensor_height_nominal = 1.25f;  // 雷达离地标称高度 (m), 取反后为 -1.25m

    // ---- 占据层数阈值 ----
    // 低矮障碍物在 Z 方向应至少占据若干层，防止单层噪点被误检
    int min_occupied_layers_obstacle = 1;

    // ---- 竖直结构判定阈值 ----
    // 竖直结构（如杆子、墙壁）特征：高度大但占据层稀疏（大部分中间层为空）
    float vertical_structure_height_min = 0.50f;  // 竖直结构最小高度 (m)
    int   max_occupied_layers_vertical  = 3;       // 竖直结构最大占据层数（稀疏判定）

    // ---- Obstacle Cluster 最小 Cell 数 ----
    // BFS 聚类时，簇内 cell 数量小于此阈值的将被过滤（去除孤立噪点）
    int min_cluster_cells = 3;

    // ---- Ground Cell 高度范围上限 (m) ----
    // （当前未在 RegionGrowing 中使用，保留供后续扩展）
    float max_ground_height_range = 0.15f;

    // ---- Tall Neighbor Filter（高大物体邻域过滤） ----
    // 对于满足 is_obstacle_candidate 的 cell，检查其 8 邻域中是否有
    // is_ground && top_height >= obstacle_top_height_max_far 的"高大物体"cell。
    // 若 tall 邻居数 >= min_tall_neighbors_for_filter，说明该 cell 紧邻高大物体
    // （如人的腿部紧挨躯干），不应被当作独立的低矮障碍物 → 取消 is_obstacle_candidate。
    bool enable_tall_neighbor_filter      = true;  // 是否启用此过滤
    int  min_tall_neighbors_for_filter    = 2;     // 触发过滤所需的最少 tall 邻居数
};


















/**
 * @brief 单个 Grid Cell 信息
 *
 * 设计思想：
 * - min_z: 地面通常是一个 cell 内的最低点（激光打到地面上）
 * - max_z: 可用于判断 cell 内是否有垂直物体（如墙壁、杆子）
 * - mean_z: 保留用于后续 PMF/CSF 扩展的备用特征
 * - slope: 该 cell 与邻域的最大坡度（Region Growing 判定依据）
 * - is_ground: Grid 级别的 Ground Label
 * - label: BFS/Region Growing 的访问标记（-1 未访问，其他为组 ID）
 * - point_num: 太少的 cell 可能是噪声，用于过滤
 *
 * ==== Voxel Occupancy Analysis 新增成员 ====
 *
 * 【为什么需要 height_range？】
 *   height_range = max_z - min_z，表示该 cell 内点云的 Z 方向跨度。
 *   地面 cell 的 height_range 很小（仅地面微小起伏），障碍物 cell 则较大。
 *   这是区分"地面"和"地面上有物体"的最直观特征。
 *
 * 【为什么需要 layer_histogram？】
 *   仅靠 height_range 无法区分"密集障碍物"和"稀疏竖直结构"。
 *   例如：一个 1m 高的杆子，height_range=1.0，但 Z 方向只有少量点（稀疏）；
 *   一个 0.2m 的路沿石，height_range=0.2，但 Z 方向点密集分布。
 *   layer_histogram 统计 Z 方向每个 5cm 层内的点数，提供占据密度信息。
 *
 * 【为什么是 layer_histogram 而不是直接用点云？】
 *   - 将连续 Z 值离散化为层索引，大幅简化统计分析
 *   - 每层用 uint8_t 计数（最大 255），内存占用极小
 *   - 方便计算 occupied_layers（非空层数）和判断占据模式
 *
 * 【occupied_layers 的含义】
 *   layer_histogram 中非零层的数量。值大说明 Z 方向连续占据（密集物体），
 *   值小说明 Z 方向稀疏占据（杆子、树枝等竖直结构）。
 *
 * 【is_vertical_structure 和 is_obstacle_candidate 的关系】
 *   - is_vertical_structure: 高度大 + 占据稀疏 → 杆子、树干、墙壁边缘
 *   - is_obstacle_candidate: 高度在 10~30cm + 占据密度合理 → 路沿石、减速带、石块
 *   两者互斥：一个 cell 不能同时是两者
 */
struct GridCell
{
    // ---- 原有成员（保持接口兼容） ----
    bool  valid      = false;      // 是否包含点云
    float min_z      = FLT_MAX;    // cell 内最低点 Z
    float max_z      = -FLT_MAX;   // cell 内最高点 Z
    float mean_z     = 0.0f;       // cell 内平均 Z
    int   point_num  = 0;          // cell 内点数
    float slope      = 0.0f;       // 与邻域的最大坡度
    bool  is_ground  = false;      // 该cell属于Ground Surface，可以作为GroundReference计算（允许与is_obstacle_candidate同时为true，即Mixed Ground Cell）
    bool  is_interpolated_ground = false;  // 跨越Invalid Cell传播得到的插值地面（不是真实Ground，仅用于GroundReference）
    int   label      = -1;         // -1=未访问, >=0 表示归属的地面块 ID

    // ---- Voxel Occupancy Analysis 新增成员 ----
    float height_range            = 0.0f;  // max_z - min_z, Z方向点云跨度
    int   occupied_layers         = 0;     // layer_histogram 中非空层数量
    std::vector<uint8_t> layer_histogram;  // Z方向占据直方图, 每层点数(最大255)
    bool  is_vertical_structure   = false; // 是否为竖直结构 (杆子/墙壁)
    bool  is_obstacle_candidate   = false; // 是否为低矮障碍物候选

    // ---- Ground Reference 新增成员 ----
    // 设计原因: 仅靠 height_range 无法区分"贴近地面的低矮障碍物"和"远离地面的高大障碍物"
    // 例如: 箱子 min_z=-0.75, max_z=-0.55 → height_range=0.20m,
    //       但它距离真实地面(-1.25m)约70cm, 属于高大障碍物, 本项目不需要检测
    // 反之: 石块 min_z=-1.25, max_z=-1.08 → height_range=0.17m,
    //       距离真实地面约17cm, 这才是真正需要检测的低矮目标
    float ground_reference_z      = 0.0f;  // 参考地面高度 (从 Ground Grid 估计, 由RegionGrowing结果推导)
    float top_height              = 0.0f;  // max_z - ground_reference_z, 障碍物顶部距参考地面的真实高度

    // ---- Obstacle Clustering 标记 ----
    // 独立的 BFS 标记，不与 GroundFilter 的 label 冲突
    int   obstacle_label          = -1;    // -1=未访问, >=0 表示归属的障碍物簇 ID
};



























// ============================================================================
// GridCluster —— 障碍物 Grid 聚类结果
// ============================================================================

/**
 * @brief 障碍物 Grid 聚类结果
 *
 * 设计思想：
 *
 * 【为什么不直接用 PCL 的 EuclideanClusterExtraction？】
 *   1. PCL 的聚类在 Point 级别操作，需要 KD-Tree 邻域搜索，复杂度 O(N·logN)
 *   2. Grid 级别的聚类直接在规则网格上做 BFS，8 邻域查找 O(1)，总复杂度 O(Grid)
 *   3. Grid 已经过 Ground Filter，障碍物 cell 之间自然被地面 cell 隔开
 *   4. 避免引入 PCL 依赖，保持模块独立
 *
 * 【为什么每个簇要保留中心、尺寸等信息？】
 *   1. 后续 Kalman Tracker 需要目标的 (x, y, z) 位置和 (length, width, height) 尺寸
 *   2. 可直接转换为 S2obstacleBox 输出给下游接口
 *   3. 中心位置用于关联匹配（匈牙利算法 / JPDA），尺寸用于初始化协方差矩阵
 *
 * 【为什么保留 cell_indices？】
 *   1. 方便溯源：从簇反查哪些 GridCell 构成该障碍物
 *   2. 可视化调试：可高亮显示簇内 cell
 *   3. 二次处理：如重新计算 bounding box 或提取更精细特征
 */
struct GridCluster
{
    int   id;                     // 簇 ID, 从 0 开始递增（无跟踪时）/ 或跟踪器分配的持久 ID
    std::vector<int> cell_indices; // 簇内 GridCell 的线性索引

    int   point_num;              // 簇内总点数 (各 cell point_num 之和)

    // ---- 轴对齐包围盒 (AABB) ----
    float min_x, max_x;
    float min_y, max_y;
    float min_z, max_z;

    // ---- 包围盒中心 (世界坐标) ----
    float center_x, center_y, center_z;

    // ---- 包围盒尺寸 ----
    float length;  // X 方向尺寸 (max_x - min_x)
    float width;   // Y 方向尺寸 (max_y - min_y)
    float height;  // Z 方向尺寸 (max_z - min_z)

    // ========================================================================
    // OBB (有向包围盒, Oriented Bounding Box) —— 解决转弯时 AABB 侵入车道问题
    // ========================================================================
    //
    // 【为什么需要 OBB？】
    //   当车辆转弯时，路沿石/减速带等低矮障碍物在车身坐标系下的朝向会旋转。
    //   AABB 使用 axis-aligned 的 box，无法随障碍物朝向旋转，导致：
    //     - 障碍物斜跨两个车道时，AABB 会侵入相邻车道
    //     - 弯道上的路沿石 AABB 会覆盖整个弯道区域
    //   OBB 沿障碍物的主方向对齐，给出更紧凑的包围盒，避免误侵入。
    //
    // 【OBB 计算方式：2D PCA】
    //   收集簇内所有 cell 的 (x, y) 中心坐标
    //   → 计算 2×2 协方差矩阵
    //   → 特征分解得到主轴方向
    //   → 将所有点投影到主轴/次轴上求 min/max
    //   → 得到 4 个角点
    //
    // 【Z 方向为什么不用 OBB？】
    //   低矮障碍物（路面上的物体）Z 方向近似垂直于地面，不需要旋转。
    //   保持 Z 方向轴对齐可以简化计算，且不影响实际效果。

    float   obb_center_x  = 0.0f;   // OBB 2D 中心 X（与 center_x 相同，除非需要独立偏移）
    float   obb_center_y  = 0.0f;   // OBB 2D 中心 Y
    float   obb_length    = 0.0f;   // OBB 长轴长度（沿主方向）
    float   obb_width     = 0.0f;   // OBB 短轴长度（沿次方向）
    float   obb_angle     = 0.0f;   // OBB 旋转角 (rad)，长轴与 X 轴正向的夹角 [-π/2, π/2]
    Point2D obb_corners[4];         // OBB 4 个角点 (逆时针: 左下→右下→右上→左上)
    bool    has_obb       = false;  // 是否已成功计算 OBB（cell 数量太少时可能退化）

    // ========================================================================
    // Phase 3-B: PCA 特征值（只做保存，不改变上面任何 OBB 计算逻辑）
    //   λ 为 cluster 内 cell 中心分布的协方差特征值（单位 m^2，cov 除以 n）。
    //   λ1 >> λ2 → 主方向明显；λ1 ≈ λ2 → 方向缺乏区分度；
    //   λ2 ≈ 0 → 共线结构（方向非常明确，不是“不可靠”）。
    // ========================================================================
    float   obb_lambda_max = 0.0f;  // 主轴特征值 λ1
    float   obb_lambda_min = 0.0f;  // 次轴特征值 λ2
    float   obb_orientation_confidence = 0.0f;  // (λ1-λ2)/(λ1+λ2)，仅诊断用

    // ========================================================================
    // HDMap 软约束标签（由 HDMapFilter 填充，Cluster 级，见迁移设计文档）
    // ========================================================================
    // 设计原则（软约束，不是硬过滤）：
    //   - 低矮障碍物可能正好位于正常道路上，因此"in_road=false"不删除 Cluster
    //   - 仅作为"额外空间先验"，由 HDMapFilter 打标签，默认保留所有障碍物
    bool    in_road        = false;  // 是否位于地图可行驶(道路)区域
    bool    map_valid      = false;  // 本次过滤时 定位+地图 是否有效
    float   map_confidence = 0.0f;   // 地图置信度: 1.0=道路内, 低值=道路外/未知
};




















// ============================================================================
// 二、ElevationMapGroundFilter 类
// ============================================================================

/**
 * @brief 基于 Elevation Map 的地面滤波算法
 *
 * 整体流程（对应论文和工程中的标准流程）：
 *
 *   PointCloud
 *      │
 *   ┌──▼──────────────┐
 *   │ 1. BuildGrid    │  遍历 PointCloud → 建立 2D Grid → 统计每个 cell 信息
 *   └──┬──────────────┘
 *      │
 *   ┌──▼──────────────────┐
 *   │ 2. ComputeGroundHeight │  Grid 的地面高度 = min_z（可替换为 PMF 等）
 *   └──┬──────────────────┘
 *      │
 *   ┌──▼──────────────┐
 *   │ 3. ComputeSlope │  计算每个 cell 与 8 邻域的最大坡度
 *   └──┬──────────────┘
 *      │
 *   ┌──▼───────────────┐
 *   │ 4. RegionGrowing │  BFS 从种子 cell 开始扩展地面区域
 *   └──┬───────────────┘
 *      │
 *   ┌──▼──────────────────┐
 *   │ 5. GenerateGroundMask│  生成 Grid 级别的 is_ground 标记
 *   └──┬──────────────────┘
 *      │
 *   ┌──▼──────────────┐
 *   │ 6. SplitPointCloud│ 再次遍历 PointCloud → 按 Grid 标记分为 Ground/Obstacle
 *   └─────┬────────────┘
 *         │
 *    GroundCloud    ObstacleCloud
 *
 * 扩展性设计：
 * - ComputeGroundHeight() 目前直接用 min_z，后续可替换为 PMF 的高程估计
 * - ComputeSlope() 与 RegionGrowing() 是独立模块，便于替换为 CSF 的布料模拟
 */
class ElevationMapGroundFilter
{
public:
    // ---- 构造 / 析构 ----
    ElevationMapGroundFilter();
    // explicit 关键字专门用于单参数构造函数（或有多个参数但除第一个外都有默认值的构造函数）。加上它的核心目的只有一个：禁止编译器进行隐式类型转换。
    explicit ElevationMapGroundFilter(const ElevationGridConfig& config);
    ElevationMapGroundFilter(const ElevationGridConfig& config, const SELF_DEBUG_CONFIG yamlConfig);
    ~ElevationMapGroundFilter() = default;

    // ---- 主流程接口 ----
    /**
     * @brief 执行完整的地面滤波流程
     * @param cloud         输入点云
     * @param ground_cloud  输出: 地面点云
     * @param obstacle_cloud 输出: 障碍物点云
     * @return true 成功, false 失败
     */
    bool Process(const PointCloud2Intensity::Ptr& cloud,
                 PointCloud2Intensity::Ptr&       ground_cloud,
                 PointCloud2Intensity::Ptr&       obstacle_cloud);

    // ---- 分步接口（方便调试和单元测试） ----
    bool BuildGrid(const pcl::PointCloud<pcl::PointXYZI>& cloud);
    void ComputeGroundHeight();
    void ComputeSlope();
    void RegionGrowing();
    void GenerateGroundMask();
    void SplitPointCloud(const pcl::PointCloud<pcl::PointXYZI>& cloud,
                         pcl::PointCloud<pcl::PointXYZI>&       ground_cloud,
                         pcl::PointCloud<pcl::PointXYZI>&       obstacle_cloud);

    // // ---- 配置接口 ----
    // void SetConfig(const ElevationGridConfig& config) { m_elevationGridConfig = config; }
    /// @brief 获取当前网格配置（供外部可视化等复用同一坐标系）
    const ElevationGridConfig& GetConfig() const { return m_elevationGridConfig; }

    /// @brief 获取 BuildGrid 计算出的有效 ROI X 起点（= max(roi_x_min, car_half_x+body_filter_x_threshold)）
    ///        供外部（如 Historical Feedback 实验）与 Grid 使用同一坐标系
    float GetEffectiveRoiXMin() const { return m_effective_roi_x_min; }

    // ---- DebugViewer 注入接口 ----
    /**
     * @brief 注入调试可视化模块（可选）
     *
     * GroundFilter 不依赖 DebugViewer，如果未注入则跳过所有调试输出。
     * 注入后，所有调试图在对应算法步骤末尾自动生成。
     *
     * @param viewer  DebugViewer 实例指针（由调用方管理生命周期）
     */
    void SetDebugViewer(DebugViewer* viewer) { m_debugViewer = viewer; }

    // ---- 调试/可视化接口 ----
    const std::vector<GridCell>& GetGrid() const { return m_vGridCell; }
    int GetGridRows() const { return m_grid_rows; }
    int GetGridCols() const { return m_grid_cols; }

    // ========================================================================
    // 以下为 Voxel Occupancy Analysis（体素占据分析）新增接口
    // ========================================================================

    /**
     * @brief 执行完整流程: Ground Filter + Voxel Occupancy Analysis + Obstacle Clustering
     *
     * 在原有 Process() 基础上新增:
     *   7. AnalyzeVerticalOccupancy  → 体素占据分析
     *   8. ClusterObstacleGrid       → 障碍物聚类
     *   9. 输出 GridCluster 列表
     *
     * @param cloud            输入点云
     * @param ground_cloud     输出: 地面点云
     * @param obstacle_cloud   输出: 障碍物点云
     * @param clusters         输出: 障碍物簇列表 (可直接转换为 S2obstacleBox)
     * @return true 成功, false 失败
     */
    bool ProcessWithObstacleDetection(
        const PointCloud2Intensity::Ptr& cloud,
        PointCloud2Intensity::Ptr&       ground_cloud,
        PointCloud2Intensity::Ptr&       obstacle_cloud,
        std::vector<GridCluster>&        clusters);

    /**
     * @brief 体素占据分析: 统计每个 GridCell 在 Z 方向的点云分布
     *
     * 功能:
     *   1. 计算 height_range = max_z - min_z
     *   2. 构建 layer_histogram (Z 方向 5cm 分层统计)
     *   3. 计算 occupied_layers (非空层数)
     *   4. 根据 top_height + height_range + occupied_layers 综合判定
     *      is_vertical_structure / is_obstacle_candidate
     *   5. 基于 is_obstacle_candidate + ground_reference_z 重新分类点云
     *      → ground_cloud_debug / obstacle_cloud_debug
     *
     * 前置条件: ComputeGroundReference() 已经执行
     *
     * @param cloud           输入点云 (用于构建 layer_histogram)
     * @param ground_cloud    输出: 重新分类后的地面点云 (Ground Surface 点)
     * @param obstacle_cloud  输出: 重新分类后的障碍物点云 (仅 Obstacle Candidate 点)
     */
    void AnalyzeVerticalOccupancy(const pcl::PointCloud<pcl::PointXYZI>& cloud,
                                  pcl::PointCloud<pcl::PointXYZI>&       ground_cloud,
                                  pcl::PointCloud<pcl::PointXYZI>&       obstacle_cloud);

    /**
     * @brief 计算每个 GridCell 的 ground_reference_z (参考地面高度)
     *
     * 设计思想:
     *   不直接使用固定 sensor_height (如 -1.25m), 因为车辆可能上坡/下坡/停在斜坡,
     *   且雷达安装高度有 ±5cm 误差。
     *
     *   而是复用 RegionGrowing 已经得到的大量连续 Ground Grid:
     *     1. 按列 (col, 即 x 方向) 收集所有 ground cell 的 min_z
     *     2. 每列使用 Robust Estimator (中位数) 得到 column_ground_ref[col]
     *     3. 无 ground cell 的列通过前后有效列线性插值填补
     *     4. 全列无效时 fallback 到 sensor_height_nominal
     *     5. 对每列做轻量平滑 (3 点滑动窗口), 防止单列异常跳动
     *
     * 前置条件: GenerateGroundMask() 已经执行 (m_vGridCell 中 is_ground 标记就绪)
     *
     * 时间复杂度: O(m_grid_cols * m_grid_rows) → O(Grid)
     */
    void ComputeGroundReference();

    /**
     * @brief 8 邻域 BFS 聚类: 将 is_obstacle_candidate==true 的 GridCell 聚类为 GridCluster
     *
     * 设计原因:
     *   - 不使用 KD-Tree: Grid 邻域查询 O(1)
     *   - 不使用 PCL EuclideanClusterExtraction: 避免引入依赖
     *   - BFS 自然实现连通分量分析
     *
     * @return 障碍物簇列表
     */
    std::vector<GridCluster> ClusterObstacleGrid();

    /**
     * @brief 将 GridCluster 列表转换为 newS2AviodObject 输出
     *
     * 设计原因:
     *   - 方便直接对接项目现有下游接口 (UDP 发送等)
     *   - 填充 depth/width/height/pos/corners 等字段
     *   - 最多输出 10 个障碍物 (与 S2obstacleBox obstacle[10] 一致)
     *
     * @param clusters  输入的障碍物簇列表
     * @param output    输出的 S2AviodObject 结构体
     */
    static void ConvertClustersToS2ObstacleBox(
        const std::vector<GridCluster>& clusters,
        const unsigned long long rec_timestamp,
        newS2AviodObject&                output);



    static void ConvertTrackToS2ObstacleBox(
    const std::vector<TrackedObstacle> & trackedObstacles,
    const unsigned long long rec_timestamp,
    newS2AviodObject&                output);
    
    // ========================================================================
    // 以下为 OBB + SimpleTracker 跟踪集成接口
    // ========================================================================

    /**
     * @brief 执行完整流程: Ground Filter + 体素分析 + OBB聚类 + SimpleTracker 跟踪
     *
     * 在 ProcessWithObstacleDetection() 基础上新增:
     *   - ComputeClusterOBB: 对每个簇计算 OBB (有向包围盒)
     *   - SimpleTracker: 跨帧 ID 关联，替代简单的 cluster_id++
     *
     * 设计原因:
     *   1. OBB 解决转弯时 AABB 侵入车道的问题
     *   2. SimpleTracker 提供跨帧 ID 一致性，下游规划模块依赖稳定 ID
     *   3. 跟踪结果通过 GetTrackedClusters() 获取
     *
     * @param cloud            输入点云
     * @param ground_cloud     输出: 地面点云
     * @param obstacle_cloud   输出: 障碍物点云
     * @return true 成功, false 失败
     */
    bool ProcessWithObstacleTracking(
        const PointCloud2Intensity::Ptr& cloud,
        PointCloud2Intensity::Ptr&       ground_cloud,
        PointCloud2Intensity::Ptr&       obstacle_cloud);

    /**
     * @brief 获取最近一次 ProcessWithObstacleTracking 的跟踪结果
     *
     * 返回的 GridCluster 中:
     *   - id: 由 SimpleTracker 分配的持久 ID（跨帧一致）
     *   - has_obb: true, obb_* 字段已填充
     *   - 只有 age >= 3 的稳定轨迹才会包含在内
     *
     * @return 跟踪后的障碍物簇列表
     */
    const std::vector<GridCluster>& GetTrackedClusters() const { return tracked_clusters_; }

    /**
     * @brief 设置跟踪器的匹配阈值
     * @param threshold  匹配距离阈值 (m), 默认 1.0m
     */
    void SetTrackerMatchThreshold(float threshold);

private:
    // ---- 内部辅助函数1 ----
    /**
     * @brief 将 (x, y) 世界坐标映射到 Grid 索引
     * @return true 表示在 ROI 范围内
     */
    inline bool WorldToGrid(float x, float y, int& row, int& col) const;


    /** ---- 内部辅助函数2 ----
     * @brief 根据X坐标获取动态坡度阈值
     * @param x 网格中心的X坐标
     * @return 对应的坡度阈值
     */
    float GetDynamicSlopeThreshold(float x) const;

    
    /** ---- 内部辅助函数3 ----
     * @brief 根据X坐标获取动态高度差阈值
     * @param x 网格中心的X坐标
     * @return 对应的高度差阈值
     */
    float GetDynamicHeightDiffThreshold(float x) const;


    /**
     * @brief Grid 线性索引转换
     * 在BuildGrid函数中分析了col方向(x轴增大)在内存块中的地址是连续的，因次举例：当x相同时，y=0和y=1在内存块中就是row乘x方向有多少格子+col
     */
    inline int  GridIndex(int row, int col) const {
        return row * m_grid_cols + col; 
    }

    inline bool InBounds(int row, int col) const {
        return row >= 0 && row < m_grid_rows && col >= 0 && col < m_grid_cols;
    }

    /**
     * @brief 获取 8 邻域方向偏移
     */
    static const std::vector<std::pair<int, int>>& GetNeighborOffsets();

    /**
     * @brief 计算两个 Grid Cell 之间的坡度
     * @return slope = |Δz| / horizontal_distance
     */
    float ComputeCellSlope(const GridCell& a, const GridCell& b, float dist) const;

    /**
     * @brief 获取初始 Seed Cells（靠近 LiDAR 的一圈 Grid）
     */
    std::vector<int> GetSeedCells() const;

    // ---- Voxel Occupancy Analysis 内部辅助函数 ----

    /**
     * @brief 将 Grid 线性索引反算为世界坐标 (cell 中心)
     * @param idx   Grid 线性索引
     * @param cx    输出: cell 中心 X 坐标
     * @param cy    输出: cell 中心 Y 坐标
     */
    inline void GridIndexToWorld(int idx, float& cx, float& cy) const;

    /**
     * @brief 基于 Occupancy Analysis 结果重新分类点云 (v3)
     *
     * 替代 SplitPointCloud 的简单 is_ground 二元划分。
     *
     * v3 修正（修复"人体上半身被误判为地面"问题）：
     *   对所有 is_ground cell，统一以 ground_reference_z + margin 判定，
     *   不再区分"纯地面 cell"和"Mixed Ground Cell"：
     *     - point.z <= ground_reference_z + margin → ground_cloud
     *     - point.z >  ground_reference_z + margin → obstacle_cloud
     *   这确保即使 RegionGrowing 误将人体躯干 cell 标记为 is_ground，
     *   其中的高点也不会被归入地面。
     *
     * 前置条件: AnalyzeVerticalOccupancy 已执行（is_obstacle_candidate / ground_reference_z 已设置）
     *
     * @param cloud           输入点云
     * @param ground_cloud    输出: 地面表面点云
     * @param obstacle_cloud  输出: 障碍物点云（仅 Obstacle Candidate 相关点）
     */
    void ReclassifyPointCloud(const pcl::PointCloud<pcl::PointXYZI>& cloud,
                              pcl::PointCloud<pcl::PointXYZI>&       ground_cloud,
                              pcl::PointCloud<pcl::PointXYZI>&       obstacle_cloud);

    /**
     * @brief 从一组 cell_indices 构建单个 GridCluster 的包围盒和统计信息
     *
     * 内部会调用 ComputeClusterOBB() 计算 OBB。
     *
     * @param cluster_id      簇 ID
     * @param cell_indices    簇内所有 GridCell 的线性索引
     * @return 填充完整的 GridCluster（含 AABB 和 OBB）
     */
    GridCluster BuildClusterFromCells(int cluster_id,
                                      const std::vector<int>& cell_indices) const;

    /**
     * @brief 对单个 GridCluster 计算 OBB (有向包围盒)
     *
     * 算法: 2D PCA
     *   1. 收集簇内所有 cell 的 (x, y) 中心坐标
     *   2. 计算 2×2 协方差矩阵
     *   3. SelfAdjointEigenSolver 特征分解
     *   4. 将点投影到主轴/次轴，求 min/max 得到 OBB 尺寸
     *   5. 计算 4 个角点 (逆时针)
     *
     * 退化处理: cell 数量 < 3 时无法可靠计算 PCA，has_obb = false
     *
     * @param cluster  待填充 OBB 的簇 (会被修改)
     */
    void ComputeClusterOBB(GridCluster& cluster) const;

    /**
     * @brief 将 GridCluster 转换为 TrackedObstacle (SimpleTracker 输入格式)
     *
     * 映射关系:
     *   GridCluster.obb_center_x/y  → TrackedObstacle.pos_x/y
     *   GridCluster.center_z        → TrackedObstacle.pos_z
     *   GridCluster.obb_length      → TrackedObstacle.depth
     *   GridCluster.obb_width       → TrackedObstacle.width
     *   GridCluster.height          → TrackedObstacle.height
     *   GridCluster.obb_corners[4]  → TrackedObstacle.corners[4]
     *
     * 如果 !has_obb，则回退使用 AABB 数据。
     *
     * @param cluster  输入簇
     * @param obs      输出 TrackedObstacle (id 未设置, 由 tracker 分配)
     */
    static void ConvertClusterToTrackedObstacle(const GridCluster& cluster,
                                                TrackedObstacle&   obs);

    /**
     * @brief 将 SimpleTracker 的 vtrackings 转回 GridCluster 列表
     *
     * 只保留 age >= 3 的稳定轨迹（与 track.cpp 中的逻辑一致）。
     *
     * @param vtrackings  跟踪器的内部轨迹列表
     * @param out         输出的 GridCluster 列表（ID 已被 tracker 覆盖）
     */
    void SyncTrackedResultsToClusters(
        const std::vector<TrackedObstacle>& vtrackings,
        std::vector<GridCluster>&           out) const;

    // ---- 成员变量 ----
    ElevationGridConfig m_elevationGridConfig;

    std::vector<GridCell> m_vGridCell;   // 1D 展平的 Grid: m_vGridCell[row * cols + col]
    
    SELF_DEBUG_CONFIG m_yamlConfig;

    int m_grid_rows = 0;
    int m_grid_cols = 0;

    float m_inv_resolution = 0.0f;  // 1 / grid_resolution，快速计算索引

    // ---- Body 调整后的有效 ROI X 边界（在 《BuildGrid》 中计算） ----
    // 原 roi_x_min 可能包含车身区域，effective_roi_x_min 排除了车身+外扩范围
    // Y 方向不受 body 影响，直接使用 roi_y_min / roi_y_max
    float m_effective_roi_x_min = 0.0f;

    // ---- OBB + Tracker 成员 ----
    std::unique_ptr<SimpleTracker> tracker_;           // 简易跟踪器 (贪心匹配 + ID管理)
    std::vector<GridCluster>       tracked_clusters_;  // 最近一次跟踪结果缓存

    // // ---- DebugViewer (可选，不影响算法) ----
    DebugViewer* m_debugViewer = nullptr;  // 由调用方管理生命周期
};

}  // namespace Lidar_Low_Detection

#endif  // ELEVATION_MAP_GROUND_FILTER_H
