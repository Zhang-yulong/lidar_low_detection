#pragma once
#ifndef DEBUG_VIEWER_H
#define DEBUG_VIEWER_H

#include "ReadYamlFile.h"
#include "ElevationMapGroundFilter.h"
#include "localization_manager.h"
#include "hdmap_manager.h"
#include "coordinate_transformer.h"

#include <vector>
#include <string>
#include <set>

// #include <opencv2/opencv.hpp>

namespace Lidar_Low_Detection
{

/**
 * @brief 算法调试可视化模块（纯观察者，不修改任何算法状态）
 *
 * 设计原则：
 *   - 所有数据来源于已有算法输出，不重新计算
 *   - 所有 OpenCV 绘图封装在此模块内，GroundFilter 不直接依赖 OpenCV
 *   - GroundFilter 只负责调用 viewer->DrawXxx(...)，不做任何绘图
 *   - 支持两种模式：show（实时显示）和 save（保存到磁盘），互相独立
 *
 * 支持的调试图：
 *   1. Grid Height Map      —— 高程热力图
 *   2. Ground Mask          —— 地面分类图
 *   3. Slope HeatMap        —— 坡度热力图
 *   4. Ground Reference     —— 参考地面热力图
 *   5. Obstacle Candidate   —— 障碍物候选 + 竖直结构
 *   6. Cluster Label        —— 聚类 Label 图
 *   7. Bounding Box         —— 包围盒俯视图
 *   8. Final PointCloud Overlay —— 最终点云叠加图
 *   9. Tracker Overlay      —— 跟踪结果点云叠加图 (TrackedObstacle 版本)
 */
class DebugViewer
{
public:
    /**
     * @brief 构造函数，从 SELF_DEBUG_CONFIG 读取所有调试开关
     * @param config 全局配置（含 DebugViewer 相关字段）
     */
    explicit DebugViewer(const SELF_DEBUG_CONFIG& config);

    ~DebugViewer() = default;

    /**
     * @brief 每帧开始前调用，递增帧计数（用于保存文件命名）
     */
    void NewFrame() { m_frameCount++; }

    /**
     * @brief 每帧结束后调用，刷新 OpenCV 窗口（仅 show 模式需要）
     */
    void SpinOnce() { cv::waitKey(1); }

    // ========================================================================
    // 调试图绘制接口
    // ========================================================================

    /**
     * @brief 1. Grid 高程热力图
     *        颜色编码: 蓝(低) → 绿(中) → 黄(高) → 红(最高)
     *        数据来源: cell.min_z
     *        调用时机: BuildGrid() 之后
     */
    void DrawHeightMap(const std::vector<GridCell>& grid,
                       int rows, int cols,
                       const ElevationGridConfig& gridCfg);

    /**
     * @brief 2. Ground Mask 分类图
     *        颜色: 绿色=Ground,  红色=Non-Ground,  灰色=Invalid
     *        数据来源: cell.valid, cell.is_ground
     *        调用时机: GenerateGroundMask() 之后
     */
    void DrawGroundMask(const std::vector<GridCell>& grid,
                        int rows, int cols,
                        const ElevationGridConfig& gridCfg);

    /**
     * @brief 3. 坡度热力图
     *        颜色编码: 蓝(小坡度) → 绿(中) → 黄(大) → 红(极大)
     *        数据来源: cell.slope
     *        调用时机: ComputeSlope() 之后
     */
    void DrawSlope(const std::vector<GridCell>& grid,
                   int rows, int cols,
                   const ElevationGridConfig& gridCfg);

    /**
     * @brief 4. Ground Reference 热力图
     *        颜色编码: 蓝→绿→黄→红 (按 ground_reference_z 取值)
     *        数据来源: cell.ground_reference_z
     *        调用时机: ComputeGroundReference() 之后
     */
    void DrawReference(const std::vector<GridCell>& grid,
                       int rows, int cols,
                       const ElevationGridConfig& gridCfg);

    /**
     * @brief 5. 障碍物候选图
     *        颜色: 绿色=Ground,  黄色=Obstacle Candidate,
     *              红色=Vertical Structure,  灰色=Invalid
     *        数据来源: cell.is_ground, cell.is_obstacle_candidate,
     *                 cell.is_vertical_structure
     *        调用时机: AnalyzeVerticalOccupancy() 之后
     */
    void DrawObstacleCandidate(const std::vector<GridCell>& grid,
                               int rows, int cols,
                               const ElevationGridConfig& gridCfg);

    /**
     * @brief 6. 聚类 Label 图
     *        每个障碍物簇用不同颜色，非障碍物 cell 为灰色/黑色
     *        数据来源: cell.obstacle_label, cell.is_obstacle_candidate
     *        调用时机: ClusterObstacleGrid() 之后
     */
    void DrawCluster(const std::vector<GridCell>& grid,
                     int rows, int cols,
                     const ElevationGridConfig& gridCfg);

    /**
     * @brief 7. 包围盒俯视图
     *        在 2D 平面上绘制每个 GridCluster 的 AABB/OBB
     *        数据来源: std::vector<GridCluster>
     *        调用时机: ClusterObstacleGrid() 之后
     */
    void DrawBoundingBox(const std::vector<GridCluster>& clusters,
                         const ElevationGridConfig& gridCfg);

    /**
     * @brief 8. 最终点云叠加图
     *        俯视图: 绿色=Ground点, 红色=Obstacle点, 白色=包围盒
     *        数据来源: ground_cloud, obstacle_cloud, clusters
     *        调用时机: SplitPointCloud() 之后
     */
    void DrawClusterOverlay(const pcl::PointCloud<pcl::PointXYZI>& groundCloud,
                     const pcl::PointCloud<pcl::PointXYZI>& obstacleCloud,
                     const std::vector<GridCluster>& clusters,
                     const ElevationGridConfig& gridCfg,
                     const std::vector<std::vector<STR_POINT2F>>& mapPolygons,
                    const LocalizationManager::Pose& pose);

    /**
     * @brief 9. 跟踪结果点云叠加图 (TrackedObstacle 版本)
     *        俯视图: 绿色=Ground点, 红色=Obstacle点,
     *              白色=跟踪器corners多边形, 黄色=中心, 青色箭头=速度向量
     *        数据来源: ground_cloud, obstacle_cloud, trackers (SimpleTracker 输出)
     *        调用时机: SimpleTracker::update() 之后
     */
    void DrawTrackOverlay(const pcl::PointCloud<pcl::PointXYZI>& groundCloud,
                     const pcl::PointCloud<pcl::PointXYZI>& obstacleCloud,
                     const std::vector<TrackedObstacle>& trackers,
                     const ElevationGridConfig& gridCfg);

    /**
     * @brief 在已构造好的 TrackerOverlay 图像上叠加 HDMap 道路/车道轮廓（白色）
     *        仅做可视化，不修改算法状态。
     *        传入 mapPolygons 为地图系多边形，内部会转至车辆系再映射为像素。
     */
    void DrawHdMapOverlay(cv::Mat& image,
                          const std::vector<std::vector<STR_POINT2F>>& mapPolygons,
                          const LocalizationManager::Pose& pose,
                          const ElevationGridConfig& gridCfg);

    /**
     * @brief 【临时调试】HDMap 叠加校准：1m×1m 参考框 + 最近 HDMap 边界点
     *        用于反算实际 pixels/meter 与理论值 kGridPixelScale/grid_resolution 是否一致，
     *        并核对 mapToVehicle → WorldToPixel 链路。排查完成后请删除本函数及其调用。
     */
    void DrawHdMapOverlayCalib(cv::Mat& image,
                               const std::vector<std::vector<STR_POINT2F>>& mapPolygons,
                               const LocalizationManager::Pose& pose,
                               const ElevationGridConfig& gridCfg) const;

    void DrawAllOverlay(const pcl::PointCloud<pcl::PointXYZI>& groundCloud,
                     const pcl::PointCloud<pcl::PointXYZI>& obstacleCloud,
                     const std::vector<TrackedObstacle>& trackers,
                     const ElevationGridConfig& gridCfg,
                     const std::vector<std::vector<STR_POINT2F>>& mapPolygons,
                     const LocalizationManager::Pose& pose);

    
    int GetFrameCountValue() const;

private:
    // ========================================================================
    // 内部辅助函数
    // ========================================================================

    /**
     * @brief 统一的 show / save 处理
     * @param image      已绘制好的图像
     * @param name       窗口名 / 文件名前缀
     * @param viewCfg    对应的调试配置（enable/show/save）
     */
    void ShowOrSave(const cv::Mat& image, const std::string& name,
                    const DEBUG_VIEWER_CONFIG& viewCfg);

    /**
     * @brief 统计本帧开启 show 的窗口总数（用于垂直均分屏幕空间）
     * @return 窗口数量，至少为 1
     */
    int CountShownWindows() const;

    /**
     * @brief 首次显示某窗口时：等比缩放并定位到自动布局位置
     *
     * 保证：
     *  - 初始窗口大小固定（按屏幕区域等比缩放，只缩小不放大），启动不再“小得需要手动放大”；
     *  - 多个窗口按“一上一下”垂直排布，不重叠（Overlay 与 TrackerOverlay 会上下排列）。
     * 仅首次调用时生效，后续保持用户手动调整后的位置/大小。
     *
     * @param name  窗口名
     * @param image 待显示的图像（用于获取原始尺寸）
     */
    void LayoutWindow(const std::string& name, const cv::Mat& image);

    /**
     * @brief 根据 min_z 值计算热力图颜色 (BGR)
     * @param z_val      当前 z 值
     * @param z_min      全局 z 最小值
     * @param z_max      全局 z 最大值
     * @return BGR 颜色 (cv::Vec3b)
     */
    static cv::Vec3b ZValueToColor(float z_val, float z_min, float z_max);

    /**
     * @brief 根据 slope 值计算热力图颜色 (BGR)
     */
    static cv::Vec3b SlopeToColor(float slope, float slope_max);

    /**
     * @brief 为不同的 cluster label 生成伪彩色
     */
    static cv::Vec3b LabelToColor(int label);

    /**
     * @brief 世界坐标 (x, y) → 图像像素坐标 (px, py)
     *        x: 前向 → 图像列 (左→右)
     *        y: 左右 → 图像行 (下→上，需要翻转)
     */
    void WorldToPixel(float wx, float wy, int& px, int& py,
                      const ElevationGridConfig& gridCfg) const;

    /**
     * @brief 计算与 Grid 调试图(1~6层)一致的图像几何参数
     *        第7/8/9层复用该坐标系，保证窗口宽高、盲区偏移与 1~6 层完全对齐
     * @param gridCfg            网格配置
     * @param[out] rows          网格行数 (Y方向)
     * @param[out] cols          网格列数 (X方向)
     * @param[out] px_blind_offset 车前盲区像素偏移
     * @param[out] img_w         图像宽度 (像素)
     * @param[out] img_h         图像高度 (像素)
     */
    void ComputeGridImageGeometry(const ElevationGridConfig& gridCfg,
                                  int& rows, int& cols,
                                  int& px_blind_offset,
                                  int& img_w, int& img_h) const;


    void AddBackGround(cv::Mat &image, const int img_w, const int img_h, const ElevationGridConfig& gridCfg);


    // ========================================================================
    // 成员变量
    // ========================================================================

    const SELF_DEBUG_CONFIG& m_DV_config;   // 全局配置引用
    int m_frameCount;              // 帧计数，用于保存文件命名

    // 图像尺寸 (像素)
    // Grid 调试图: 每 cell 映射为 scale 个像素
    // 点云叠加图: 使用固定分辨率
    static constexpr int kGridPixelScale = 10;    // 每个 Grid Cell 映射的像素数
    static constexpr int kOverlayWidth   = 800;  // 点云叠加图宽度
    static constexpr int kOverlayHeight  = 800;  // 点云叠加图高度

    //普通图片
    // 3. 图像尺寸计算 (必须包含盲区宽度)
    int expand_img_w = 100; 
    int expand_img_h = 200; 

    // ========================================================================
    // show 模式：窗口初始尺寸与自动排布（固定初始大小 + 一上一下，避免重叠）
    // 以下为屏幕布局参数，可按实际屏幕分辨率调整：
    //   m_layoutMaxW/m_layoutMaxY 对应可用屏幕区域（默认按 1920×1080 屏幕预留）
    // ========================================================================
    std::set<std::string> m_shownWindows;  // 已创建并完成定位的窗口名
    int m_layoutX        = 1000;             // 下一个窗口左上角 X
    int m_layoutY        = 40;             // 下一个窗口左上角 Y（初始 = 顶部留白）
    int m_layoutTopY     = 40;             // 顶部留白（避开标题栏）
    int m_layoutMaxW     = 1900;           // 窗口最大宽度 (px)
    int m_layoutMaxY     = 1000;           // 窗口垂直排布上限 (px)
    int m_windowGap      = 20;             // 相邻窗口间隔 (px)

};

}  // namespace Lidar_Low_Detection

#endif  // DEBUG_VIEWER_H
