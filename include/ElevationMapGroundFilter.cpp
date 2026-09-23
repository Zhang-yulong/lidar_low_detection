/**
 * @file ElevationMapGroundFilter.cpp
 * @brief 基于 Elevation Map 的地面滤波算法 —— 完整工程实现
 *
 * 设计理念：
 *   本模块参考了 Autoware 的 ray_ground_filter 和 Apollo 的 ground_segmentation，
 *   采用 Grid-based Elevation Mapping + Region Growing 的经典范式。
 *
 *   为什么不直接使用 PCL 的 SASegmentation / PMF？
 *     - PCL 的 RANSAC 平面拟合假设地面是单一平面，无法处理坡道、起伏路面
 *     - PCL 的 PMF 基于形态学，参数调节困难，且对大范围场景效率低
 *     - 自己实现可以精确控制每个环节，方便后续替换 PMF/CSF
 *
 * 核心思想：
 *   - 先在 Grid 层面做 Label（高效，Grid 数量 << 点数量）
 *   - 再回溯到 Point 层面做 Split（每个点只查一次 Grid）
 *
 * 时间复杂度：O(N + R*C)，N 为点数，R*C 为 Grid 数量
 * 空间复杂度：O(R*C + N)，Grid 存储 + 输入点云
 */

#include "ElevationMapGroundFilter.h"
#include "DebugViewer.h"
#include "ulog_api.h"
// Phase 3-B': cv::minAreaRect（1cm Raw Point Image → 实验 OBB）。
// 仅在 .cpp 中依赖 OpenCV，头文件仍然不依赖任何 OpenCV 类型。
#include <opencv2/imgproc.hpp>
namespace Lidar_Low_Detection
{

// ============================================================================
// 8 邻域方向偏移表（static，全局一份，避免重复构造）
// ============================================================================

const std::vector<std::pair<int, int>>& ElevationMapGroundFilter::GetNeighborOffsets()
{
    // 8 邻域: N, NE, E, SE, S, SW, W, NW
    static const std::vector<std::pair<int, int>> offsets = {
        {-1,  0},  // N  上     ->物理空间：车辆的 左 (Left)
        {-1,  1},  // NE 右上   -
        { 0,  1},  // E  右     ->物理空间：车辆的 前 (Forward)
        { 1,  1},  // SE 右下
        { 1,  0},  // S  下     ->物理空间：车辆的 右 (Right)
        { 1, -1},  // SW 左下
        { 0, -1},  // W  左     ->物理空间：车辆的 后 (Backward)
        {-1, -1}   // NW 左上
    };
    return offsets;
}

// ============================================================================
// 构造 / 析构
// ============================================================================

ElevationMapGroundFilter::ElevationMapGroundFilter()
{
    // 使用默认配置
}

ElevationMapGroundFilter::ElevationMapGroundFilter(const ElevationGridConfig& config)
    : m_elevationGridConfig(config)
{
}


ElevationMapGroundFilter::ElevationMapGroundFilter(const ElevationGridConfig& config, const SELF_DEBUG_CONFIG yamlConfig)
    : m_elevationGridConfig(config), m_yamlConfig(yamlConfig)
{

}
// ============================================================================
// 主流程
// ============================================================================

bool ElevationMapGroundFilter::Process(
    const PointCloud2Intensity::Ptr& cloud,
    PointCloud2Intensity::Ptr&       ground_cloud,
    PointCloud2Intensity::Ptr&       obstacle_cloud)
{
    if (!cloud || cloud->empty())
    {
        return false;
    }

    // Step 1: 建立 Elevation Map（Grid）并统计信息 —— 只遍历一次 PointCloud
    if (!BuildGrid(*cloud))
    {
        return false;
    }

    // Step 2: 计算每个 Grid 的地面高度（默认 min_z，可替换为 PMF）
    ComputeGroundHeight();

    // Step 3: 计算每个 Grid 与邻域的最大坡度
    ComputeSlope();

    // Step 4: BFS Region Growing —— 从种子 Grid 开始扩展地面
    RegionGrowing();

    // Step 5: 生成 Ground Label Map
    GenerateGroundMask();

    // Step 6: 回溯 PointCloud，按 Grid Label 分离
    // SplitPointCloud(*cloud, *ground_cloud, *obstacle_cloud);

    return true;
}

// ============================================================================
// 一、BuildGrid —— 建立 Grid & 统计信息
// ============================================================================

/**
 * 设计思想：
 *
 * 【为什么只遍历一次 PointCloud？】
 *   遍历 PointCloud 是 O(N) 的操作，每次遍历都有 cache miss 和内存带宽压力。
 *   如果分多次遍历（先算 min_z，再算 max_z，再算 mean_z），会把 O(N) 变成 3*O(N)。
 *   多次遍历还会导致点云数据从 L3 cache 重复加载。
 *   在一次遍历中完成所有统计，利用 CPU 流水线和数据局部性，大幅提升效率。
 *
 * 【为什么 Elevation Mapping 保存 min_z 而不是 mean_z？】
 *   - 单个 Grid Cell（例 0.1m×0.1m）可能同时包含地面反射点和障碍物反射点
 *   - 地面点通常对应最低的 Z 值（激光从上方打到地面）
 *   - 如果使用 mean_z，障碍物点（如路沿石顶面、车顶）会抬高平均值
 *   - 极端情况：cell 内有 50 个地面点(z≈-1.5)和 1 个障碍物点(z≈-0.5)，
 *     mean_z 会被显著拉高，导致无法正确判断地面
 *   - 因此 Elevation Mapping 的经典做法是保留 min_z 作为地面高度估计
 *
 * 【为什么需要 point_num？】
 *   - 过滤孤立噪点：如果某个 cell 只有 1~2 个点，很可能是噪声或多路径反射
 *   - 确保统计可靠性：点数太少时 min_z 不可靠
 *   - Region Growing 的门控条件
 *
 * 【为什么需要 label？】
 *   - BFS 访问标记：-1 = 未访问
 *   - 区分不同 ground segment（后续可扩展为连通分量分析）
 *
 * 【为什么 Ground Label 不能直接放在 Point 里？】
 *   - Grid 数量远小于 Point 数量（本例 Grid≈80,000，Point≈53,000）
 *     但在典型场景中 Grid 有效比例更低（~30%）
 *   - Region Growing 操作的是 Grid 之间的拓扑关系，不是 Point 之间的
 *   - 先 Label Grid 再回溯 Point 是典型的"粗粒度→细粒度"策略
 *   - 如果直接 Label Point，Region Growing 需要在点级别做邻域搜索，
 *     复杂度从 O(R*C) 变成 O(N*logN)（KD-Tree 邻域搜索）
 */

bool ElevationMapGroundFilter::BuildGrid(const pcl::PointCloud<pcl::PointXYZI>& cloud)
{
    if (cloud.empty())
    {
        return false;
    }

    // ---- 计算 Grid 尺寸 ----
    m_inv_resolution = 1.0f / m_elevationGridConfig.grid_resolution;

    // ── 计算 Body 调整后的有效 ROI X 边界 ──
    // 只对 X（前向）做 body 调整: 车身占据 x ∈ [0, car_half_x+body_filter]，Grid 从车身之外开始
    // Y（左右）不受影响，直接使用 yaml 中的 roi_y_min/max
    float final_car_x = m_elevationGridConfig.car_half_x + m_elevationGridConfig.body_filter_x_threshold;
    m_effective_roi_x_min = std::max(m_elevationGridConfig.roi_x_min, final_car_x);

    // x: 前向 (forward), y: 左向 (left)
    float roi_x_span = m_elevationGridConfig.roi_x_max - m_effective_roi_x_min;  // 前向跨度 (m)
    float roi_y_span = m_elevationGridConfig.roi_y_max - m_elevationGridConfig.roi_y_min;  // 左右跨度 (m)

    /*
    在 C++ 中，二维数组或一维数组模拟的二维网格，通常采用行主序（Row-Major）存储。内存索引计算公式为：idx = row * m_grid_cols + col。
    这意味着：
    1. 当 row 固定，col 递增时，内存地址是连续递增的。
       在本项目中，col 映射的是物理 X 轴（前向），因此【前向(X)】在内存中是连续的。
    2. 当 col 固定(假设x=0.9)，row 递增(假设y=-3 -》y=0 -》 y=3)时，相当于在内存中跨越了整行（步长为 m_grid_cols），地址不连续。
       在本项目中，row 映射的是物理 Y 轴（左右），因此【左右(Y)】在内存中是不连续的。

    当我们在网格中沿 X 轴（前向） 移动时，内存地址是连续递增的。因此，X 方向的跨度被映射为 Column（列）。
    当我们在网格中沿 Y 轴（左右） 移动时，相当于在内存中跨越了一整行，内存地址是不连续的。因此，Y 方向的跨度被映射为 Row（行）。
    */

    // m_grid_cols 沿 x 方向 (前向), m_grid_rows 沿 y 方向 (左右)
    m_grid_cols = static_cast<int>(std::ceil(roi_x_span * m_inv_resolution));  // x→col (前向)
    m_grid_rows = static_cast<int>(std::ceil(roi_y_span * m_inv_resolution));  // y→row (左右)

    // 预分配并初始化 Grid
    m_vGridCell.clear();
    m_vGridCell.resize(static_cast<size_t>(m_grid_rows) * m_grid_cols);

    // ── Phase 3-B': 重置 1cm Raw Point Image（尺寸由配置 ROI 现算）──
    // 放在 BuildGrid 内：与本帧 Grid 尺寸同时确定；图像内容在 ReclassifyPointCloud 中填充。
    ResetRawPointImage();

    // std::cout << "Elevation Grid: " << m_grid_rows << " rows (y, ±" << m_elevationGridConfig.roi_y_max
    //           << "m) × " << m_grid_cols << " cols ("<< m_elevationGridConfig.roi_x_min <<", " << m_elevationGridConfig.roi_x_max
    //           << "m), total cells: " << m_vGridCell.size() << std::endl;

    LOG_RAW("@@@ 过滤后InputCloud: %d ->生成Elevation Grid: %d rows (y, ±%.2fm) × %d cols (%.2f, %.2fm), total cells: %d\n", 
            cloud.size(), m_grid_rows, m_elevationGridConfig.roi_y_max, m_grid_cols, m_effective_roi_x_min, m_elevationGridConfig.roi_x_max, m_vGridCell.size());

    /**
     * 【一次性遍历 PointCloud 完成所有统计】
     *
     * 时间复杂度：O(N)，N = cloud.size()
     *   - 每个点只做一次 (x,y)→(row,col) 映射：O(1) 算术运算
     *   - 每次更新 GridCell 也是 O(1)
     *
     * 空间复杂度：O(R*C)，与点数无关
     */
    int test_num=0;
    for (const auto& point : cloud.points)
    {
        // 过滤 NaN / Inf
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
        {
            continue;
        }

        int row, col;
        if (!WorldToGrid(point.x, point.y, row, col))
        {
            continue;  // 超出 ROI 范围
        }

        int idx = GridIndex(row, col);
        GridCell& cell = m_vGridCell[idx];

        // ── 一次性更新所有统计量 ──
        cell.min_z  = std::min(cell.min_z, point.z);
        cell.max_z  = std::max(cell.max_z, point.z);
        cell.mean_z += point.z;    // 累加，后续除以 point_num
        cell.point_num++;
    
        //
        // 定义中心点坐标
        float cx = 4.363379f;
        float cy = 0.691507f;
        float cz = -1.293546f;
        float half_size = 0.5f;

        // 在遍历点云的循环中设置 if 条件
        // LOG_RAW("要测试的低矮障碍物点周围的点：\n");
        if (point.x >= cx - half_size && point.x <= cx + half_size &&
            point.y >= cy - half_size && point.y <= cy + half_size)
        {   
            // 满足条件的点即为包围盒内的点
            // 在此处执行提取或保存操作
            // LOG_RAW("pt%d,(%.3f,%.3f,%.3f)\n", test_num,point.x, point.y, point.z);
            test_num++;
        }
    
    }

    int invalid_count = 0;
    // ---- 后处理：标记 valid，计算 mean_z ----
    for (auto& cell : m_vGridCell)
    {
        if (cell.point_num >= m_elevationGridConfig.min_points_per_cell)
        {
            cell.valid  = true;
            cell.mean_z /= static_cast<float>(cell.point_num);
        }
        else
        {
            invalid_count++;
            // 点数不足，视为无效 cell
            cell.valid     = false;
            cell.min_z     = FLT_MAX;
            cell.max_z     = -FLT_MAX;
            cell.mean_z    = 0.0f;
            cell.point_num = 0;
        }
    }

    LOG_RAW("要求cell中最低点数量: %d ->有效cell数量: %d, 无效cell数量: %d\n", m_elevationGridConfig.min_points_per_cell, (m_vGridCell.size()- invalid_count), invalid_count);

    // ── Debug: 高程热力图 ──
    if (m_debugViewer)
    {
        m_debugViewer->DrawHeightMap(m_vGridCell, m_grid_rows, m_grid_cols, m_elevationGridConfig);
    }

    return true;
}

// ============================================================================
// 内部函数：WorldToGrid
// ============================================================================

inline bool ElevationMapGroundFilter::WorldToGrid(float x, float y, int& row, int& col) const
{
    // 使用半开区间 [min, max) 避免边界点越界
    // 例如 x=40.0 时 col = floor(40*10) = 400, 但 m_grid_cols=400, 越界!
    // 使用 >= 确保 x==roi_x_max 被提前排除
    if (x < m_effective_roi_x_min || x >= m_elevationGridConfig.roi_x_max ||
        y < m_elevationGridConfig.roi_y_min || y >= m_elevationGridConfig.roi_y_max)
    {
        return false;
    }

    // 坐标系映射（已在上方验证不会越界，但保留 InBounds 做防御性编程）:
    //   x (前向, forward)  → col
    //   y (左向, left)     → row
    row = static_cast<int>(std::floor((y - m_elevationGridConfig.roi_y_min) * m_inv_resolution));
    col = static_cast<int>(std::floor((x - m_effective_roi_x_min) * m_inv_resolution));

    // 防御性检查：浮点精度可能导致 row/col 刚好等于 m_grid_rows/m_grid_cols
    if (row >= m_grid_rows) row = m_grid_rows - 1;
    if (col >= m_grid_cols) col = m_grid_cols - 1;

    return InBounds(row, col);
}



float ElevationMapGroundFilter::GetDynamicSlopeThreshold(float x) const {
    if (x < 3.5f) {
        return m_elevationGridConfig.slope_threshold; // 近距离保持严格
    } else if (x > m_elevationGridConfig.roi_x_max) {
        return 0.80f; // 超出ROI，使用最大值
    } else {
        // 在 [3.5, roi_x_max] 区间内线性插值
        return m_elevationGridConfig.slope_threshold + (x - m_elevationGridConfig.slope_threshold) * 0.1f;
    }
}



float ElevationMapGroundFilter::GetDynamicHeightDiffThreshold(float x) const {
    if (x < 3.5f) {
        return m_elevationGridConfig.height_diff_threshold; // 近距离保持严格, 比如我想检测（10-30cm，算加一点点±5cm误差，设置0.35）
    } else if (x > m_elevationGridConfig.roi_x_max) {
        return 0.6; // 超出ROI，使用最大值， 0.6m对应x_max是8m，
    } else {
        // 在 [3.5, roi_x_max] 区间内线性插值
        return m_elevationGridConfig.height_diff_threshold + (x - 3.5f) * (0.60f - m_elevationGridConfig.height_diff_threshold) / (m_elevationGridConfig.roi_x_max - 3.5f);
    }
}


// ============================================================================
// 三、ComputeGroundHeight —— 计算地面高度
// ============================================================================

/**
 * 设计思想：
 *
 * 【为什么 Ground 一般对应最低点？】
 *   - 激光雷达安装在车顶，光束从上方斜向下扫描
 *   - 地面是场景中最低的连续表面（除了下坡和沟壑）
 *   - 打到地面的激光点 Z 值最小（车身坐标系下 Z 轴向上）
 *
 * 【为什么 Obstacle 会提高 mean_z？】
 *   - 障碍物（车辆、行人、墙壁）的顶部点 Z 值明显高于地面
 *   - 如果 cell 内同时有地面和障碍物点，mean_z 会被障碍物拉高
 *   - 极端情况：50 个地面点 z=-1.5 + 1 个障碍物点 z=0.5 → mean_z ≈ -1.46
 *     而真实地面是 -1.5，差别虽然不大，但在坡道场景会被放大
 *
 * 【哪些情况下 min_z 可能失效？】
 *   1. 悬空物体：树枝、天桥下方 —— laser 先打到上方物体，min_z 不是地面
 *   2. 多路径反射：导致虚假低点
 *   3. 下坡/沟壑：地面点 Z 低于正常值，但这是真实地形，不算失效
 *   4. 传感器遮挡：近处物体挡住后方地面
 *   解决方案：后续可扩展为 PMF（通过多尺度形态学滤波去除悬空物影响）
 *
 * 【如何处理空 Grid？】
 *   - 在 BuildGrid 中已经过滤：point_num < min_points_per_cell 标记为 invalid
 *   - Region Growing 时跳过 invalid cell
 *   - 如果种子区域有大量空 Grid（如 ROI 边缘），不影响核心区域
 *
 * 【后续扩展：替换为 PMF】
 *   - 当前 ComputeGroundHeight 直接使用 min_z
 *   - 替换为 PMF 时，只需修改此函数：对 m_vGridCell 执行多尺度形态学开运算
 *   - 接口不变，Region Growing 等下游模块无需修改
 */

void ElevationMapGroundFilter::ComputeGroundHeight()
{
    // 当前实现：Ground Height = min_z
    // 已在 BuildGrid 中计算完毕，此函数作为扩展占位
    //
    // PMF 扩展示例（伪代码）：
    //   for (int window_size = 1; window_size <= max_window; window_size *= base)
    //   {
    //       for each cell:
    //           cell.min_z = min(min_z in window)
    //       for each cell:
    //           if (original_min_z - cell.min_z < threshold)
    //               cell.min_z = original_min_z
    //   }
    //
    // CSF 扩展示例（伪代码）：
    //   在 Grid 上建立虚拟布料，通过粒子运动模拟布料贴合地面
}

// ============================================================================
// 四、ComputeSlope —— 计算坡度
// ============================================================================

/**
 * 设计思想：
 *
 * 【为什么不用简单的高度差？】
 *   - 高度差 |Δz| 本身不包含空间距离信息
 *   - 同样 0.1m 的高度差：
 *     - 如果水平距离 1m，这是缓坡（slope=0.1，arctan(0.1)约 5.7°）
 *     - 如果水平距离 0.1m，这是陡坎（slope=1.0，arctan(1.0)约 45°）
 *   - 用 slope = |Δz| / distance 可以归一化距离的影响
 *
 * 【为什么 Slope 适合坡道？】
 *   - 坡道：地面连续上升，相邻 cell 之间的高度差随距离线性增长
 *     slope 保持恒定（≈ 坡度角的正切值）
 *   - 障碍物边缘：相邻 cell 高度突变，distance 很小但 |Δz| 很大
 *     slope 会非常大，自然被排除
 *   - 平坦地面：|Δz| 接近 0，slope 接近 0
 *
 * 【如何设置 Slope Threshold？】
 *   - 城市道路最大坡度通常 ≤ 8%（约 4.6°），对应 slope ≈ 0.08
 *   - 地下车库坡道可达 15%（约 8.5°），对应 slope ≈ 0.15
 *   - 建议阈值 0.15 ~ 0.20，同时配合 height_diff_threshold 做二次确认
 *   - 为什么需要 height_diff_threshold？
 *     对角线方向 distance = sqrt(2)*resolution ≈ 0.14m
 *     如果 |Δz|=0.02m（路面微小起伏），slope=0.02/0.14≈0.14 > 0.15 可能误判
 *     加入 height_diff_threshold=0.2m，|Δz|<0.2m 时不做 slope 判定
 *
 * 时间复杂度：O(R*C*8) → O(R*C)，每个 cell 最多检查 8 个邻居
 */

void ElevationMapGroundFilter::ComputeSlope()
{
    const auto& offsets = GetNeighborOffsets();

    for (int row = 0; row < m_grid_rows; ++row)  //从 y_min（最右边，-）开始，一直遍历到 y_max（最左边，+）
    {
        for (int col = 0; col < m_grid_cols; ++col) //从 x_min（车后方/近处）开始，一直遍历到 x_max（车前方/远处）
        {
            int idx = GridIndex(row, col);
            GridCell& cell = m_vGridCell[idx];

            if (!cell.valid)
            {
                cell.slope = FLT_MAX;  // 无效 cell 的 slope 设为极大，不会被扩展
                continue;
            }

            float max_slope = 0.0f;
            const float cell_z = cell.min_z;  // 使用地面高度

            for (const auto& [dr, dc] : offsets)
            {
                int nr = row + dr;
                int nc = col + dc;

                if (!InBounds(nr, nc))
                {
                    continue;
                }

                const GridCell& neighbor = m_vGridCell[GridIndex(nr, nc)];
                if (!neighbor.valid)
                {
                    continue;
                }

                // 计算两个 cell 中心之间的水平距离
                // 对角线方向：sqrt(resolution² + resolution²) = resolution * sqrt(2)
                float dist = m_elevationGridConfig.grid_resolution;
                if (dr != 0 && dc != 0)
                {
                    dist *= std::sqrt(2.0f);  // 对角线
                }

                float slope = ComputeCellSlope(cell, neighbor, dist);
                max_slope = std::max(max_slope, slope);
            }

            cell.slope = max_slope;
        }
    }

    //── Debug: 坡度热力图 ──
    if (m_debugViewer)
    {
        m_debugViewer->DrawSlope(m_vGridCell, m_grid_rows, m_grid_cols, m_elevationGridConfig);
    }
}

float ElevationMapGroundFilter::ComputeCellSlope(
    const GridCell& a, const GridCell& b, float dist) const
{
    if (dist < 1e-6f)
    {
        return FLT_MAX;  // 除零保护
    }
    return std::abs(a.min_z - b.min_z) / dist;
}

// ============================================================================
// 五、RegionGrowing —— 区域生长
// ============================================================================

/**
 * 设计思想：
 *
 * 【Region Growing 流程图】
 *
 *   初始化:
 *   ┌──────────────────────────┐
 *   │ 选取种子 Cells（Seed）     │
 *   │ → 距离 LiDAR 最近一圈 Grid  │
 *   │ → 条件: valid && point_num  │
 *   │   足够                      │
 *   └──────────┬───────────────┘
 *              │
 *              ▼
 *   ┌──────────────────────────┐
 *   │ 将 Seeds 全部入队          │
 *   │ 标记 label = group_id     │
 *   └──────────┬───────────────┘
 *              │
 *              ▼
 *      ┌───────────────┐
 *      │ BFS Queue 非空？│──No──▶ 结束
 *      └───────┬───────┘
 *              │ Yes
 *              ▼
 *   ┌──────────────────────────┐
 *   │ 出队一个 Cell (r, c)       │
 *   └──────────┬───────────────┘
 *              │
 *              ▼
 *   ┌──────────────────────────┐
 *   │ 遍历 8 邻域 (nr, nc)       │
 *   └──────────┬───────────────┘
 *              │
 *              ▼
 *      ┌──────────────────┐
 *      │ 邻域 Cell 是否满足   │──No──▶ 继续下一个邻域
 *      │ 扩展条件？           │
 *      └──────┬───────────┘
 *             │ Yes
 *             ▼
 *   ┌──────────────────────────┐
 *   │ 标记 label, 入队          │
 *   └──────────────────────────┘
 *
 * 【扩展条件（多级门控）】
 *   1. Grid 有效（valid == true）
 *   2. 未访问（label == -1）
 *   3. 坡度满足要求（slope <= slope_threshold）
 *   4. 高度差满足要求（|Δz| <= height_diff_threshold）
 *   5. Point 数量满足要求（point_num >= min_points_per_cell）
 *
 * 【为什么 Region Growing 比逐列扫描鲁棒？】
 *   - 逐列扫描（如 ray_ground_filter）：按角度射线逐点判断，遇到障碍物后
 *     后面的地面点会被错误分类为障碍物（遮挡问题）
 *   - Region Growing：从已知地面种子出发，通过连通性扩展，不受扫描顺序影响
 *   - 对坡道：逐列扫描的"局部最低点"会随坡道漂移；Region Growing 通过坡度
 *     条件允许缓慢上升，但又阻止突变
 *   - 对孤立障碍物：逐列扫描无法区分"障碍物后面仍是地面"；Region Growing
 *     可以先绕过障碍物（通过其他路径），再回到障碍物后方
 *
 * 时间复杂度：O(R*C)，每个 cell 最多入队一次
 */

void ElevationMapGroundFilter::RegionGrowing()
{
    // ---- 获取种子 Cells ----
    std::vector<int> seeds = GetSeedCells();
    if (seeds.empty())
    {
        return;
    }

    // ---- BFS 初始化 ----
    std::queue<int> bfs_queue;
    int group_id = 0;

    for (int seed_idx : seeds)
    {
        GridCell& cell = m_vGridCell[seed_idx];
        if (cell.valid && cell.label == -1)
        {
            cell.label = group_id;
            bfs_queue.push(seed_idx);
        }
    }

    const auto& offsets = GetNeighborOffsets();

    // ---- BFS 主循环 ----
    while (!bfs_queue.empty())
    {
        int current_idx = bfs_queue.front();
        bfs_queue.pop();

        // 从线性索引反算行列
        int row = current_idx / m_grid_cols;
        int col = current_idx % m_grid_cols;

        const GridCell& current_cell = m_vGridCell[current_idx];

        // 遍历 8 邻域
        for (const auto& [dr, dc] : offsets)
        {
            // ================================================================
            // 尝试扩展：先尝试直接邻居(d=1)，若为Invalid则尝试跨越1~2个Invalid Cell
            // ================================================================
            for (int skip = 0; skip <= 2; ++skip)
            {
                int nr = row + (skip + 1) * dr;
                int nc = col + (skip + 1) * dc;

                if (!InBounds(nr, nc))
                {
                    break;  // 越界，不再尝试更远的 skip
                }

                int neighbor_idx = GridIndex(nr, nc);
                GridCell& neighbor = m_vGridCell[neighbor_idx];

                if (skip == 0)
                {
                    // ── 直接邻居（skip=0）：标准门控 ──
                    // (1) 有效
                    if (!neighbor.valid) continue;  // Invalid → 尝试 skip=1
                    // (2) 未访问
                    if (neighbor.label != -1) { break; }  // 已访问 → 不再尝试更远
                    // (3) 点数足够
                    if (neighbor.point_num < m_elevationGridConfig.min_points_per_cell) { break; }

                    // 获取动态阈值
                    float neighbor_cx, neighbor_cy;
                    GridIndexToWorld(neighbor_idx, neighbor_cx, neighbor_cy);
                    float dynamic_slope_thresh = GetDynamicSlopeThreshold(neighbor_cx);
                    float dynamic_height_diff_thresh = GetDynamicHeightDiffThreshold(neighbor_cx);

                    // (4) 坡度条件
                    if (neighbor.slope > dynamic_slope_thresh) { break; }

                    // (5) 高度差条件
                    float dist = m_elevationGridConfig.grid_resolution;
                    if (dr != 0 && dc != 0) dist *= std::sqrt(2.0f);
                    float height_diff = std::abs(current_cell.min_z - neighbor.min_z);
                    if (height_diff > dynamic_height_diff_thresh) { break; }

                    // ── 扩展成功 ──
                    neighbor.label = group_id;
                    bfs_queue.push(neighbor_idx);
                    break;  // 已成功扩展，不再尝试更远的 skip
                }
                // else
                // {
                //     // ── 跨越 skip 个 Invalid Cell（skip=1或2）──
                //     // 首先验证中间所有 cell 都是 Invalid 且未被插值标记过
                //     bool all_invalid = true;
                //     for (int k = 1; k <= skip; ++k)
                //     {
                //         int mid_r = row + k * dr;
                //         int mid_c = col + k * dc;
                //         if (!InBounds(mid_r, mid_c)) { all_invalid = false; break; }
                //         const GridCell& mid_cell = m_vGridCell[GridIndex(mid_r, mid_c)];
                //         if (mid_cell.valid || mid_cell.is_interpolated_ground || mid_cell.label >= 0)
                //         {
                //             all_invalid = false;
                //             break;
                //         }
                //     }
                //     if (!all_invalid) { break; }  // 中间有非Invalid → 停止此方向

                //     // 检查目标 cell
                //     if (!neighbor.valid) continue;  // 目标也Invalid → 尝试更大的 skip
                //     if (neighbor.label != -1) { break; }  // 已访问 → 停止
                //     if (neighbor.point_num < m_elevationGridConfig.min_points_per_cell) { break; }

                //     // 获取目标 cell 的动态阈值
                //     float neighbor_cx, neighbor_cy;
                //     GridIndexToWorld(neighbor_idx, neighbor_cx, neighbor_cy);
                //     float dynamic_slope_thresh = GetDynamicSlopeThreshold(neighbor_cx);
                //     float dynamic_height_diff_thresh = GetDynamicHeightDiffThreshold(neighbor_cx);

                //     // 坡度条件：使用跨越距离计算
                //     float dist = m_elevationGridConfig.grid_resolution * (skip + 1);
                //     if (dr != 0 && dc != 0) dist *= std::sqrt(2.0f);
                //     float jump_slope = std::abs(current_cell.min_z - neighbor.min_z) / dist;
                //     if (jump_slope > dynamic_slope_thresh) { break; }

                //     // 高度差条件
                //     float height_diff = std::abs(current_cell.min_z - neighbor.min_z);
                //     if (height_diff > dynamic_height_diff_thresh) { break; }

                //     // ── 跨越成功！标记中间 Invalid Cell 为 Interpolated Ground ──
                //     for (int k = 1; k <= skip; ++k)
                //     {
                //         int mid_r = row + k * dr;
                //         int mid_c = col + k * dc;
                //         int mid_idx = GridIndex(mid_r, mid_c);
                //         GridCell& mid_cell = m_vGridCell[mid_idx];
                //         mid_cell.is_interpolated_ground = true;
                //         mid_cell.label = group_id;
                //         // 线性插值估计 min_z（用于 GroundReference）
                //         float t = static_cast<float>(k) / static_cast<float>(skip + 1);
                //         mid_cell.min_z = current_cell.min_z + t * (neighbor.min_z - current_cell.min_z);
                //     }

                //     // 标记目标 cell 为真实 Ground
                //     neighbor.label = group_id;
                //     bfs_queue.push(neighbor_idx);
                //     break;  // 已成功扩展
                // }
            }
        }
    }
}

// ============================================================================
// 内部函数：获取种子 Cells
// ============================================================================

/**
 * 种子选取策略：
 *
 * 【默认：距离 LiDAR 最近的一圈 Grid】
 *   - LiDAR 安装在车顶，正下方最近一圈 Grid 最可能包含地面
 *   - 选取 x ∈ [x_min, x_min + N*resolution]（车前方最近几排）
 *   - 选取整行 y ∈ [y_min, y_max]
 *
 * 【为什么这样选？】
 *   - 车正下方的地面几乎 100% 是真实地面（车身遮挡极少）
 *   - 从这些种子出发，BFS 可以沿连续地面扩展到远处
 *   - 如果从 ROI 边缘（如 40m 远处）选种子，可能远处地面被障碍物遮挡
 *
 * 后续扩展：
 *   - 结合 IMU/里程计估计车身位姿，更精确地选取种子
 *   - 可以使用多帧信息辅助种子选取
 */
std::vector<int> ElevationMapGroundFilter::GetSeedCells() const
{
    std::vector<int> seeds;

    // 选取 ROI 前方最近 3 排 Grid 作为种子区域
    // 对应车前方 0~0.3m（3 * 0.1m 分辨率）
    constexpr int kSeedRows = 3;

    // 从 x_min 开始（车前方最近处）
    int start_col = 0;   //由于airy向下打，前几排点云畸变严重，前面构造修改了roi_min_x
    int end_col   = std::min(kSeedRows, m_grid_cols);
    float physical_start_x = m_effective_roi_x_min + start_col * m_elevationGridConfig.grid_resolution;
    float physical_end_x = m_effective_roi_x_min + end_col * m_elevationGridConfig.grid_resolution;
    LOG_RAW("选种子区域：第 %d 排开始, 第 %d 排结束 ; 对应距离：（%.2f, %.2f)\n", start_col, end_col, physical_start_x, physical_end_x);
    for (int row = 0; row < m_grid_rows; ++row)
    {
        for (int col = start_col; col < end_col; ++col)
        {
            int idx = GridIndex(row, col);
            const GridCell& cell = m_vGridCell[idx];

            if (cell.valid && cell.point_num >= m_elevationGridConfig.min_points_per_cell)
            {
                seeds.push_back(idx);
            }
        }
    }

    return seeds;
}

// ============================================================================
// 六、GenerateGroundMask —— 生成 Ground Label Map
// ============================================================================

/**
 * 设计思想：
 *
 * 【Ground Label Map 的作用】
 *   - 将 BFS 的访问标记（label >= 0）转换为语义标记（is_ground）
 *   - is_ground 表示"该 cell 属于 Ground Surface，可作为 GroundReference 计算"
 *   - 允许 is_ground 与 is_obstacle_candidate 同时为 true（Mixed Ground Cell）
 *   - 如果有多个 ground segment（不同 group_id），统一标记为 ground
 *   - 作为 Grid 级别的二值掩码，供 ComputeGroundReference 查询
 *
 * 【为什么很多自动驾驶项目都先 Label Grid，而不是直接 Label Point？】
 *   1. 计算效率：Grid 数量（~24,000 有效）<< 点数量（~53,000），
 *      在 Grid 上做 Region Growing 比在点上做快 10× 以上
 *   2. 拓扑明确：Grid 之间的邻接关系是规则网格，不需要 KD-Tree
 *   3. 鲁棒性：Grid 级别的统计量（min_z、point_num）比单点更稳定
 *   4. 可扩展：Grid 是 PMF、CSF、CNN-based 方法的统一输入格式
 *   5. 后处理：可以在 Grid 上做形态学闭运算填补空洞、开运算去除孤立 ground
 */

void ElevationMapGroundFilter::GenerateGroundMask()
{
    int ground_grid_count = 0;
    for (auto& cell : m_vGridCell)
    {
        // label >= 0 表示 BFS 访问过 → 属于地面区域
        cell.is_ground = (cell.valid && cell.label >= 0);
        if(cell.is_ground)
            ground_grid_count++;
    }

    //用来看地面最远的grid（x方向）//想在点云图中可视化出一条最远地面的grid中心表示的线
    std::vector<std::pair<float,float>> all_Grid_Line;
    float ground_cell_max_x = m_effective_roi_x_min;
    int ground_cell_max_row = 0;
    int ground_cell_max_col = 0;
    for (int row = 0; row < m_grid_rows; ++row)  //从 y_min（最右边，-）开始，一直遍历到 y_max（最左边，+）
    {
        for (int col = m_grid_cols -1 ; col >= 0; --col) //从 x_max（车前方/远处）开始，一直遍历到 x_min（车后方/近处）
        {
            int idx = GridIndex(row, col);
            GridCell& cell = m_vGridCell[idx];
            if(cell.is_ground){
                // X 轴物理坐标 (前向)
                float ground_line_physical_x = m_effective_roi_x_min + col * m_elevationGridConfig.grid_resolution;
                
                // Y 轴物理坐标 (左右)
                float line_physical_y = m_elevationGridConfig.roi_y_min + row * m_elevationGridConfig.grid_resolution;
                
                // 存入容器
                auto line_xy = std::make_pair(ground_line_physical_x, line_physical_y);
                all_Grid_Line.push_back(line_xy);

                if(ground_line_physical_x >= ground_cell_max_x)
                {
                    ground_cell_max_x = ground_line_physical_x;
                    ground_cell_max_row = row;
                    ground_cell_max_col = col;
                }
            }

        }
    }

    //防止越界
    if(ground_cell_max_col == (m_grid_cols -1)){
        LOG_RAW("[GenerateGroundMask]ground cell size: %d -》rest cell size: %d ; ground max X: %.2f\n", ground_grid_count, m_vGridCell.size()-ground_grid_count, ground_cell_max_x);

    }
    else{
        int x_first_noGround_idx = ground_cell_max_row * m_grid_cols + (ground_cell_max_col+1);
        GridCell& cell = m_vGridCell[x_first_noGround_idx];

        float first_no_ground_line_physical_x = m_effective_roi_x_min + (ground_cell_max_col+1) * m_elevationGridConfig.grid_resolution;
                
        LOG_RAW("[GroundMask]ground cell size: %d -》ground max X: %.2f; No ground min X: %.2f\n", ground_grid_count, m_vGridCell.size()-ground_grid_count, ground_cell_max_x, first_no_ground_line_physical_x);
        // LOG_RAW("   -》\n", cell.min_z, cell.ground_reference_z, cell.top_height);
    }

    // LOG_RAW("[GenerateGroundMask]ground cell size: %d -》rest cell size: %d ; ground max X: %.2f\n", ground_grid_count, m_vGridCell.size()-ground_grid_count, ground_cell_max_x);

    //── Debug: Ground Mask 分类图 ──
    if (m_debugViewer)
    {
        m_debugViewer->DrawGroundMask(m_vGridCell, m_grid_rows, m_grid_cols, m_elevationGridConfig);
    }
}

// ============================================================================
// 七、SplitPointCloud —— Point Label & 分离
// ============================================================================

/**
 * 设计思想：
 *
 * 【再次遍历 PointCloud，根据 Grid 标记分离】
 *   - 每个点通过 (x, y) → (row, col) 快速查询对应 Grid
 *   - 如果 Grid 被标记为 ground → 加入 ground_cloud
 *   - 否则 → 加入 obstacle_cloud
 *
 * 【时间复杂度分析】
 *   - 遍历 PointCloud: O(N)
 *   - 每次 WorldToGrid 查询: O(1)（纯算术运算，无 KD-Tree）
 *   - 总体: O(N)
 *
 * 【为什么不把 Ground Label 存到 Point 里？】
 *   再次强调：
 *   - 避免在 BFS 中对每个 Point 操作（O(N) × 8 邻域 → O(8N*logN)）
 *   - 保持 Point 类型不变（pcl::PointXYZI），不引入额外的 label 字段
 *   - 如需在 Point 上保留 label，可以在输出阶段添加 intensity 字段编码
 *
 * 【为什么一定要 Ground Filter 之后再聚类？】
 *   - 如果不做 Ground Filter：
 *     1. 地面点占全点云 60%~80%，会参与 Euclidean Cluster
 *     2. 整个地面（包括坡道）是一个巨大连通分量
 *     3. 地面上的障碍物与地面连通，形成超级 Cluster
 *     4. 聚类算法无法区分"坡道"和"障碍物"
 *     5. 坡道形成巨大 Cluster → 所有物体被合并 → 检测失败
 *   - Ground Filter 后：
 *     ObstacleCloud 中只有悬空点，障碍物自然分离
 *     Euclidean Cluster 可以正确区分每个障碍物
 */
/*
注意: 此函数仅被旧版 Process() 使用。
ProcessWithObstacleDetection / ProcessWithObstacleTracking 已改用
AnalyzeVerticalOccupancy 内部的 ReclassifyPointCloud，
它可以正确处理 Mixed Ground Cell (is_ground && is_obstacle_candidate) 的情况。
*/
void ElevationMapGroundFilter::SplitPointCloud(
    const pcl::PointCloud<pcl::PointXYZI>& cloud,
    pcl::PointCloud<pcl::PointXYZI>&       ground_cloud,
    pcl::PointCloud<pcl::PointXYZI>&       obstacle_cloud)
{
    ground_cloud.clear();
    obstacle_cloud.clear();

    // 预留容量（减少 reallocation）
    ground_cloud.reserve(cloud.size());
    obstacle_cloud.reserve(cloud.size());

    // float compare_ground_grid_min_x = m_elevationGridConfig.roi_x_max;
    
    pcl::PointXYZI temp_point;
    for (const auto& point : cloud.points)
    {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
        {
            continue;
        }

        int row, col;
        if (!WorldToGrid(point.x, point.y, row, col))
        {
            // 超出 ROI 的点默认归入障碍物（可能是远处物体）
            obstacle_cloud.push_back(point);
            continue;
        }

        const GridCell& cell = m_vGridCell[GridIndex(row, col)];

        if (cell.valid && cell.is_ground)
        {
            ground_cloud.push_back(point);
            // if(point.x < compare_ground_grid_min_x){
            //     compare_ground_grid_min_x = point.x;
            //     temp_point = point;
            // }
                
        }
        else
        {
            obstacle_cloud.push_back(point);
        }
    }
}

// ============================================================================
// 八、ProcessWithObstacleDetection —— 完整流程: Ground Filter + 低矮障碍物检测
// ============================================================================

/**
 * 设计思想：
 *
 * 【为什么需要 ProcessWithObstacleDetection？】
 *   原有的 Process() 只输出 Ground/Obstacle 点云，无法区分"高大障碍物"和"低矮障碍物"。
 *   ProcessWithObstacleDetection 在 Ground Filter 之后增加了:
 *     7. AnalyzeVerticalOccupancy  → 体素占据分析
 *     8. ClusterObstacleGrid       → 障碍物聚类
 *   最终输出 GridCluster 列表，可直接转换为下游接口所需的 S2obstacleBox。
 *
 * 【为什么不在 Process() 中直接修改？】
 *   保持原有 Process() 接口不变，保证已有调用方的兼容性。
 *   新增 ProcessWithObstacleDetection() 作为增强版接口。
 *
 * 【调用顺序保证】
 *   必须按此顺序调用:
 *     1~6: 与 Process() 相同的 Ground Filter 流程
 *     7:   AnalyzeVerticalOccupancy → 必须在 SplitPointCloud 之后？
 *           不，AnalyzeVerticalOccupancy 只需要 m_vGridCell 中的 min_z/max_z 数据，
 *           以及原始点云用于构建 layer_histogram
 *     8:   ClusterObstacleGrid → 依赖 AnalyzeVerticalOccupancy 的判定结果
 */
bool ElevationMapGroundFilter::ProcessWithObstacleDetection(
    const PointCloud2Intensity::Ptr& cloud,
    PointCloud2Intensity::Ptr&       ground_cloud,
    PointCloud2Intensity::Ptr&       obstacle_cloud,
    std::vector<GridCluster>&        clusters)
{
    if (!cloud || cloud->empty())
    {
        return false;
    }

    auto t_start = std::chrono::steady_clock::now();
    
    // Step 1~6: 与 Process() 相同的 Ground Filter 流程
    if (!BuildGrid(*cloud))
    {
        return false;
    }

    ComputeGroundHeight();
    ComputeSlope();
    RegionGrowing();
    GenerateGroundMask();

    auto t_end_1_ = std::chrono::steady_clock::now();
    auto duration_1 = std::chrono::duration_cast<std::chrono::milliseconds>(t_end_1_ - t_start);
         

    // ────────────────────────────────────────────────
    // Step 6.5: 从 Ground Grid 估计参考地面高度
    // ────────────────────────────────────────────────
    // 必须在 SplitPointCloud 之前调用（Grid 数据完整），
    // 必须在 AnalyzeVerticalOccupancy 之前调用（需要 ground_reference_z）
    ComputeGroundReference();

    // SplitPointCloud(*cloud, *ground_cloud, *obstacle_cloud);

    auto t_end_2_ = std::chrono::steady_clock::now();
    auto duration_2 = std::chrono::duration_cast<std::chrono::milliseconds>(t_end_2_ - t_end_1_);
         

    // LOG_RAW("[第一阶段结束] GroundReference完成; \n" \
    //         "       Process过程: %lld ms, GroundReference过程: %lld ms\n",
    //     duration_1.count(), duration_2.count());

    // ────────────────────────────────────────────────
    // Step 7: Voxel Occupancy Analysis（体素占据分析）
    //         内部完成: layer_histogram 构建 + is_obstacle_candidate 判定
    //                   + 基于判定结果的点云重分类（替代 SplitPointCloud）
    // ────────────────────────────────────────────────
    // AnalyzeVerticalOccupancy 综合 top_height + height_range + occupied_layers
    // 判定 is_obstacle_candidate，区分低矮障碍物和高大障碍物
    AnalyzeVerticalOccupancy(*cloud, *ground_cloud, *obstacle_cloud);
    auto t_end_3_ = std::chrono::steady_clock::now();
    auto duration_3 = std::chrono::duration_cast<std::chrono::milliseconds>(t_end_3_ - t_end_2_);
         
    LOG_RAW("----[VoxelOccupancy v2 + Reclassify] use time: %lld ms, "
            "Ground: %zu, Obstacle: %zu\n",
            duration_3.count(), ground_cloud->size(), obstacle_cloud->size());

    // ────────────────────────────────────────────────
    // Step 8: Obstacle Grid Clustering（障碍物聚类）
    // ────────────────────────────────────────────────
    clusters = ClusterObstacleGrid();
    auto t_end_4_ = std::chrono::steady_clock::now();
    auto duration_4 = std::chrono::duration_cast<std::chrono::milliseconds>(t_end_4_ - t_end_3_);
         
    LOG_RAW("[ClusterObstacle] total clusters: %zu (filtered >= %d cells), use time: %lld ms\n" \
    "****************\n",
        clusters.size(), m_elevationGridConfig.min_cluster_cells, duration_4.count());

    // ── Debug: 聚类 Label 图 ──
    if (m_debugViewer)
    {
        m_debugViewer->DrawCluster(m_vGridCell, m_grid_rows, m_grid_cols, m_elevationGridConfig);
    }

    // ── Debug: 包围盒俯视图 ──
    if (m_debugViewer)
    {
        m_debugViewer->DrawBoundingBox(clusters, m_elevationGridConfig);
    }

    // // ── Debug: 最终点云叠加图 ──
    // if (m_debugViewer)
    // {
    //      8-27日换了
    //     m_debugViewer->DrawOverlay(*ground_cloud, *obstacle_cloud, clusters, m_elevationGridConfig);
    // }

    return true;
}

// ============================================================================
// 九-A、ComputeGroundReference —— 从 Ground Grid 估计参考地面高度
// ============================================================================

/**
 * 设计思想：
 *
 * 【为什么不能直接用 sensor_height（如 -1.25m）作为 ground_reference_z？】
 *   1. 车辆可能在上坡/下坡/斜坡停车 → 地面高度随 x 变化
 *   2. 激光雷达安装高度有 ±5cm 误差
 *   3. 3.5m 以远激光点云会逐渐上翘（即使是平面，z 值也会 > -1.25m）
 *   4. 车外可能有向下凹的楼梯/检修平台（z < -1.25m），这些不是地面参考
 *
 * 【为什么用"每列 ground cell 中位数"而不是"固定值"？】
 *   1. RegionGrowing 已经输出大量连续的 Ground Grid，质量可靠
 *   2. 中位数对离群值（如个别误标 ground 的障碍物 cell）鲁棒
 *   3. 逐列计算可以自动跟随坡道、起伏路面
 *
 * 【为什么需要插值？】
 *   某些列可能完全没有 ground cell（如被大型障碍物完全遮挡），
 *   此时通过前后有效列的 ground reference 线性插值填补。
 *
 * 【为什么需要轻量平滑？】
 *   单列可能因为地面点云稀疏导致中位数跳动，
 *   3 点滑动窗口平滑消除了这种高频噪声，同时保留了坡道的低频趋势。
 *
 * 【与 RegionGrowing 的关系】
 *   完全复用 RegionGrowing 的结果（cell.is_ground = label >= 0），
 *   不做任何额外的点云遍历或地面分割。仅做 O(Grid) 的后处理统计。
 *
 * 时间复杂度：O(m_grid_cols * m_grid_rows)
 */

void ElevationMapGroundFilter::ComputeGroundReference()
{
    const int cols = m_grid_cols;    // x→col (前向)
    const int rows = m_grid_rows;    // y→row (左右)

    // ================================================================
    // Phase 1: 逐列收集 ground cell 的 min_z
    // ================================================================
    // column_ground_zs[col] = 该列所有 ground cell 的 min_z 值列表

    std::vector<std::vector<float>> column_ground_zs(cols);
    for (int col = 0; col < cols; ++col)
    {
        column_ground_zs[col].reserve(static_cast<size_t>(rows));
    }

    for (int row = 0; row < rows; ++row)
    {
        for (int col = 0; col < cols; ++col)
        {
            const GridCell& cell = m_vGridCell[GridIndex(row, col)];
            //if ((cell.valid && cell.is_ground) //
            if ((cell.valid && cell.is_ground) || cell.is_interpolated_ground)
            {
                column_ground_zs[col].push_back(cell.min_z);
            }
        }
    }

    // LOG_RAW("[Trace-Phase1] Col33 collected %zu ground cells.\n", column_ground_zs[33].size());
    // LOG_RAW("[Trace-Phase1] Col34 collected %zu ground cells.\n", column_ground_zs[34].size());
    // LOG_RAW("[Trace-Phase1] Col35 collected %zu ground cells.\n", column_ground_zs[35].size());

    // ================================================================
    // Phase 2: 每列计算 Robust 代表高度（中位数）
    // ================================================================
    // column_ref[col]:
    //   - 有效: 该列 ground cell min_z 的中位数
    //   - 无效: FLT_MAX（标记待插值）

    std::vector<float> column_ref(cols, FLT_MAX);

    for (int col = 0; col < cols; ++col)
    {
        auto& zs = column_ground_zs[col];
        if (zs.empty())
        {
            continue;
        }

        // 中位数: 对离群值鲁棒（如个别误标为 ground 的障碍物 cell）
        std::sort(zs.begin(), zs.end());
        size_t n = zs.size();
        if (n % 2 == 1)
        {
            column_ref[col] = zs[n / 2];
        }
        else
        {
            column_ref[col] = (zs[n / 2 - 1] + zs[n / 2]) * 0.5f;
        }
    }


    // // [新增追踪日志] Phase 2 结束：查看平滑前的原始中位数
    // LOG_RAW("[Trace-Phase2] Col33 Raw Median = %.4f\n", column_ref[33]);
    // LOG_RAW("[Trace-Phase2] Col34 Raw Median = %.4f\n", column_ref[34]);
    // LOG_RAW("[Trace-Phase2] Col35 Raw Median = %.4f\n", column_ref[35]);

    // ================================================================
    // Phase 3: 插值填补无效列
    // ================================================================
    // 策略: 从左到右扫描, 用最近的有效列填充无效列
    //   如果前后都有有效列 → 线性插值
    //   如果只有一侧有有效列 → 直接用该侧的值
    //   如果整行都没有有效列 → 保持 FLT_MAX (后续 fallback)

    // 先找第一个有效列
    int first_valid = -1;
    for (int col = 0; col < cols; ++col)
    {
        if (column_ref[col] != FLT_MAX)
        {
            first_valid = col;
            break;
        }
    }

    // 找最后一个有效列
    int last_valid = -1;
    for (int col = cols - 1; col >= 0; --col)
    {
        if (column_ref[col] != FLT_MAX)
        {
            last_valid = col;
            break;
        }
    }

    if (first_valid >= 0 && last_valid >= 0)
    {
        // 左侧外推: 第一个有效列之前的所有列用第一个有效列的值
        for (int col = 0; col < first_valid; ++col)
        {
            column_ref[col] = column_ref[first_valid];
        }

        // 右侧外推: 最后一个有效列之后的所有列用最后一个有效列的值
        for (int col = last_valid + 1; col < cols; ++col)
        {
            column_ref[col] = column_ref[last_valid];
        }

        // 中间线性插值
        int prev_valid = first_valid;
        for (int col = first_valid + 1; col <= last_valid; ++col)
        {
            if (column_ref[col] != FLT_MAX)
            {
                // 当前列有效，更新 prev_valid
                prev_valid = col;
            }
            else
            {
                // 找下一个有效列
                int next_valid = col;
                while (next_valid <= last_valid && column_ref[next_valid] == FLT_MAX)
                {
                    next_valid++;
                }
                if (next_valid > last_valid) break;  // 不会发生，但防御

                // 线性插值: prev_valid 到 next_valid
                float z_start = column_ref[prev_valid];
                float z_end   = column_ref[next_valid];
                int   span    = next_valid - prev_valid;

                for (int c = prev_valid + 1; c < next_valid; ++c)
                {
                    float t = static_cast<float>(c - prev_valid) / static_cast<float>(span);
                    column_ref[c] = z_start + (z_end - z_start) * t;
                }

                col = next_valid - 1;  // 循环末尾 ++col 后到 next_valid
            }
        }
    }

    // // [新增追踪日志] Phase 3 结束：查看插值后、平滑前的值
    // LOG_RAW("[Trace-Phase3] Col33 After Interp = %.4f\n", column_ref[33]);
    // LOG_RAW("[Trace-Phase3] Col34 After Interp = %.4f\n", column_ref[34]);
    // LOG_RAW("[Trace-Phase3] Col35 After Interp = %.4f\n", column_ref[35]);

    // ================================================================
    // Phase 4: Fallback → 使用 sensor_height_nominal（取反）
    // ================================================================
    // 如果完全没有 ground cell（极端情况），所有列用标称高度
    // const float fallback_z = -m_elevationGridConfig.sensor_height_nominal;    //默认是sensor_height_nominal为正
    const float fallback_z = m_elevationGridConfig.sensor_height_nominal;
    // LOG_RAW("in ComputeGroundReference: 主雷达安装高度sensor_height_nominal-> %.3f, 取反fallback_z = %.3f\n",
    //     m_elevationGridConfig.sensor_height_nominal, fallback_z);
    
    // 仍为 FLT_MAX 的列用 fallback
    for (int col = 0; col < cols; ++col)
    {
        if (column_ref[col] == FLT_MAX)
        {
            float cx = m_effective_roi_x_min + (static_cast<float>(col) + 0.5f) * m_elevationGridConfig.grid_resolution;
            LOG_RAW("***GroundRef-col%d，对应cell中心位置: %.2f, use 默认高度!!!\n",col , cx);
            column_ref[col] = fallback_z;
            
        }
    }

    // ================================================================
    // Phase 5: 轻量平滑（3 点滑动窗口）
    // ================================================================
    // 消除单列统计噪声，同时保留坡道的低频趋势

    std::vector<float> smoothed_ref = column_ref;  // 副本

    for (int col = 1; col < cols - 1; ++col)
    {
        // 仅当相邻列都存在有效 ground 数据时才平滑
        // (避免插值列之间的过度平滑)
        smoothed_ref[col] = (column_ref[col - 1] + column_ref[col] + column_ref[col + 1]) / 3.0f;
    }
    // 首尾列不参与平滑（保持原值）

    column_ref = std::move(smoothed_ref);

    // ================================================================
    // Phase 6: 将 column_ref 分发到每个 GridCell
    // ================================================================

    for (int row = 0; row < rows; ++row)
    {
        for (int col = 0; col < cols; ++col)
        {
            GridCell& cell = m_vGridCell[GridIndex(row, col)];
            if (!cell.valid) continue;

            cell.ground_reference_z = column_ref[col];

            // 同步计算 top_height
            cell.top_height = cell.max_z - cell.ground_reference_z;
        }
    }

    // 诊断日志
    int ground_cols = 0;
    for (int col = 0; col < cols; ++col)
    {
        if (!column_ground_zs[col].empty()){
            ground_cols++;}
    }

    // int idx_1 = GridIndex(row, 0);
    // GridCell& cell1 = m_vGridCell[idx];

    // LOG_RAW("[GroundReference] %d/%d columns have ground cells, "
    //         "地面高度参考范围: [%.3f, %.3f], 高度平均值范围: [%.3f, %.3f], 主雷达安装高度fallback: %.2f\n",
    //         ground_cols, cols,
    //         column_ref[0], column_ref[cols - 1], m_vGridCell[] fallback_z);
    
    

    // LOG_RAW("[GroundRef-NearCols] ");
    // for (int col = 0; col < std::min(10, cols); ++col){
    //     float cx = m_effective_roi_x_min + (col+0.5f)*m_elevationGridConfig.grid_resolution;
    //     LOG_RAW("col%d(x=%.2f): ref=%.3f  ", col, cx, column_ref[col]);
    // }
    // LOG_RAW("\n");


    //测试4m位置的10*10*10为什么看不见
    for (int col = 30; col < std::min(33, cols); ++col) {
    auto& zs = column_ground_zs[col];
        if (!zs.empty()) {
            std::sort(zs.begin(), zs.end());
            // LOG_RAW("[Test-GroundRef-col%d] n=%zu min=%.3f median=%.3f max=%.3f\n",
            //         col, zs.size(), zs.front(), 
            //         zs[zs.size()/2], zs.back());
        }
    }




    // ── Debug: Ground Reference 热力图 ──
    if (m_debugViewer)
    {
        m_debugViewer->DrawReference(m_vGridCell, m_grid_rows, m_grid_cols, m_elevationGridConfig);
    }
}

// ============================================================================
// 九、AnalyzeVerticalOccupancy —— 体素占据分析（基于 Ground Reference 重设计）
// ============================================================================

/**
 * 设计思想（v2 重设计）：
 *
 * 【为什么 v1 的 height_range 判定有严重缺陷？】
 *   v1 仅依据 height_range = max_z - min_z 判断 obstacle_candidate。
 *   例如: 箱子 min_z=-0.75, max_z=-0.55 → height_range=0.20m
 *   但雷达安装高度约 1.25m，箱子顶部距地面约 70cm，属于高大障碍物，本项目无需检测。
 *   相反: 石块 min_z=-1.25, max_z=-1.08 → height_range≈0.17m
 *   石块顶部距地面约 17cm，才是真正需要检测的低矮目标。
 *   仅靠 height_range 无法区分这两者！
 *
 * 【v2 核心改进：Ground Reference】
 *   - ground_reference_z: 由 ComputeGroundReference() 从 Ground Grid 自动估计，
 *     跟随坡道/起伏路面变化，而非固定 -1.25m
 *   - top_height = max_z - ground_reference_z: 障碍物顶部距参考地面的真实高度
 *   - 综合 top_height + height_range + occupied_layers 共同判定
 *
 * 【为什么同时保留 top_height 和 height_range？】
 *   - top_height: 从"地面"到"障碍物顶部"的高度（外部视角）
 *   - height_range: 障碍物自身的高度跨度（内部视角）
 *   两者表达不同语义:
 *     - 平放在地面的石块: top_height≈15cm, height_range≈15cm → 一致
 *     - 悬空低矮物体: top_height≈40cm, height_range≈5cm → 不一致, 可能不是障碍物
 *     - 下沉区域(楼梯): top_height<0 → 在参考地面以下, 过滤
 *
 * 【判定规则（多级门控，自上而下）】
 *
 *   ┌─ Rule 0: 下沉/凹陷区域 ────────────────────────────────────┐
 *   │  top_height < 0  (即 max_z < ground_reference_z)             │
 *   │  → 目标整体在参考地面以下（向下楼梯/检修坑/下凹平台）          │
 *   │  → 直接过滤，不参与任何候选判定                               │
 *   └─────────────────────────────────────────────────────────────┘
 *
 *   ┌─ Rule 1: 竖直结构 ─────────────────────────────────────────┐
 *   │  height_range > vertical_structure_height_min (0.5m)         │
 *   │  && occupied_layers <= max_occupied_layers_vertical          │
 *   │  → 杆子/树干/墙壁边缘 (高度大 + 占据稀疏)                     │
 *   └─────────────────────────────────────────────────────────────┘
 *
 *   ┌─ Rule 2: 低矮障碍物候选 (核心) ─────────────────────────────┐
 *   │  分段判定:                                                   │
 *   │    近距离 (cell x < near_range_boundary, 默认 3m):           │
 *   │      top_height ∈ [top_height_min_near, top_height_max_near] │
 *   │      推荐 5~40cm (近距离点云密集, 可检测更低矮目标)            │
 *   │    中距离 (cell x >= near_range_boundary):                   │
 *   │      top_height ∈ [top_height_min_far, top_height_max_far]   │
 *   │      推荐 10~50cm (中距离点云稀疏, 太小不可靠)                │
 *   │  && height_range >= 0.02m (至少有 2cm 自身厚度, 过滤贴地噪点) │
 *   │  && occupied_layers >= min_occupied_layers_obstacle          │
 *   │  && !is_vertical_structure                                   │
 *   │  && top_height > 0 (确保在参考地面以上)                       │
 *   └─────────────────────────────────────────────────────────────┘
 *
 * 【自然过滤效果】
 *   高大箱子: top_height ≈ 70cm → 超出 top_height_max → 过滤 ✓
 *   墙体:     top_height ≫ 50cm → 超出 → 过滤 ✓
 *   行人腿部: top_height ≈ 30~80cm, occupied_layers 稀疏 → 过滤 ✓
 *   大型设备: top_height ≫ 50cm → 过滤 ✓
 *   向下楼梯: top_height < 0 → Rule 0 过滤 ✓
 *   下凹平台: top_height < 0 → Rule 0 过滤 ✓
 *   检修坑:   top_height < 0 → Rule 0 过滤 ✓
 *
 * 【为什么不直接在 BuildGrid 中构建 layer_histogram？】
 *   1. BuildGrid 的职责是"建立 Grid 并统计基础信息"，保持单一职责
 *   2. layer_histogram 需要知道 min_z 才能计算层偏移，而 min_z 在 BuildGrid 后才确定
 *   3. 分离为独立步骤，方便调试和参数调优
 *   4. 未来可选择性启用（如只在特定场景开启低矮障碍物检测）
 */
void ElevationMapGroundFilter::AnalyzeVerticalOccupancy(
    const pcl::PointCloud<pcl::PointXYZI>& cloud,
    pcl::PointCloud<pcl::PointXYZI>&       ground_cloud,
    pcl::PointCloud<pcl::PointXYZI>&       obstacle_cloud)
{
    if (cloud.empty())
    {
        return;
    }

    const float layer_res = m_elevationGridConfig.layer_resolution;

    // ================================================================
    // Phase 1: 为每个 valid cell 初始化 layer_histogram 并计算 height_range
    // ================================================================
    //
    // num_layers = ceil(height_range / layer_resolution)
    // 注意: top_height 和 ground_reference_z 已在 ComputeGroundReference() 中计算
    // height_range 在 BuildGrid 中已有 max_z/min_z 数据, 在此计算

    for (auto& cell : m_vGridCell)
    {
        if (!cell.valid) continue;

        cell.height_range = cell.max_z - cell.min_z;

        int num_layers = static_cast<int>(
            std::ceil(cell.height_range * (1.0f / layer_res)));
        if (num_layers < 1) num_layers = 1;

        constexpr int kMaxLayers = 200;
        if (num_layers > kMaxLayers) num_layers = kMaxLayers;

        //将
        cell.layer_histogram.assign(static_cast<size_t>(num_layers), 0);
    }

    // ================================================================
    // Phase 2: 遍历点云，填充 layer_histogram
    // ================================================================
    //
    // layer_idx = floor((point.z - cell.min_z) / layer_resolution)
    // 边界: point.z == max_z → 归入最后一层

    for (const auto& point : cloud.points)
    {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
            continue;

        int row, col;
        if (!WorldToGrid(point.x, point.y, row, col)) continue;

        GridCell& cell = m_vGridCell[GridIndex(row, col)];
        if (!cell.valid || cell.layer_histogram.empty()) continue;

        int layer_idx = static_cast<int>(
            std::floor((point.z - cell.min_z) * (1.0f / layer_res))); //向下取整就知道在第几层

        int max_layer = static_cast<int>(cell.layer_histogram.size()) - 1;
        if (layer_idx < 0) layer_idx = 0;
        if (layer_idx > max_layer) layer_idx = max_layer;

        if (cell.layer_histogram[static_cast<size_t>(layer_idx)] < 255)
        {
            cell.layer_histogram[static_cast<size_t>(layer_idx)]++;
        }
    }

    // ================================================================
    // Phase 3: 判定 is_vertical_structure / is_obstacle_candidate
    //          核心改进: 综合 top_height + height_range + occupied_layers
    // ================================================================
    //
    // occupied_layers: 统计 layer_histogram 中非零层数
    //   这反映了 Z 方向的占据密度
    //
    // 判定逻辑（多级门控）:
    //
    //   对于每个 valid cell:
    //     1. 计算 occupied_layers
    //     2. Rule 0: top_height < 0 → 下沉区域, 直接过滤
    //     3. Rule 1: 竖直结构 (高度大 + 占据稀疏)
    //     4. Rule 2: 低矮障碍物候选 (top_height 在合适范围 + 占据合理)
    //     5. 其他 → 都不是
    //
    // 【为什么使用动态 top_height 阈值？】
    //   近距离 (x<3m): 点云密集, 5cm 的石块也能可靠检测
    //   中距离 (3~6m): 点云稀疏, 需要至少 10cm 才有足够点数支撑判定
    //   阈值随距离线性变化, 避免近处漏检+远处误检

    // 获取 cell 中心 x 坐标的辅助 lambda
    auto getCellCenterX = [this](int idx) -> float {
        // idx → (row, col) → x
        int col = idx % m_grid_cols;
        return m_effective_roi_x_min +
               (static_cast<float>(col) + 0.5f) * m_elevationGridConfig.grid_resolution;
    };

    for (auto& cell : m_vGridCell)
    {
        if (!cell.valid || cell.layer_histogram.empty()) continue;

        // ── 统计 occupied_layers ──
        cell.occupied_layers = 0;
        for (const auto& count : cell.layer_histogram)
        {
            if (count > 0) cell.occupied_layers++;
        }

        // 重置标记
        cell.is_vertical_structure = false;
        cell.is_obstacle_candidate = false;

        // ────────────────────────────────────────────────────────────
        // Rule 0: 下沉/凹陷区域过滤
        // ────────────────────────────────────────────────────────────
        // top_height < 0 表示 max_z < ground_reference_z,
        // 即目标整体在参考地面以下（向下楼梯/下凹平台/检修坑）
        // 这类目标不是本项目的检测对象，直接跳过
        if (cell.top_height < 0.0f)
        {
            continue;
        }

        // ────────────────────────────────────────────────────────────
        // Rule 1: 竖直结构 (杆子/树干/墙壁边缘)
        // ────────────────────────────────────────────────────────────
        // 条件: 自身高度跨度大 (>0.5m) 但占据层稀疏 (≤3 层)
        // 这类目标虽然 top_height 可能很大, 但通过 occupied_layers 稀疏特征识别
        if (cell.height_range > m_elevationGridConfig.vertical_structure_height_min &&
            cell.occupied_layers <= m_elevationGridConfig.max_occupied_layers_vertical)
        {
            cell.is_vertical_structure = true;
            continue;
        }

        // ────────────────────────────────────────────────────────────
        // Rule 2: 低矮障碍物候选 (核心判定)
        // ────────────────────────────────────────────────────────────
        // 判定依据 (三项共同参与):
        //   (a) top_height 在合适范围 (分段: 近距离 5~40cm, 中距离 10~50cm)
        //   (b) height_range 有最少自身厚度 (≥2cm, 过滤贴地噪点)
        //   (c) occupied_layers 足够密集 (≥2 层, 过滤单层噪点)
        //
        // 【为什么还需要 height_range 约束？】
        //   悬空物（如低垂的树枝）可能 top_height 在范围内,
        //   但 height_range 极小（只有一层点）, 不是障碍物。
        //   height_range ≥ 0.02m 确保目标有至少 2cm 的自身厚度。

        // 获取 cell 的 x 坐标用于分段判定
        // 注意: 这里需要知道 cell 在 grid 中的位置
        // 遍历所有 cell 时无法直接获取 idx，改用 row/col 遍历
    }

    // ────────────────────────────────────────────────────────────────
    // 第二次遍历: 对每个 cell 执行 top_height 分段判定
    // (需要 cell 的 col 索引来确定 x 坐标)
    // ────────────────────────────────────────────────────────────────

    const float near_boundary = m_elevationGridConfig.near_range_boundary;

    for (int row = 0; row < m_grid_rows; ++row)
    {
        for (int col = 0; col < m_grid_cols; ++col)
        {
            int idx = GridIndex(row, col);
            GridCell& cell = m_vGridCell[idx];

            // 只处理尚未标记的 valid cell
            if (!cell.valid || cell.layer_histogram.empty()) continue;
            if (cell.is_vertical_structure) continue;   // Rule 1 已标记
            if (cell.top_height < 0.0f) continue;        // Rule 0 已过滤

            // 计算 cell 中心 x 坐标
            float cx = m_effective_roi_x_min +
                       (static_cast<float>(col) + 0.5f) * m_elevationGridConfig.grid_resolution;

            // 分段选择 top_height 阈值
            float top_min, top_max;
            if (cx < near_boundary)
            {
                // 近距离: 点云密集, 可检测 5~40cm 低矮障碍物
                top_min = m_elevationGridConfig.obstacle_top_height_min_near;
                top_max = m_elevationGridConfig.obstacle_top_height_max_near;
            }
            else
            {
                // 中距离: 点云稀疏, 检测 10~50cm 障碍物
                top_min = m_elevationGridConfig.obstacle_top_height_min_far;
                top_max = m_elevationGridConfig.obstacle_top_height_max_far;
            }

            // ── 综合判定 ──
            // (a) top_height 在分段范围内
            bool top_ok = (cell.top_height >= top_min && cell.top_height <= top_max);

            // (b) height_range 有最少自身厚度 (≥2cm)
            constexpr float kMinHeightRange = 0.02f;
            bool range_ok = (cell.height_range >= kMinHeightRange);

            // (c) occupied_layers 足够密集
            bool layers_ok = (cell.occupied_layers >= m_elevationGridConfig.min_occupied_layers_obstacle);

            if (top_ok && range_ok && layers_ok)
            {
                cell.is_obstacle_candidate = true;

                // ── 调试：打印同时是 ground 的 obstacle_candidate ──
                if (cell.is_ground)
                {
                    // LOG_RAW("[DEBUG-GroundAsObs] row=%d col=%d x=%.2f y=%.2f "
                    //         "min_z=%.3f max_z=%.3f top_h=%.3f h_range=%.3f "
                    //         "ground_ref=%.3f occ_layers=%d pt_num=%d\n",
                    //         row, col, cx, 
                    //         m_elevationGridConfig.roi_y_min + (row+0.5f)*m_elevationGridConfig.grid_resolution,
                    //         cell.min_z, cell.max_z, cell.top_height, cell.height_range,
                    //         cell.ground_reference_z, cell.occupied_layers, cell.point_num);
                }
            }
        }
    }

    // ================================================================
    // Phase 3.5: Tall Neighbor Filter（高大物体邻域过滤）
    // ================================================================
    //
    // 设计思想：
    //   人的腿部 cell 和路沿石 cell 在单 cell 视角下无法区分（两者都有
    //   地面级 min_z + 15~35cm 的 top_height）。但人的腿部 cell 紧挨着
    //   躯干 cell（top_height > obstacle_top_height_max_far），而路沿石
    //   不会紧挨高大物体。
    //
    //   检查每个 is_obstacle_candidate==true 的 cell：
    //     统计 8 邻域中满足 is_ground && top_height >= obstacle_top_height_max_far
    //     的"tall 邻居"数量。
    //     若 >= min_tall_neighbors_for_filter（推荐 2），说明该 cell 紧邻高大物体，
    //     不是独立的低矮障碍物 → 取消 is_obstacle_candidate。
    //
    // 【为什么用 >= 而非 > 比较 top_height？】
    //   宁可多过滤（人的腿被清除），也不要让人的腿被当作低矮障碍物输出。
    //
    // 【为什么 min_tall_neighbors 推荐 2？】
    //   - 人的腿 cell 被人的躯干 cell 包围，tall 邻居 ≥ 4 → 过滤 ✓
    //   - 路沿石挨着一个人：仅边界处 1~2 个 tall 邻居 → 不触发 ✓
    //   - 路沿石（无高大物体相邻）：0 个 tall 邻居 → 不触发 ✓

    if (m_elevationGridConfig.enable_tall_neighbor_filter)
    {
        const auto& offsets = GetNeighborOffsets();
        const float tall_threshold = m_elevationGridConfig.obstacle_top_height_max_far;
        int tall_filtered_count = 0;

        for (int row = 0; row < m_grid_rows; ++row)
        {
            for (int col = 0; col < m_grid_cols; ++col)
            {
                int idx = GridIndex(row, col);
                GridCell& cell = m_vGridCell[idx];

                if (!cell.is_obstacle_candidate) continue;

                // 统计 8 邻域中的 tall neighbor
                int tall_neighbor_count = 0;
                for (const auto& [dr, dc] : offsets)
                {
                    int nr = row + dr;
                    int nc = col + dc;
                    if (!InBounds(nr, nc)) continue;

                    const GridCell& neighbor = m_vGridCell[GridIndex(nr, nc)];
                    if (neighbor.valid &&
                        neighbor.is_ground &&
                        neighbor.top_height >= tall_threshold)
                    {
                        tall_neighbor_count++;
                    }
                }

                if (tall_neighbor_count >= m_elevationGridConfig.min_tall_neighbors_for_filter)
                {
                    cell.is_obstacle_candidate = false;
                    tall_filtered_count++;

                    float cx = m_effective_roi_x_min +
                               (static_cast<float>(col) + 0.5f) * m_elevationGridConfig.grid_resolution;
                    // LOG_RAW("[TallNeighborFilter] cell(%d,%d) x=%.2f y=%.2f "
                    //         "top_h=%.3f cancelled: %d tall neighbors >= %d\n",
                    //         row, col, cx,
                    //         m_elevationGridConfig.roi_y_min + (row+0.5f)*m_elevationGridConfig.grid_resolution,
                    //         cell.top_height, tall_neighbor_count,
                    //         m_elevationGridConfig.min_tall_neighbors_for_filter);
                }
            }
        }

        LOG_RAW("[TallNeighborFilter] enabled, threshold=tall neighbors>=%d, "
                "cancelled %d obstacle_candidates\n",
                m_elevationGridConfig.min_tall_neighbors_for_filter, tall_filtered_count);
    }

    // 统计日志
    int vertical_count  = 0;
    int obstacle_count  = 0;
    int depressed_count = 0;
    for (int i =0; i< m_vGridCell.size(); i++)
    // for (const auto& cell : m_vGridCell)
    {
        if (m_vGridCell[i].is_vertical_structure){
            vertical_count++;
        }
           
        // if (m_vGridCell[i].is_obstacle_candidate){
        //     obstacle_count++;

        //     float cell_x=0.0f;
        //     float cell_y=0.0f;
        //     GridIndexToWorld(i, cell_x, cell_y);
        //     LOG_RAW("候选: %d, cell(%.2f, %.2f),point=%d, layer= %d, ref_z=%.2f, max_z=%.2f, min_z=%.2f\n", 
        //         i,cell_x,cell_y,m_vGridCell[i].point_num, m_vGridCell[i].layer_histogram.size(), 
        //         m_vGridCell[i].ground_reference_z, m_vGridCell[i].max_z, m_vGridCell[i].min_z);
        // }

            
        if (m_vGridCell[i].valid && m_vGridCell[i].top_height < 0.0f){
            depressed_count++;
        }
            
    }

    LOG_RAW("[VoxelOccupancy v2] vertical: %d, obstacle: %d, depressed: %d "
    "(top_height range: near[%.2f,%.2f] far[%.2f,%.2f], boundary: %.1fm)\n",
    vertical_count, obstacle_count, depressed_count,
    m_elevationGridConfig.obstacle_top_height_min_near, m_elevationGridConfig.obstacle_top_height_max_near,
    m_elevationGridConfig.obstacle_top_height_min_far,  m_elevationGridConfig.obstacle_top_height_max_far,
    m_elevationGridConfig.near_range_boundary);
    

    // ── Debug: 障碍物候选图 ──
    if (m_debugViewer)
    {
        m_debugViewer->DrawObstacleCandidate(m_vGridCell, m_grid_rows, m_grid_cols, m_elevationGridConfig);
    }

    // ================================================================
    // Phase 4: 基于 is_obstacle_candidate + ground_reference_z 重新分类点云
    // ================================================================
    //
    // 设计思想：
    //
    // 【为什么在这里重新分类而不是依赖 SplitPointCloud？】
    //   SplitPointCloud 仅根据 cell.is_ground 做二元划分：
    //     is_ground → ground_cloud
    //     !is_ground → obstacle_cloud
    //   这无法区分 Mixed Ground Cell (is_ground && is_obstacle_candidate) 中
    //   哪些点是地面、哪些点是障碍物。
    //
    // 【新分类规则】
    //   对于每个点，查找其所属 cell：
    //     1. cell.is_obstacle_candidate == true:
    //        - point.z > ground_reference_z + margin → ObstaclePoint (障碍物高于地面)
    //        - point.z <= ground_reference_z + margin → GroundPoint (地面点)
    //     2. cell.is_ground == true && cell.is_obstacle_candidate == false:
    //        - 纯地面 cell → 全部归入 GroundPoint
    //     3. 其他 (既不是地面也不是障碍物候选):
    //        - 高大物体/噪声 → 归入 obstacle_cloud (保持兼容)
    //
    // 【GroundCloud 含义】
    //   包含 Ground Surface 点：纯地面 cell 的全部点 + Mixed cell 中贴近地面的点
    //
    // 【ObstacleCloud 含义】
    //   仅包含真正满足 Obstacle Candidate 条件的点（高于参考地面的障碍物点）
    //
    // 【为什么用 ground_reference_z 而不是 cell.min_z？】
    //   ground_reference_z 是逐列从 Ground Grid 估计的"真实地面高度"，
    //   经过中位数、插值、平滑处理，比单个 cell 的 min_z 更鲁棒。
    //   cell.min_z 可能受孤立噪点影响（如一个异常低点）。
    //
    // 【margin 选取依据】
    //   使用 layer_resolution（默认 5cm）作为容差。
    //   在 mixed cell 中，z 接近 ground_reference_z 的点视为地面点，
    //   z 显著高于 ground_reference_z 的点视为障碍物点。

    ReclassifyPointCloud(cloud, ground_cloud, obstacle_cloud);
}

// ============================================================================
// 九-C、ReclassifyPointCloud —— 基于 Occupancy Analysis 结果重新分类点云
// ============================================================================

/**
 * 设计思想：
 *
 * 【为什么需要这个函数？】
 *   SplitPointCloud 仅根据 cell.is_ground 做简单二元划分，无法处理 Mixed Ground Cell
 *   (is_ground && is_obstacle_candidate) 的情况。
 *   本函数利用 AnalyzeVerticalOccupancy 的结果（is_obstacle_candidate, ground_reference_z）
 *   对每个点做更精细的分类。
 *
 * 【v3 修正：统一用 ground_reference_z 判定（修复"人体上半身被误判为地面"问题）】
 *   v2 中"纯地面 cell"(is_ground && !is_obstacle_candidate) 的所有点全部归入 ground_cloud。
 *   这导致 RegionGrowing 误爬上去的人体躯干 cell（is_ground=true 但实际不是地面）中的
 *   高点（z≈-0.93）被错误归入地面。
 *
 *   v3 修正：对所有 is_ground cell，统一以 ground_reference_z + kGroundMargin 判定：
 *     - point.z <= ground_reference_z + margin → ground_cloud（地面表面点）
 *     - point.z >  ground_reference_z + margin → obstacle_cloud（高于地面的点）
 *   不再区分"纯地面 cell"和"混合 cell"。
 *
 * 【分类规则 (v3)】
 *   对于每个点，查找其所属 cell：
 *     1. cell.is_ground == true:
 *        - point.z >  ground_reference_z + kGroundMargin → ObstaclePoint
 *        - point.z <= ground_reference_z + kGroundMargin → GroundPoint
 *     2. cell.is_obstacle_candidate == true (且 !is_ground):
 *        - 非地面障碍物候选（罕见） → ObstaclePoint
 *     3. 其他 (既不是地面也不是障碍物候选):
 *        - 高大物体/噪声 → 归入 obstacle_cloud（保持兼容，不影响 tracker/cluster）
 *
 * 【为什么地面容差使用 layer_resolution？】
 *   使用 1 个 layer_resolution（默认 5cm）作为容差。
 *   在 mixed cell 中，z 接近 ground_reference_z 的点视为地面表面点，
 *   z 显著高于 ground_reference_z 的点视为障碍物点。
 *
 * 时间复杂度: O(N), N = cloud.size()
 */
void ElevationMapGroundFilter::ReclassifyPointCloud(
    const pcl::PointCloud<pcl::PointXYZI>& cloud,
    pcl::PointCloud<pcl::PointXYZI>&       ground_cloud,
    pcl::PointCloud<pcl::PointXYZI>&       obstacle_cloud)
{
    ground_cloud.clear();
    obstacle_cloud.clear();
    ground_cloud.reserve(cloud.size());
    obstacle_cloud.reserve(cloud.size());

    // 地面容差: 1 个 layer_resolution
    const float kGroundMargin = m_elevationGridConfig.layer_resolution;

    int ground_point_count   = 0;
    int obstacle_point_count = 0;
    int above_ground_in_ground_cell = 0;  // is_ground cell 中高于参考地面的点数（v3 新增统计）

    // ── Phase 3-B': 障碍物点 → 1cm Raw Point Image ──────────────────────────
    // 复用本函数已有的点云遍历（不新增一次 20w+ 点的遍历）：
    // “凡是被判为 obstacle 的点（与 obstacle_cloud 完全同源）”就在 1cm 图像上写一个像素。
    // 注意：地面点【不】写像素，否则 ROI 会被地面点填满，轮廓退化为 Grid 形状。
    auto push_obstacle = [&](const pcl::PointXYZI& p) {
        obstacle_cloud.push_back(p);
        obstacle_point_count++;
        RasterizeRawPoint(p.x, p.y);
    };

    for (const auto& point : cloud.points)
    {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
            continue;

        int row, col;
        if (!WorldToGrid(point.x, point.y, row, col))
        {
            // 超出 ROI 的点默认归入障碍物
            push_obstacle(point);
            continue;
        }

        const GridCell& cell = m_vGridCell[GridIndex(row, col)];

        if (!cell.valid)
        {
            push_obstacle(point);
            continue;
        }

        // ── v3 统一逻辑：对所有 is_ground cell，以 ground_reference_z 判定 ──
        // 设计原因：RegionGrowing 可能误将人体/高物体的 cell 标记为 is_ground，
        //   此时不能盲信 is_ground，必须用 ground_reference_z 做二次校验。
        //   对 is_ground cell 中高于参考地面的点 → obstacle_cloud（而非 ground_cloud）。
        if (cell.is_ground)
        {
            // Ground Surface cell（纯地面 或 Mixed Ground Cell）
            if (point.z > cell.ground_reference_z + kGroundMargin)
            {
                push_obstacle(point);
                above_ground_in_ground_cell++;
            }
            else
            {
                ground_cloud.push_back(point);
                ground_point_count++;
            }
        }
        else if (cell.is_obstacle_candidate)
        {
            // 非地面障碍物候选（罕见：被障碍物完全覆盖、无地面点的 cell）
            push_obstacle(point);
        }
        else
        {
            // ── 既不是地面也不是障碍物候选（高大物体/孤立噪点） ──
            push_obstacle(point);
        }
    }

    LOG_RAW("[Reclassify v3] Ground: %d, Obstacle: %d (above_ground_in_ground_cell: %d)\n",
            ground_point_count, obstacle_point_count, above_ground_in_ground_cell);
}

// ============================================================================
// 十、ClusterObstacleGrid —— 8 邻域 BFS 障碍物聚类
// ============================================================================

/**
 * 设计思想：
 *
 * 【为什么使用 8 邻域 BFS 而不是 KD-Tree / EuclideanCluster？】
 *   1. Grid 是规则网格，邻域查询 O(1)，无需 KD-Tree 的 O(logN) 搜索
 *   2. BFS 天然适合 "连通分量分析" —— 找到所有相邻的 obstacle_candidate cell
 *   3. 8 邻域（包括对角线）比 4 邻域更鲁棒，能连接对角线相邻的 cell
 *
 * 【为什么聚类在 Grid 级别而不是 Point 级别？】
 *   1. 效率: Grid 级别 BFS 每个 cell 只入队一次，O(Grid)
 *      如果 Point 级别 BFS，需要 O(N*logN) 的邻域搜索
 *   2. 语义: GridCell 已经过 Voxel Occupancy Analysis 判定，
 *      只有 is_obstacle_candidate==true 的 cell 才会被聚类
 *   3. 噪声鲁棒: 孤立的单点噪声在 Grid 级别自然被过滤
 *
 * 【聚类流程】
 *
 *   遍历所有 GridCell:
 *     if cell.is_obstacle_candidate && cell.obstacle_label == -1:
 *       ┌─ BFS 初始化 ─────────────────────────┐
 *       │  queue.push(cell_idx)                 │
 *       │  cell.obstacle_label = cluster_id      │
 *       │  cluster_cells.push_back(cell_idx)    │
 *       └──────────────────────────────────────┘
 *              │
 *              ▼
 *       ┌─ BFS 扩展 ───────────────────────────┐
 *       │  while queue not empty:              │
 *       │    pop current                       │
 *       │    for each 8-neighbor:              │
 *       │      if valid && is_obstacle_candidate│
 *       │         && obstacle_label == -1:     │
 *       │        mark, push, collect           │
 *       └──────────────────────────────────────┘
 *              │
 *              ▼
 *       ┌─ 构建簇 ─────────────────────────────┐
 *       │  if cluster_cells.size() >= min:     │
 *       │    BuildClusterFromCells()           │
 *       │    clusters.push_back(cluster)       │
 *       └──────────────────────────────────────┘
 *              │
 *       cluster_id++
 *
 * 【时间复杂度】
 *   - 每个 obstacle_candidate cell 恰好被访问一次: O(N_obstacle_cells)
 *   - 每个 cell 最多检查 8 个邻居: O(8 * N_obstacle_cells) = O(Grid)
 *   - BuildClusterFromCells 对每个簇执行一次，总复杂度 O(N_obstacle_cells)
 *
 * 【为什么需要 min_cluster_cells？】
 *   过滤孤立的噪点 cell（如果某个 cell 被标记为 obstacle_candidate
 *   但周围没有其他候选 cell，可能是传感器噪声）
 */
std::vector<GridCluster> ElevationMapGroundFilter::ClusterObstacleGrid()
{
    std::vector<GridCluster> clusters;

    // 先重置所有 cell 的 obstacle_label
    for (auto& cell : m_vGridCell)
    {
        cell.obstacle_label = -1;
    }

    const auto& offsets = GetNeighborOffsets();
    int cluster_id = 0;

    // 遍历所有 GridCell
    for (int row = 0; row < m_grid_rows; ++row)
    {
        for (int col = 0; col < m_grid_cols; ++col)
        {
            int start_idx = GridIndex(row, col);
            GridCell& start_cell = m_vGridCell[start_idx];

            // 必须是 obstacle_candidate 且未被访问
            if (!start_cell.is_obstacle_candidate || start_cell.obstacle_label != -1)
            {
                continue;
            }

            // ── BFS 收集连通分量 ──
            std::vector<int> cluster_cells;
            std::queue<int>  bfs_queue;

            start_cell.obstacle_label = cluster_id;
            bfs_queue.push(start_idx);
            cluster_cells.push_back(start_idx);

            while (!bfs_queue.empty())
            {
                int cur_idx = bfs_queue.front();
                bfs_queue.pop();

                int r = cur_idx / m_grid_cols;
                int c = cur_idx % m_grid_cols;

                for (const auto& [dr, dc] : offsets)
                {
                    int nr = r + dr;
                    int nc = c + dc;

                    if (!InBounds(nr, nc))
                    {
                        continue;
                    }

                    int neighbor_idx = GridIndex(nr, nc);
                    GridCell& neighbor = m_vGridCell[neighbor_idx];

                    // 必须是 obstacle_candidate 且未被访问
                    if (neighbor.is_obstacle_candidate &&
                        neighbor.obstacle_label == -1)
                    {
                        neighbor.obstacle_label = cluster_id;
                        bfs_queue.push(neighbor_idx);
                        cluster_cells.push_back(neighbor_idx);
                    }
                }
            }


            // ── 过滤小簇（孤立噪点） ──
            if (static_cast<int>(cluster_cells.size()) >= m_elevationGridConfig.min_cluster_cells)
            {
                
                GridCluster cluster = BuildClusterFromCells(cluster_id, cluster_cells);

                // ── Phase 3-B': 1cm Raw Point Image → Cluster ROI → minAreaRect ──
                // 只写 cluster.img_* 实验字段；cluster.obb_*（PCA）完全不受影响。
                ComputeRawImageOBB(cluster);

                clusters.push_back(cluster);
                
                // if( cluster.center_x <  m_effective_roi_x_min + 0.3) // 车身附近聚类过滤
                // {
                //     LOG_RAW("[待解决]Cluster id: %d : Center(%.2f, %.2f, %.2f), 由 %d 个cell组成, 总point_num: %d\n", 
                //         cluster.id, cluster.center_x, cluster.center_y, cluster.center_z, cluster.cell_indices.size(), cluster.point_num);
        
                // }
                // else {
                //     clusters.push_back(cluster);
                // }
            }

            cluster_id++;
        }
    }

    for(size_t i=0; i < clusters.size(); i++){

        if(!clusters[i].has_obb){
            LOG_RAW("Cluster id: %d: Center(%.2f, %.2f, %.2f), height= %.2f, (len= %.2f, width= %.2f), 由 %d 个cell组成, 总point_num: %d\n", 
            clusters[i].id, clusters[i].center_x, clusters[i].center_y, clusters[i].center_z, clusters[i].height, 
            clusters[i].length, clusters[i].width,
            clusters[i].cell_indices.size(), clusters[i].point_num);
        }
        else{
            LOG_RAW("Cluster id: %d:[OBB] Center(%.2f, %.2f, %.2f), height= %.2f, (len= %.2f, width= %.2f), 由 %d 个cell组成, 总point_num: %d\n", 
            clusters[i].id, clusters[i].obb_center_x, clusters[i].obb_center_y, clusters[i].center_z, clusters[i].height,
            clusters[i].obb_length, clusters[i].obb_width,
            clusters[i].cell_indices.size(), clusters[i].point_num);
        }
        
    }

    // ========================================================================
    // Phase 3-B': A/B 日志（Grid PCA OBB vs 1cm Raw Point Image OBB）
    //   yaw 统一打印为度：PCA 归一到 [0,180)，IMG 本身为 [0,180)
    //   dYaw 为无向轴夹角（mod 180 后取最小值），禁止用 fabs 直接相减
    // ========================================================================
    for (size_t i = 0; i < clusters.size(); i++)
    {
        const GridCluster& cl = clusters[i];

        // PCA yaw（[-π/2, π/2)）→ 度并归一到 [0,180)
        float pca_yaw_deg = cl.obb_angle * 180.0f / static_cast<float>(M_PI);
        if (pca_yaw_deg < 0.0f) pca_yaw_deg += 180.0f;

        if (!cl.has_img_obb)
        {
            LOG_RAW("[RawImgOBB] frame=%d cluster=%d cells=%zu raw_pts=%d px=%d px_dil=%d roi=[c %d..%d][r %d..%d] "
                    "| PCA: c=(%.2f,%.2f) L=%.2f W=%.2f yaw=%.1f | IMG: has=0 (pixels<2 or ROI empty)\n",
                    m_rawImageFrame, cl.id, cl.cell_indices.size(), cl.img_raw_pt_count, cl.img_pixel_count,
                    cl.img_pixel_count_dilated,
                    cl.img_roi_min_col, cl.img_roi_max_col, cl.img_roi_min_row, cl.img_roi_max_row,
                    cl.obb_center_x, cl.obb_center_y, cl.obb_length, cl.obb_width, pca_yaw_deg);
            continue;
        }

        float img_yaw_deg = cl.img_yaw_rad * 180.0f / static_cast<float>(M_PI);
        if (img_yaw_deg < 0.0f) img_yaw_deg += 180.0f;
        if (img_yaw_deg >= 180.0f) img_yaw_deg -= 180.0f;

        // 无向轴夹角：Δ ∈ [-90, 90]，取绝对值后与 (180-|Δ|) 比较取小
        float d = img_yaw_deg - pca_yaw_deg;
        while (d <= -90.0f) d += 180.0f;
        while (d >    90.0f) d -= 180.0f;
        const float d_yaw = std::fabs(d);

        LOG_RAW("[RawImgOBB] frame=%d cluster=%d cells=%zu raw_pts=%d px=%d px_dil=%d roi=[c %d..%d][r %d..%d] "
                "| PCA: c=(%.2f,%.2f) L=%.2f W=%.2f yaw=%.1f | IMG: c=(%.2f,%.2f) L=%.2f W=%.2f yaw=%.1f has=1 "
                "| dYaw=%.1f dL=%.2f dW=%.2f\n",
                m_rawImageFrame, cl.id, cl.cell_indices.size(), cl.img_raw_pt_count, cl.img_pixel_count,
                cl.img_pixel_count_dilated,
                cl.img_roi_min_col, cl.img_roi_max_col, cl.img_roi_min_row, cl.img_roi_max_row,
                cl.obb_center_x, cl.obb_center_y, cl.obb_length, cl.obb_width, pca_yaw_deg,
                cl.img_center_x, cl.img_center_y, cl.img_length, cl.img_width, img_yaw_deg,
                d_yaw,
                cl.img_length - cl.obb_length,
                cl.img_width  - cl.obb_width);
    }

    


    return clusters;
}

// ============================================================================
// 十一、GridIndexToWorld —— Grid 线性索引 → 世界坐标
// ============================================================================

/**
 * 将 Grid 线性索引反算为对应 cell 中心的世界坐标
 *
 * 坐标系映射:
 *   idx → (row, col)
 *   row → y (左右方向): y_center = roi_y_min + (row + 0.5) * resolution
 *   col → x (前向方向): x_center = m_effective_roi_x_min + (col + 0.5) * resolution
 *
 * +0.5 偏移使坐标落在 cell 中心，而非左下角
 */
inline void ElevationMapGroundFilter::GridIndexToWorld(int idx, float& cx, float& cy) const
{
    int row = idx / m_grid_cols;
    int col = idx % m_grid_cols;
    float half_res = m_elevationGridConfig.grid_resolution * 0.5f;
    cx = m_effective_roi_x_min + (static_cast<float>(col) + 0.5f) * m_elevationGridConfig.grid_resolution;
    cy = m_elevationGridConfig.roi_y_min + (static_cast<float>(row) + 0.5f) * m_elevationGridConfig.grid_resolution;
}

// ============================================================================
// 十二、BuildClusterFromCells —— 从 cell 索引列表构建 GridCluster
// ============================================================================

/**
 * 设计思想：
 *
 * 【为什么需要单独的 BuildClusterFromCells？】
 *   将 BFS 收集到的 cell_indices 集合转换为结构化的 GridCluster:
 *     1. 计算 AABB (轴对齐包围盒)
 *     2. 计算中心位置
 *     3. 计算尺寸 (length × width × height)
 *     4. 汇总点数
 *
 * 【为什么用 AABB 而不是 OBB（有向包围盒）？】
 *   1. AABB 计算简单: O(N_cells)，只需 min/max 操作
 *   2. OBB 需要 PCA 或旋转卡壳，计算量大
 *   3. 低矮障碍物（路沿石、减速带）通常沿道路方向排列，
 *      在车身坐标系下近似轴对齐
 *   4. 后续 Kalman Tracker 通常用 AABB 初始化状态
 *
 * 【length/width/height 的含义】
 *   - length (X方向): 沿车辆前进方向
 *   - width  (Y方向): 沿车辆横向
 *   - height (Z方向): 垂直方向
 *   这与 S2obstacleBox 的 depth/width/height 一致
 *
 * 【为什么保留 center_* 和尺寸？】
 *   1. Kalman Tracker 初始化状态: [center_x, center_y, vx, vy, length, width]
 *   2. 关联匹配 (Gating): 用中心位置 + 尺寸计算马氏距离
 *   3. 直接转换为 S2obstacleBox: center → pos, length → depth, width → width
 */
GridCluster ElevationMapGroundFilter::BuildClusterFromCells(
    int cluster_id,
    const std::vector<int>& cell_indices) const
{
    GridCluster cluster;
    cluster.id = cluster_id;
    cluster.cell_indices = cell_indices;

    // 初始化 AABB 极值
    cluster.min_x =  FLT_MAX;
    cluster.max_x = -FLT_MAX;
    cluster.min_y =  FLT_MAX;
    cluster.max_y = -FLT_MAX;
    cluster.min_z =  FLT_MAX;
    cluster.max_z = -FLT_MAX;
    cluster.point_num = 0;

    for (int idx : cell_indices)
    {
        const GridCell& cell = m_vGridCell[idx];

        // 累加点数
        cluster.point_num += cell.point_num;

        // 获取 cell 中心的世界坐标
        float cx, cy;
        GridIndexToWorld(idx, cx, cy);

        // 扩展 AABB（考虑 cell 的半分辨率边界）
        float half_res = m_elevationGridConfig.grid_resolution * 0.5f;
        cluster.min_x = std::min(cluster.min_x, cx - half_res);
        cluster.max_x = std::max(cluster.max_x, cx + half_res);
        cluster.min_y = std::min(cluster.min_y, cy - half_res);
        cluster.max_y = std::max(cluster.max_y, cy + half_res);

        // Z 边界来自 cell 的 min_z / max_z
        cluster.min_z = std::min(cluster.min_z, cell.min_z);
        cluster.max_z = std::max(cluster.max_z, cell.max_z);
    }

    // 计算中心
    cluster.center_x = (cluster.min_x + cluster.max_x) * 0.5f;
    cluster.center_y = (cluster.min_y + cluster.max_y) * 0.5f;
    cluster.center_z = (cluster.min_z + cluster.max_z) * 0.5f;

    // 计算尺寸
    cluster.length = cluster.max_x - cluster.min_x;
    cluster.width  = cluster.max_y - cluster.min_y;
    cluster.height = cluster.max_z - cluster.min_z;

    // ── 计算 OBB (有向包围盒) ──
    // 设计原因: 转弯时 AABB 会侵入相邻车道，OBB 沿障碍物主方向对齐更紧凑
    ComputeClusterOBB(cluster);

    return cluster;
}

// ============================================================================
// 十三、ConvertClustersToS2ObstacleBox —— GridCluster → newS2AviodObject
// ============================================================================

/**
 * 设计思想：
 *
 * 【为什么需要这个转换函数？】
 *   项目已有下游接口使用 newS2AviodObject 和 S2obstacleBox 结构体
 *   （如 UDP 通信、规划模块等）。将 GridCluster 直接转换为 S2obstacleBox
 *   可以无缝对接现有接口，无需修改下游代码。
 *
 * 【字段映射】
 *   GridCluster           →  S2obstacleBox
 *   ────────────────────────────────────────
 *   cluster.id            →  id
 *   cluster.obb_length    →  depth   (OBB 长轴, 优先; 回退 AABB length)
 *   cluster.obb_width     →  width   (OBB 短轴, 优先; 回退 AABB width)
 *   cluster.height        →  height  (Z轴高度)
 *   cluster.obb_center_x  →  pos_x   (OBB 中心, 优先; 回退 AABB center)
 *   cluster.obb_center_y  →  pos_y
 *   cluster.center_z      →  pos_z
 *   OBB/AABB corners      →  corners[4] (OBB 优先, AABB 回退)
 *
 * 【为什么优先使用 OBB？】
 *   当车辆转弯时，AABB 角点可能侵入相邻车道。
 *   OBB 沿障碍物主方向对齐，给出更紧凑的包围盒，避免误侵入。
 *   如果 OBB 计算失败 (has_obb==false，如 cell 数量<3)，自动回退 AABB。
 *
 * 【corners 顺序】
 *   逆时针: 左下 → 右下 → 右上 → 左上
 *   与 TrackedObstacle.corners 和 S2obstacleBox.corners 一致
 *
 * 【为什么最多输出 10 个？】
 *   newS2AviodObject::obstacle[10] 数组固定为 10（现为 20），需要截断
 */
void ElevationMapGroundFilter::ConvertClustersToS2ObstacleBox(
    const std::vector<GridCluster>& clusters,
    const unsigned long long rec_timestamp,
    newS2AviodObject&                output)
{
    // obstacle 数组大小为 20（见 Type.h），取 min( clusters.size(), 20 )
    constexpr int kMaxObstacles = 20;
    output.obs_num = static_cast<int>(std::min(clusters.size(), static_cast<size_t>(kMaxObstacles)));
    output.timestamp = rec_timestamp;

    if(!clusters.empty()){
        output.return_val = 1;
    }
    else
        output.return_val = 0;

    for (int i = 0; i < output.obs_num; ++i)
    {
        const GridCluster& cluster = clusters[static_cast<size_t>(i)];
        S2obstacleBox& box = output.obstacle[i];

        // ── 基本属性: 优先 OBB，回退 AABB ──
        box.id = cluster.id;

        if (cluster.has_obb)
        {
            // OBB 尺寸 (长轴→depth, 短轴→width)
            box.depth  = cluster.obb_length;
            box.width  = cluster.obb_width;
            box.height = cluster.height;

            // OBB 中心
            box.pos_x = cluster.obb_center_x;
            box.pos_y = cluster.obb_center_y;
            box.pos_z = cluster.center_z;

            // OBB 角点 (逆时针)
            box.corners[0] = cluster.obb_corners[0];
            box.corners[1] = cluster.obb_corners[1];
            box.corners[2] = cluster.obb_corners[2];
            box.corners[3] = cluster.obb_corners[3];
        }
        else
        {
            // OBB 不可用，回退 AABB
            box.depth  = cluster.length;
            box.width  = cluster.width;
            box.height = cluster.height;

            box.pos_x = cluster.center_x;
            box.pos_y = cluster.center_y;
            box.pos_z = cluster.center_z;

            // AABB 角点
            box.corners[0] = { cluster.min_x, cluster.min_y };
            box.corners[1] = { cluster.max_x, cluster.min_y };
            box.corners[2] = { cluster.max_x, cluster.max_y };
            box.corners[3] = { cluster.min_x, cluster.max_y };
        }
    }
}



void ElevationMapGroundFilter::ConvertTrackToS2ObstacleBox(
    const std::vector<TrackedObstacle> & trackedObstacles,
    const unsigned long long rec_timestamp,
    newS2AviodObject&                output)
{
    // obstacle 数组大小为 20（见 Type.h），取 min( clusters.size(), 20 )
    constexpr int kMaxObstacles = 20;
    output.obs_num = static_cast<int>(std::min(trackedObstacles.size(), static_cast<size_t>(kMaxObstacles)));
    output.timestamp = rec_timestamp;

    if(!trackedObstacles.empty()){
        output.return_val = 1;
    }
    else
        output.return_val = 0;

    for (int i = 0; i < output.obs_num; ++i)
    {
        const TrackedObstacle& Obstacle= trackedObstacles[static_cast<size_t>(i)];
        S2obstacleBox& box = output.obstacle[i];

        box.id = Obstacle.id;

        box.depth  = Obstacle.depth;
        box.width  = Obstacle.width;
        box.height = Obstacle.height;

        box.pos_x = Obstacle.pos_x;
        box.pos_y = Obstacle.pos_y;
        box.pos_z = Obstacle.pos_z;

        box.corners[0] = Obstacle.corners[0];
        box.corners[1] = Obstacle.corners[1];
        box.corners[2] = Obstacle.corners[2];
        box.corners[3] = Obstacle.corners[3];
        
    }
}


// ============================================================================
// 十三-B、Phase 3-B': 1cm Raw Point Image（分配 / 栅格化 / Cluster ROI → minAreaRect）
// ============================================================================
//
// 设计约束（严格对应 Phase 3-B' Prompt）：
//   1. 图像内容 = ReclassifyPointCloud() 判为 obstacle 的点（与 obstacle_cloud 同源）。
//      若改用 BuildGrid 阶段的全部原始点，ROI 会被地面点填满，轮廓退化为 Grid 形状。
//   2. 点云遍历不新增：栅格化挂在 ReclassifyPointCloud() 已有的点云循环里。
//   3. Cluster 不再重新找点云：只扫自己的 1cm Image ROI（O(ROI 面积)）。
//   4. 不使用 findContours / morphology / 连通域选择 / 历史信息。
//   5. 只写 GridCluster.img_* 实验字段；PCA OBB / tracker / detections / UDP 全不受影响。
//
// 坐标系（与 Grid 是两套独立网格，只能通过世界坐标换算）：
//   col = floor((x - roi_x_min) / 0.01)      row = floor((y - roi_y_min) / 0.01)
//   像素 (col,row) 的世界代表点 = (roi_x_min + (col+0.5)*0.01, roi_y_min + (row+0.5)*0.01)

void ElevationMapGroundFilter::ResetRawPointImage()
{
    const float inv = 1.0f / kRawImageResolution;

    // 尺寸由配置 ROI 现算（不硬编码：不同车辆配置的 roi_* 不同）
    m_rawImageW = static_cast<int>(std::ceil(
        (m_elevationGridConfig.roi_x_max - m_elevationGridConfig.roi_x_min) * inv));
    m_rawImageH = static_cast<int>(std::ceil(
        (m_elevationGridConfig.roi_y_max - m_elevationGridConfig.roi_y_min) * inv));

    if (m_rawImageW <= 0 || m_rawImageH <= 0)
    {
        m_rawImageW = 0;
        m_rawImageH = 0;
        m_rawPointImage.clear();
        m_rawPointCounter.clear();
        return;
    }

    // assign：尺寸不变时复用已分配的容量（不会每帧重新 malloc）
    const size_t total = static_cast<size_t>(m_rawImageW) * static_cast<size_t>(m_rawImageH);
    m_rawPointImage.assign(total, 0);
    m_rawPointCounter.assign(total, 0);
    ++m_rawImageFrame;

    LOG_RAW("[RawImg] frame=%d reset 1cm RawPointImage: %d x %d (%.3fm/pixel), origin=(%.2f, %.2f), buf=%zu B x2\n",
            m_rawImageFrame, m_rawImageW, m_rawImageH, kRawImageResolution,
            m_elevationGridConfig.roi_x_min, m_elevationGridConfig.roi_y_min, total);
}

void ElevationMapGroundFilter::RasterizeRawPoint(float x, float y)
{
    if (m_rawImageW <= 0 || m_rawImageH <= 0) return;
    if (!std::isfinite(x) || !std::isfinite(y)) return;

    const float inv = 1.0f / kRawImageResolution;
    const int col = static_cast<int>(std::floor((x - m_elevationGridConfig.roi_x_min) * inv));
    const int row = static_cast<int>(std::floor((y - m_elevationGridConfig.roi_y_min) * inv));

    // ROI 外的点（如 x >= roi_x_max）没有像素：它们仍会进入 obstacle_cloud，
    // 但对 1cm 图像不可见 —— 只有 ROI 边缘的 cluster 会受影响。
    if (col < 0 || col >= m_rawImageW || row < 0 || row >= m_rawImageH) return;

    const size_t k = static_cast<size_t>(row) * m_rawImageW + col;
    m_rawPointImage[k] = 255;
    if (m_rawPointCounter[k] < 255)
    {
        ++m_rawPointCounter[k];
    }
}

void ElevationMapGroundFilter::ComputeRawImageOBB(GridCluster& cluster) const
{
    // 实验字段默认无效（无论成败都不影响 cluster.obb_* / has_obb）
    cluster.has_img_obb      = false;
    cluster.img_pixel_count  = 0;
    cluster.img_pixel_count_dilated = 0;
    cluster.img_raw_pt_count = 0;

    if (m_rawImageW <= 0 || m_rawImageH <= 0) return;

    const float res     = kRawImageResolution;
    const float inv_res = 1.0f / res;

    // ── Step 1: Cluster 的 Grid cell 世界范围（BuildClusterFromCells 已含 ±半格）→ 1cm ROI ──
    // padding = 0：只取 Grid cell 覆盖范围，不做任何额外空间扩展。
    // 端点用 floor 取像素下标，数学上等价于“边界像素包含”，额外包含量 ≤ 1cm。
    int c0 = static_cast<int>(std::floor((cluster.min_x - m_elevationGridConfig.roi_x_min) * inv_res));
    int c1 = static_cast<int>(std::floor((cluster.max_x - m_elevationGridConfig.roi_x_min) * inv_res));
    int r0 = static_cast<int>(std::floor((cluster.min_y - m_elevationGridConfig.roi_y_min) * inv_res));
    int r1 = static_cast<int>(std::floor((cluster.max_y - m_elevationGridConfig.roi_y_min) * inv_res));

    if (c1 < 0 || r1 < 0) return;                                   // ROI 在图像之外
    c0 = std::max(0, c0); r0 = std::max(0, r0);                     // 夹到图像边界
    c1 = std::min(m_rawImageW - 1, c1);
    r1 = std::min(m_rawImageH - 1, r1);
    if (c0 > c1 || r0 > r1) return;

    cluster.img_roi_min_col = c0;
    cluster.img_roi_max_col = c1;
    cluster.img_roi_min_row = r0;
    cluster.img_roi_max_row = r1;

    // ── Step 2: ROI 内收集全部非零像素（不做 findContours / morphology 二值图运算）──
    std::vector<cv::Point> pts;                       // 真实占用像素（图像像素坐标）
    pts.reserve(static_cast<size_t>(c1 - c0 + 1) * static_cast<size_t>(r1 - r0 + 1));

    int raw_pt_count = 0;
    for (int r = r0; r <= r1; ++r)
    {
        const uint8_t* img_row = m_rawPointImage.data() + static_cast<size_t>(r) * m_rawImageW;
        const uint8_t* cnt_row = (m_rawPointCounter.size() == m_rawPointImage.size())
                               ? (m_rawPointCounter.data() + static_cast<size_t>(r) * m_rawImageW)
                               : nullptr;
        for (int c = c0; c <= c1; ++c)
        {
            if (img_row[c])
            {
                pts.emplace_back(c, r);          // 像素坐标 (col, row)
                if (cnt_row) raw_pt_count += cnt_row[c];
            }
        }
    }
    cluster.img_pixel_count  = static_cast<int>(pts.size());
    cluster.img_raw_pt_count = raw_pt_count;

    // ── Step 3: 唯一失败判据（在膨胀之前判断：单点被膨胀后成为方块，yaw 无意义）──
    if (pts.size() < 2) return;

    // ── Step 3.5: Phase 3-B' 实验：ROI 内像素膨胀 ──────────────────────────────
    // 低矮目标单帧原始点常常只有几个（3~8 个），minAreaRect 对“少一两个极值点”极敏感
    // → yaw / L / W 逐帧抖动。先对占用像素做 Chebyshev 半径 R 的膨胀，提高拟合点集密度。
    //   - kRawImageDilateRadius = 0   → 关闭（等价于原始点集）
    //   - kRawImageDilateYawOnly=true → 只用膨胀集定方向，L/W/中心用原始像素重投影（不放大尺寸）
    //   - kRawImageDilateYawOnly=false→ 全部量都用膨胀集（L/W 各向同性放大 2*R*1cm）
    const int R = kRawImageDilateRadius;
    std::vector<cv::Point> pts_fit;
    if (R > 0)
    {
        // 在 "ROI + R 环" 的局部缓冲内膨胀，避免为每个 cluster 分配整幅图像
        const int lw = (c1 - c0 + 1) + 2 * R;   // 局部宽（列）
        const int lh = (r1 - r0 + 1) + 2 * R;   // 局部高（行）
        std::vector<uint8_t> buf(static_cast<size_t>(lw) * static_cast<size_t>(lh), 0);

        for (const auto& p : pts)
        {
            const int lc = (p.x - c0) + R;
            const int lr = (p.y - r0) + R;
            for (int dr = -R; dr <= R; ++dr)
            {
                for (int dc = -R; dc <= R; ++dc)
                {
                    buf[static_cast<size_t>(lr + dr) * lw + (lc + dc)] = 1;
                }
            }
        }

        pts_fit.reserve(static_cast<size_t>(lw) * static_cast<size_t>(lh));
        for (int lr = 0; lr < lh; ++lr)
        {
            for (int lc = 0; lc < lw; ++lc)
            {
                if (buf[static_cast<size_t>(lr) * lw + lc])
                {
                    // 局部 → 图像像素坐标：最多超出原 ROI 边界 R=1cm，世界坐标仍然良定义
                    pts_fit.emplace_back(lc - R + c0, lr - R + r0);
                }
            }
        }
    }
    else
    {
        pts_fit = pts;
    }
    cluster.img_pixel_count_dilated = static_cast<int>(pts_fit.size());

    // ── Step 4: minAreaRect（OpenCV 3.3：不读 RotatedRect::angle，只用 4 顶点几何）──
    const cv::RotatedRect rr = cv::minAreaRect(pts_fit);
    cv::Point2f v[4];
    rr.points(v);

    // 像素 → 世界（像素中心约定，与 RasterizeRawPoint 的 floor 成对）
    Eigen::Vector2f P[4];
    for (int i = 0; i < 4; ++i)
    {
        P[i] = Eigen::Vector2f(m_elevationGridConfig.roi_x_min + (v[i].x + 0.5f) * res,
                               m_elevationGridConfig.roi_y_min + (v[i].y + 0.5f) * res);
    }

    // (1) 两条相邻边中较长者为长轴 u
    const Eigen::Vector2f e0 = P[1] - P[0];
    const Eigen::Vector2f e1 = P[2] - P[1];
    const float n0 = e0.norm();
    const float n1 = e1.norm();
    Eigen::Vector2f u = (n0 >= n1) ? e0 : e1;
    float L  = std::max(n0, n1);   // 长轴长度
    float Wd = std::min(n0, n1);   // 短轴长度
    if (L <= 1e-6f) return;              // 退化保护（正常不会到这里）
    u /= L;

    // (2) yaw ∈ [0, π)：把 u 统一到上半平面
    if (u.y() < 0.0f || (u.y() == 0.0f && u.x() < 0.0f))
    {
        u = -u;
    }
    const float yaw = std::atan2(u.y(), u.x());

    // (3) 右手正交副轴 v = rot90(u)
    const Eigen::Vector2f v_axis(-u.y(), u.x());

    // (4) 中心 = 4 顶点均值（膨胀集的中心）
    Eigen::Vector2f center = (P[0] + P[1] + P[2] + P[3]) * 0.25f;

    // (4.5) 可选：只用膨胀集定方向，尺寸/中心用【原始像素】在该方向上重投影
    //       （u 的正负号不影响 span；中心偏移与 u 同号翻转，故结果与归一化顺序无关）
    if (R > 0 && kRawImageDilateYawOnly)
    {
        float mn_u = FLT_MAX, mx_u = -FLT_MAX, mn_v = FLT_MAX, mx_v = -FLT_MAX;
        for (const auto& p : pts)
        {
            const Eigen::Vector2f pw(m_elevationGridConfig.roi_x_min + (p.x + 0.5f) * res,
                                     m_elevationGridConfig.roi_y_min + (p.y + 0.5f) * res);
            const Eigen::Vector2f d = pw - center;
            const float pu = d.dot(u);
            const float pv = d.dot(v_axis);
            mn_u = std::min(mn_u, pu); mx_u = std::max(mx_u, pu);
            mn_v = std::min(mn_v, pv); mx_v = std::max(mx_v, pv);
        }
        if (mx_u > mn_u)
        {
            center = center + u * (0.5f * (mn_u + mx_u)) + v_axis * (0.5f * (mn_v + mx_v));
            L  = mx_u - mn_u;
            Wd = mx_v - mn_v;    // 原始点共线时可为 0（已知退化，日志可见）
        }
    }

    // (5) corners：左下→右下→右上→左上（与 ComputeClusterOBB 完全同序，C0→C1 = 长轴）
    const float hl = L  * 0.5f;
    const float hw = Wd * 0.5f;
    const Eigen::Vector2f C0 = center - hl * u - hw * v_axis;
    const Eigen::Vector2f C1 = center + hl * u - hw * v_axis;
    const Eigen::Vector2f C2 = center + hl * u + hw * v_axis;
    const Eigen::Vector2f C3 = center - hl * u + hw * v_axis;

    cluster.img_corners[0] = { C0.x(), C0.y() };
    cluster.img_corners[1] = { C1.x(), C1.y() };
    cluster.img_corners[2] = { C2.x(), C2.y() };
    cluster.img_corners[3] = { C3.x(), C3.y() };
    cluster.img_center_x = center.x();
    cluster.img_center_y = center.y();
    cluster.img_length   = L;
    cluster.img_width    = Wd;
    cluster.img_yaw_rad  = yaw;          // [0, π)
    cluster.has_img_obb  = true;
}

// ============================================================================
// 十四、ComputeClusterOBB —— 2D PCA 计算有向包围盒
// ============================================================================

/**
 * 设计思想：
 *
 * 【为什么用 2D PCA 而不是 3D PCA？】
 *   1. 低矮障碍物（路沿石、减速带）近似水平放置，Z 方向不需要旋转
 *   2. 2D PCA 只需要 2×2 特征分解，比 3D 快得多
 *   3. Z 方向保持轴对齐不影响实际效果（障碍物高度本身就是垂直的）
 *
 * 【PCA 计算步骤】
 *   1. 收集所有 cell 的 (x, y) 中心坐标
 *   2. 计算均值 μ = (1/n)·Σ p_i
 *   3. 计算协方差矩阵 C = (1/n)·Σ (p_i - μ)(p_i - μ)^T
 *   4. SelfAdjointEigenSolver 特征分解: C·v = λ·v
 *   5. 最大特征值对应的特征向量 → 主轴方向 (obb_length)
 *      最小特征值对应的特征向量 → 次轴方向 (obb_width)
 *   6. 将所有点投影到主轴/次轴，求 min/max 得到 OBB 尺寸
 *   7. 主轴与 X 轴夹角 → obb_angle
 *
 * 【为什么需要 ≥ 3 个 cell？】
 *   2 个 cell 的协方差矩阵是奇异的（只有 1 个有效维度），
 *   PCA 退化为过两点的直线，OBB width = 0，没有实际意义。
 *   cell 数量 < 3 时设置 has_obb = false，调用方回退 AABB。
 *
 * 【时间复杂度】
 *   O(N_cells)，每个 cell 做常数次浮点运算。
 *   特征分解 2×2 矩阵是 O(1)。
 *
 * 【旋转角 obb_angle 的定义】
 *   主轴 (obb_length 方向) 与 X 轴正向的夹角，范围 [-π/2, π/2]。
 *   正值表示主轴偏向 Y+ 方向。
 */
void ElevationMapGroundFilter::ComputeClusterOBB(GridCluster& cluster) const
{
    const size_t n = cluster.cell_indices.size();

    // 至少需要 3 个 cell 才能可靠计算 PCA
    if (n < 3)
    {
        cluster.has_obb = false;
        return;
    }

    // ── Step 1: 收集 2D 中心坐标 ──
    std::vector<Eigen::Vector2f> points;
    points.reserve(n);
    for (int idx : cluster.cell_indices)
    {
        // if(m_vGridCell[idx].point_num >= 10 ){

            float cx, cy;
            GridIndexToWorld(idx, cx, cy);
            points.emplace_back(cx, cy);
        // }
        // else{
        //     int row = idx / m_grid_cols;
        //     int col = idx % m_grid_cols;
        //     LOG_RAW(" 剔除idx=[%d, %d] ",col,row);
        // }
    }
    // LOG_RAW("\n");

    // ── Step 2: 计算均值 ──
    Eigen::Vector2f mean = Eigen::Vector2f::Zero();
    for (const auto& p : points)
    {
        mean += p;
    }
    mean /= static_cast<float>(n);

    // ── Step 3: 计算 2×2 协方差矩阵 ──
    // C = (1/n) * Σ (p_i - μ)(p_i - μ)^T
    Eigen::Matrix2f cov = Eigen::Matrix2f::Zero();
    for (const auto& p : points)
    {
        Eigen::Vector2f d = p - mean;
        cov += d * d.transpose();
    }
    cov /= static_cast<float>(n);

    // ── Step 4: 特征分解 ──
    // SelfAdjointEigenSolver 保证特征值按升序排列: λ0 ≤ λ1
    // eigenvectors().col(0) → 次轴方向 (对应 λ0)
    // eigenvectors().col(1) → 主轴方向 (对应 λ1)
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> solver(cov);
    if (solver.info() != Eigen::Success)
    {
        cluster.has_obb = false;
        return;
    }

    Eigen::Vector2f eigen_vals  = solver.eigenvalues();
    Eigen::Matrix2f eigen_vecs  = solver.eigenvectors();

    // 主轴 (λ 更大) → col(1), 次轴 (λ 更小) → col(0)
    Eigen::Vector2f axis_primary   = eigen_vecs.col(1);  // obb_length 方向
    Eigen::Vector2f axis_secondary = eigen_vecs.col(0);  // obb_width 方向


    float lambda_min = eigen_vals(0);
    float lambda_max = eigen_vals(1);

    float orientation_confidence =
        (lambda_max - lambda_min) /
        std::max(lambda_max + lambda_min, 1e-6f);

    // Phase 3-B: 保存 PCA 特征值/置信度（供 STATIC OBB 方向可观测性判断使用）。
    // 仅新增“保存”，不改变本函数任何 OBB 计算与输出。
    cluster.obb_lambda_max = lambda_max;
    cluster.obb_lambda_min = lambda_min;
    cluster.obb_orientation_confidence = orientation_confidence;

    float raw_angle =
        std::atan2(axis_primary.y(), axis_primary.x());

    // printf(
    //     "[OBB-PCA] "
    //     "cluster=%d "
    //     "cells=%zu "
    //     "mean=(%.3f, %.3f) "
    //     "lambda=(%.6f, %.6f) "
    //     "axis_primary=(%.4f, %.4f) "
    //     "raw_angle=%.2fdeg "
    //     "confidence=%.3f\n",
    //     cluster.id,
    //     n,
    //     mean.x(),
    //     mean.y(),
    //     lambda_min,
    //     lambda_max,
    //     axis_primary.x(),
    //     axis_primary.y(),
    //     raw_angle * 180.0f / static_cast<float>(M_PI),
    //     orientation_confidence
    // );

    // printf("[OBB-CELLS] cluster=%d:", cluster.id);

    for (int idx : cluster.cell_indices)
    {
        int row = idx / m_grid_cols;
        int col = idx % m_grid_cols;

        float cx, cy;
        GridIndexToWorld(idx, cx, cy);

        // printf(" id=[%d,%d]->(%.2f,%.2f), point_num=%d, min_z=%.2f, max_z=%.2f, height_diff=%.2f \n", col, row, cx, cy,
        // m_vGridCell[idx].point_num, m_vGridCell[idx].min_z, m_vGridCell[idx].max_z, m_vGridCell[idx].height_range
        // );
    }

    // printf("\n");


    // ── Step 5: 投影求 OBB 尺寸 ──
    float min_pri =  FLT_MAX, max_pri = -FLT_MAX;
    float min_sec =  FLT_MAX, max_sec = -FLT_MAX;

    for (const auto& p : points)
    {
        Eigen::Vector2f d = p - mean;
        float proj_pri = d.dot(axis_primary);
        float proj_sec = d.dot(axis_secondary);

        min_pri = std::min(min_pri, proj_pri);
        max_pri = std::max(max_pri, proj_pri);
        min_sec = std::min(min_sec, proj_sec);
        max_sec = std::max(max_sec, proj_sec);
    }

    // ── Step 6: 填充 OBB 数据 ──
    cluster.obb_center_x = mean.x();
    cluster.obb_center_y = mean.y();

    // 长轴 = obb_length, 短轴 = obb_width
    // 确保 length >= width（语义一致性）
    float len_pri = max_pri - min_pri;   // 主轴方向跨度
    float len_sec = max_sec - min_sec;   // 次轴方向跨度

    if (len_pri >= len_sec)
    {
        cluster.obb_length = len_pri;
        cluster.obb_width  = len_sec;
        cluster.obb_angle  = std::atan2(axis_primary.y(), axis_primary.x());
    }
    else
    {
        // 理论上不会发生（主轴 λ 更大），但是做保护
        cluster.obb_length = len_sec;
        cluster.obb_width  = len_pri;
        cluster.obb_angle  = std::atan2(axis_secondary.y(), axis_secondary.x());
    }

    // ── Step 7: 计算 4 个角点（逆时针: 左下→右下→右上→左上）──
    // OBB 在主轴/次轴坐标系下的 4 个角:
    //   (min_pri, min_sec), (max_pri, min_sec),
    //   (max_pri, max_sec), (min_pri, max_sec)
    // 变换回世界坐标: corner = mean + axis_pri * pri + axis_sec * sec

    auto worldCorner = [&](float pri, float sec) -> Point2D {
        Eigen::Vector2f w = mean + axis_primary * pri + axis_secondary * sec;
        return { w.x(), w.y() };
    };

    cluster.obb_corners[0] = worldCorner(min_pri, min_sec);  // 左下
    cluster.obb_corners[1] = worldCorner(max_pri, min_sec);  // 右下
    cluster.obb_corners[2] = worldCorner(max_pri, max_sec);  // 右上
    cluster.obb_corners[3] = worldCorner(min_pri, max_sec);  // 左上

    cluster.has_obb = true;
}

// ============================================================================
// 十五、ConvertClusterToTrackedObstacle —— GridCluster → TrackedObstacle
// ============================================================================

/**
 * 设计思想：
 *
 * 【为什么需要这个转换？】
 *   SimpleTracker::update() 的输入是 std::vector<TrackedObstacle>，
 *   需要将 GridCluster 映射为 TrackedObstacle 格式。
 *   OBB 优先，AABB 回退。
 *
 * 【Translation 和 Rotation 的填充】
 *   TrackedObstacle 有 Translation[3] 和 Rotation[9] 字段（来自深度相机跟踪），
 *   这里用 OBB 信息填充:
 *     Translation = (obb_center_x, obb_center_y, center_z)
 *     Rotation    = 2D 旋转矩阵 (绕 Z 轴旋转 obb_angle)
 *
 * 【为什么不在 GridCluster 中直接存 TrackedObstacle？】
 *   GridCluster 是算法内部的数据结构，TrackedObstacle 是 track.h 的结构。
 *   保持两者解耦，GridCluster 不依赖 track.h。
 */
void ElevationMapGroundFilter::ConvertClusterToTrackedObstacle(
    const GridCluster& cluster,
    TrackedObstacle&   obs)
{
    // ── 位置: OBB 优先 ──
    if (cluster.has_obb)
    {
        obs.pos_x = cluster.obb_center_x;
        obs.pos_y = cluster.obb_center_y;
        obs.depth = cluster.obb_length;
        obs.width = cluster.obb_width;

        // OBB 角点
        obs.corners[0] = cluster.obb_corners[0];
        obs.corners[1] = cluster.obb_corners[1];
        obs.corners[2] = cluster.obb_corners[2];
        obs.corners[3] = cluster.obb_corners[3];

        // Rotation: 2D 旋转矩阵 (绕 Z 轴)
        float c = std::cos(cluster.obb_angle);
        float s = std::sin(cluster.obb_angle);
        obs.Rotation[0] =  c;  obs.Rotation[1] = -s;  obs.Rotation[2] = 0.0f;
        obs.Rotation[3] =  s;  obs.Rotation[4] =  c;  obs.Rotation[5] = 0.0f;
        obs.Rotation[6] = 0.0f; obs.Rotation[7] = 0.0f; obs.Rotation[8] = 1.0f;
    }
    else
    {
        // 回退 AABB
        obs.pos_x = cluster.center_x;
        obs.pos_y = cluster.center_y;
        obs.depth = cluster.length;
        obs.width = cluster.width;

        obs.corners[0] = { cluster.min_x, cluster.min_y };
        obs.corners[1] = { cluster.max_x, cluster.min_y };
        obs.corners[2] = { cluster.max_x, cluster.max_y };
        obs.corners[3] = { cluster.min_x, cluster.max_y };

        // 无旋转（单位矩阵）
        for (int i = 0; i < 9; i++) obs.Rotation[i] = 0.0f;
        obs.Rotation[0] = 1.0f;
        obs.Rotation[4] = 1.0f;
        obs.Rotation[8] = 1.0f;
    }

    obs.pos_z  = cluster.center_z;
    obs.height = cluster.height;

    // Translation
    obs.Translation[0] = obs.pos_x;
    obs.Translation[1] = obs.pos_y;
    obs.Translation[2] = obs.pos_z;

    // 其他字段由 tracker 填充
    obs.id       = -1;
    obs.age      = 0;
    obs.lastSeen = 0;
}

// ============================================================================
// 十六、SyncTrackedResultsToClusters —— 跟踪结果回写 GridCluster
// ============================================================================

/**
 * 设计思想：
 *
 * 【为什么需要回写？】
 *   SimpleTracker 维护的是 TrackedObstacle 列表（含持久 ID、age、lastSeen）。
 *   下游接口（ConvertClustersToS2ObstacleBox）需要 GridCluster 格式。
 *   因此需要将跟踪结果的关键信息（ID、位置、尺寸、角点）回写到 GridCluster。
 *
 * 【为什么只保留 age >= 3 的轨迹？】
 *   与 track.cpp 中的 finalObstacles 过滤逻辑一致:
 *     1. 新检测到的目标需要连续 3 帧确认（防止误检）
 *     2. age < 3 的轨迹可能是暂态的噪点
 *
 * 【注意】
 *   回写后的 GridCluster 的 OBB/尺寸信息来自 tracker 中的最新观测，
 *   可能与原始聚类结果略有差异（tracker 在匹配距离 > 0.2m 时才更新位置）。
 *   这种设计是有意为之——tracker 对微小抖动保持位置不变，输出更稳定。
 */
void ElevationMapGroundFilter::SyncTrackedResultsToClusters(
    const std::vector<TrackedObstacle>& vtrackings,
    std::vector<GridCluster>&           out) const
{
    out.clear();
    out.reserve(vtrackings.size());

    for (const auto& track : vtrackings)
    {
        // 只输出稳定轨迹 (age >= 3)
        if (track.age < 3)
        {
            continue;
        }

        GridCluster cluster;
        cluster.id        = track.id;
        cluster.point_num = 0;  // tracker 不维护 point_num

        // 位置和尺寸来自 tracker
        cluster.center_x = track.pos_x;
        cluster.center_y = track.pos_y;
        cluster.center_z = track.pos_z;

        cluster.length = track.depth;
        cluster.width  = track.width;
        cluster.height = track.height;

        // AABB 边界（从 corners 反推）
        cluster.min_x = track.corners[0].x;
        cluster.max_x = track.corners[1].x;
        cluster.min_y = track.corners[0].y;
        cluster.max_y = track.corners[2].y;
        cluster.min_z = track.pos_z - track.height * 0.5f;
        cluster.max_z = track.pos_z + track.height * 0.5f;

        // OBB 角点来自 tracker 的 corners（已在 ConvertClusterToTrackedObstacle 中填充）
        cluster.has_obb = true;
        cluster.obb_center_x = track.pos_x;
        cluster.obb_center_y = track.pos_y;
        cluster.obb_length   = track.depth;
        cluster.obb_width    = track.width;
        // obb_angle 从 Rotation 矩阵反算
        cluster.obb_angle    = std::atan2(track.Rotation[3], track.Rotation[0]);
        cluster.obb_corners[0] = track.corners[0];
        cluster.obb_corners[1] = track.corners[1];
        cluster.obb_corners[2] = track.corners[2];
        cluster.obb_corners[3] = track.corners[3];

        out.push_back(cluster);
    }
}

// ============================================================================
// 十七、ProcessWithObstacleTracking —— Ground Filter + OBB + SimpleTracker
// ============================================================================

/**
 * 设计思想：
 *
 * 【完整流程】
 *
 *   Raw PointCloud
 *        │
 *   ┌────▼──────────────────────────┐
 *   │ 1~6: Ground Filter (不变)      │  BuildGrid → ... → SplitPointCloud
 *   └────┬──────────────────────────┘
 *        │
 *   ┌────▼──────────────────────────┐
 *   │ 7: AnalyzeVerticalOccupancy   │  体素占据分析 → is_obstacle_candidate
 *   └────┬──────────────────────────┘
 *        │
 *   ┌────▼──────────────────────────┐
 *   │ 8: ClusterObstacleGrid        │  8邻域 BFS → GridCluster 列表
 *   │    + ComputeClusterOBB        │  PCA → OBB (有向包围盒)
 *   └────┬──────────────────────────┘
 *        │
 *   ┌────▼──────────────────────────┐
 *   │ 9: ConvertClusterTo           │  GridCluster → TrackedObstacle
 *   │    TrackedObstacle            │
 *   └────┬──────────────────────────┘
 *        │
 *   ┌────▼──────────────────────────┐
 *   │ 10: SimpleTracker::update()   │  贪心匹配 + ID 分配 + 丢帧管理
 *   └────┬──────────────────────────┘
 *        │
 *   ┌────▼──────────────────────────┐
 *   │ 11: SyncTrackedResults        │  TrackedObstacle → GridCluster (age≥3)
 *   │     ToClusters                │
 *   └────┬──────────────────────────┘
 *        │
 *   tracked_clusters_ (可通过 GetTrackedClusters() 获取)
 *
 * 【为什么 SimpleTracker 替代 cluster_id++？】
 *   cluster_id++ 每帧从 0 重新编号，下游模块无法关联前后帧的同一障碍物。
 *   SimpleTracker 通过欧氏距离贪心匹配，提供跨帧一致的持久 ID。
 *
 * 【tracker_ 的生命周期】
 *   tracker_ 在首次调用时懒初始化（Lazy Initialization），
 *   确保 ElevationMapGroundFilter 对象跨帧存在时，tracker 内部状态持续累积。
 */
bool ElevationMapGroundFilter::ProcessWithObstacleTracking(
    const PointCloud2Intensity::Ptr& cloud,
    PointCloud2Intensity::Ptr&       ground_cloud,
    PointCloud2Intensity::Ptr&       obstacle_cloud)
{
    if (!cloud || cloud->empty())
    {
        return false;
    }

    // ── Step 1~6: Ground Filter ──
    if (!BuildGrid(*cloud))
    {
        return false;
    }

    ComputeGroundHeight();
    ComputeSlope();
    RegionGrowing();
    GenerateGroundMask();

    // ── Step 6.5: Ground Reference ──
    ComputeGroundReference();

    // SplitPointCloud(*cloud, *ground_cloud, *obstacle_cloud);

    // ── Step 7: Voxel Occupancy Analysis + Point Cloud Reclassification ──
    // AnalyzeVerticalOccupancy 内部完成:
    //   layer_histogram 构建 + is_obstacle_candidate 判定
    //   + 基于判定结果的点云重分类（替代 SplitPointCloud）
    AnalyzeVerticalOccupancy(*cloud, *ground_cloud, *obstacle_cloud);

    // ── Step 8: Obstacle Clustering + OBB ──
    // BuildClusterFromCells 内部已调用 ComputeClusterOBB
    std::vector<GridCluster> raw_clusters = ClusterObstacleGrid();

    // // ── Debug: 聚类 Label 图 ──
    // if (m_debugViewer)
    // {
    //     m_debugViewer->DrawCluster(m_vGridCell, m_grid_rows, m_grid_cols, m_elevationGridConfig);
    // }

    // // ── Debug: 包围盒俯视图 ──
    // if (m_debugViewer)
    // {
    //     m_debugViewer->DrawBoundingBox(raw_clusters, m_elevationGridConfig);
    // }

    // // ── Debug: 最终点云叠加图 ──
    // if (m_debugViewer)
    // {
    //     // m_debugViewer->DrawOverlay(*ground_cloud, *obstacle_cloud, raw_clusters, m_elevationGridConfig);
    // }

    // ── Step 9: GridCluster → TrackedObstacle ──
    std::vector<TrackedObstacle> detections;
    detections.reserve(raw_clusters.size());
    for (const auto& cluster : raw_clusters)
    {
        TrackedObstacle obs;
        ConvertClusterToTrackedObstacle(cluster, obs);
        detections.push_back(obs);
    }

    // // ── Step 10: SimpleTracker 跟踪 ──
    // // Lazy 初始化 tracker_
    // if (!tracker_)
    // {
    //     tracker_ = std::make_unique<SimpleTracker>();
    // }
    // tracker_->update(detections);

    // // ── Step 11: 跟踪结果回写 GridCluster ──
    // SyncTrackedResultsToClusters(tracker_->vtrackings, tracked_clusters_);

    // LOG_RAW("[ProcessWithObstacleTracking] raw clusters: %zu, tracked: %zu\n",
    //         raw_clusters.size(), tracked_clusters_.size());

    return true;
}

// ============================================================================
// 十八、SetTrackerMatchThreshold —— 设置跟踪匹配阈值
// ============================================================================

void ElevationMapGroundFilter::SetTrackerMatchThreshold(float threshold)
{
    if (!tracker_)
    {
        tracker_ = std::make_unique<SimpleTracker>();
    }
    tracker_->match_threshold = threshold;
}

}  // namespace Lidar_Low_Detection
