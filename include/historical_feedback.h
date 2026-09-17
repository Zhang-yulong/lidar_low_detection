#pragma once
#ifndef HISTORICAL_FEEDBACK_H
#define HISTORICAL_FEEDBACK_H

/**
 * @file historical_feedback.h
 * @brief Phase 2/3 Quick Validation —— Historical Feedback 实验性数据结构与辅助函数
 *
 * 目的（Quick Validation，非正式 Phase 3）：
 *   验证 "Historical Track → Map Anchor → 当前 Radar Frame A → A 所在 Cell +
 *   8 邻居(3×3) → 当前 Cluster/Track 关联" 这一闭环是否可靠，
 *   为当前帧因 LiDAR 视角变化导致的漏检提供稳定的空间先验。
 *
 * 本轮明确不做：
 *   - 不修改 BuildGrid
 *   - 不实现 Raw Point Image / 5cm Image / CSR / Raw Point minAreaRect
 *   - 不实现 Historical Image / 5-frame history / Historical Contour / Morphological Fusion
 *   - 不重写 Tracker、不修改最终 UDP 输出、不做 motion_state 完整分类
 *
 * 本轮只做（Debug / Validation only）：
 *   - 复用已有 GridCluster / TrackedObstacle / Localization / CoordinateTransformer / SimpleTracker
 *   - 将历史 Track 锚定到地图系（外部侧表 MapAnchoredTrack，不修改 TrackedObstacle）
 *   - 把 Map Anchor 投影回当前雷达系得到预测位置 A
 *   - A → 当前 Grid Base Cell → 3×3 Neighbor Search → 候选当前 Cluster
 *   - 关联：优先 SAME_TRACK_ID（当前 tracker 同 ID 且本帧匹配），否则几何距离（带门控）
 *   - 记录关联结果 + DebugViewer / 日志可视化
 *   - 上一阶段的 OBB rasterize / overlap / fused_cells 统计保留但降级为 secondary
 *
 * 坐标系约定（已从代码确认）：
 *   - Grid / GridCluster / TrackedObstacle.pos_* 均为【主雷达系】：
 *       x = 前向(forward), y = 左向(left)
 *     Grid 索引: row = floor((y - roi_y_min)/res), col = floor((x - effective_roi_x_min)/res)
 *     Grid cell 中心: cx = effective_roi_x_min + (col+0.5)*res, cy = roi_y_min + (row+0.5)*res
 *   - 地图系为 ENU（x=东, y=北, 米）
 *   - CoordinateTransformer::vehicleToMap / mapToVehicle 为互逆纯计算，
 *     内部已统一应用 gridHeadingOffsetDeg 补偿
 */

#include "ElevationMapGroundFilter.h"   // ElevationGridConfig / GridCluster / Point2D
#include <vector>
#include <algorithm>
#include <cmath>

namespace Lidar_Low_Detection
{

/// 历史 Track 需要达到的稳定年龄（连续匹配帧数），低于此值不产生 Historical Feedback
constexpr int kMinTrackAgeForFeedback = 3;

// ============================================================================
// Phase 2/3（本轮）：Map Anchor → 当前 Radar A → 3×3 Grid 关联
// ============================================================================

/// 3×3 关联原因（HistoricalAssociation reason）
enum HistoricalMatchReason
{
    kHistMatchNone        = -1,  ///< 未关联（当前无对应检测 / 该目标处于 miss / ROI 外）
    kHistMatchSameTrackId = 0,   ///< 当前 tracker 存在同 ID Track 且本帧匹配成功（最可靠确认）
    kHistMatchGeometric   = 1    ///< 当前 tracker 无同 ID 目标 → 3×3 窗口内最近候选（几何距离 + 门控）
};

/// 几何 fallback 门控：A → 候选 Cluster 中心的最大允许距离（米）。
/// 只有在候选 Cluster 确实占据 3×3 窗口内 cell、且距离不超过该门控时才允许几何关联，
/// 避免"无限制选取最近 Cluster"造成误关联。
constexpr float kHistAssociationMaxDistM = 0.5f;

/**
 * @brief 地图系锚定的历史 Track 状态（Quick Validation 侧表，不修改 TrackedObstacle）
 *
 * 设计思想：
 *   - TrackedObstacle 当前【没有】 map_x/map_y / yaw / motion_state（Phase 2 未实现）。
 *   - 本次验证通过这个外部侧表，在每帧 Track 匹配成功时用当前位姿把其雷达系位置/角点
 *     锚定到地图系；匹配失败（miss）时冻结锚点，从而正确解耦自车运动。
 *   - 这是对正式 Phase 2 "Map-frame Stable Track" 的最小外部等价实现。
 */
struct MapAnchoredTrack
{
    int     id         = -1;     ///< Track ID（与 SimpleTracker 分配的 ID 一致）
    double  map_x      = 0.0;    ///< 地图系锚点 X（东, m）
    double  map_y      = 0.0;    ///< 地图系锚点 Y（北, m）
    bool    has_map    = false;  ///< 是否已成功建立地图锚点
    float   depth      = 0.0f;   ///< 长度（雷达系 x 方向, m）
    float   width      = 0.0f;   ///< 宽度（雷达系 y 方向, m）
    Point2D map_corners[4] = {}; ///< 4 角点在地图系（用于重建 OBB 区域）
    int     age        = 0;      ///< 最后一次更新锚点时的 track.age
    int     lastSeen   = 0;      ///< 最后一次更新锚点时的 track.lastSeen

    // ---- Phase 3-B: STATIC OBB 的“已验证 yaw”记忆（地图系，无向长轴，弧度）----
    // 语义与 map_x/map_y/map_corners 不同：这里只保存【被接受】的稳定 yaw
    //   - 观测到可靠 PCA 主方向时写入（可观测 + 与历史连续 / 首次可观测）
    //   - PCA 不可观测或与历史明显冲突时【不写入】（保留上一次可靠值）
    //   - UpdateMapAnchors() 不修改这两个字段（保证锚点始终基于 RAW 几何）
    bool    has_static_yaw     = false;
    float   static_yaw_map_rad = 0.0f;
};

/**
 * @brief Historical Feedback Region（投影到当前雷达系的空间先验）
 *
 * 由 MapAnchoredTrack 经 mapToVehicle 投影到当前帧得到，并把 OBB 区域 rasterize 到
 * 当前 Grid（10cm cell 中心采样）。
 */
struct HistoricalFeedbackRegion
{
    int    track_id           = -1;   ///< 来源 Track ID
    int    matched_cluster_id = -1;   ///< 与当前 Cluster 最大重叠的 cluster id（-1=无重叠）
    bool   has_map_anchor     = false;///< 是否存在有效地图锚点
    float  center_x = 0.0f, center_y = 0.0f; ///< 当前雷达系中心
    float  length   = 0.0f, width    = 0.0f; ///< length/width（雷达系）
    Point2D corners[4] = {};         ///< 当前雷达系 4 角点（OBB 区域）
    std::vector<int> projected_cell_indices; ///< 当前 Grid 被 OBB 覆盖的 cell（线性索引, secondary）
    bool   valid = false;            ///< A 投影是否存在（本帧该历史 Track 的反馈记录有效）

    // 与当前 Cluster 的重叠统计（Quick Validation 日志/可视化用）
    int current_cells     = 0;       ///< 匹配到的当前 Cluster cell 数
    int historical_cells  = 0;       ///< Historical 覆盖 cell 数
    int overlap_cells     = 0;       ///< 与当前 Cluster 重叠 cell 数
    int fused_cells       = 0;       ///< current ∪ historical cell 数

    // ========================================================================
    // Phase 2/3（本轮主验证指标）：Map Anchor → 当前 Radar A → 3×3 Grid 关联
    // 上述 OBB rasterize / overlap / fused 统计保留，但已降级为 secondary。
    // ========================================================================

    // A：历史 Track 的地图锚点投影到【当前主雷达系】后的预测位置（x=前向, y=左向, m）
    float  projected_x = 0.0f;
    float  projected_y = 0.0f;

    // A → 当前 Grid Base Cell。
    //   严格 ROI 内：a_inside_grid = true；
    //   距 ROI 边界 ≤1 cell（如刚超出盲区/前方 max_x）：夹取到最近边界 cell 仍做 3×3 搜索；
    //   完全在 ROI 外（>1 cell）：a_searchable=false，不搜索（视作无当前检测）。
    bool   a_inside_grid = false;
    bool   a_searchable  = false;
    int    base_row = -1;            ///< Base Cell 行（夹取后）
    int    base_col = -1;            ///< Base Cell 列（夹取后）
    int    base_linear_index = -1;   ///< Base Cell 线性索引 row*cols+col

    /// 3×3 Search Window 中落在 Grid 范围内的 cell 线性索引（≤9，边界裁剪）
    std::vector<int> search_window_indices;

    /// 当前 Cluster Candidate：3×3 窗口内出现的当前 Cluster ID（去重、升序）
    std::vector<int> candidate_cluster_ids;

    // 关联结果（核心）
    int    association_cluster_id = -1;      ///< 最终关联的当前 Cluster ID（-1=无）
    int    association_reason     = kHistMatchNone;  ///< HistoricalMatchReason
    bool   association_in_window  = false;   ///< 关联 Cluster 是否位于 3×3 窗口内（1001≠1002 也能关联）
    float  association_distance   = -1.0f;   ///< A → 关联 Cluster 中心 欧氏距离(m)
    float  matched_center_x = 0.0f;          ///< 关联 Cluster 中心（DebugViewer 画线 A→Center 用）
    float  matched_center_y = 0.0f;
    bool   no_current_detection = false;     ///< 当前帧该历史目标无对应检测（miss / 完全 ROI 外）

    // 当前 tracker 中同 ID Track 的实时状态（-1 = 当前 tracker 已无该 ID）
    int    live_track_age      = -1;         ///< live track.age
    int    live_track_lastSeen = -1;         ///< live track.lastSeen（miss 帧计数）
    bool   live_track_matched  = false;      ///< live track 本帧是否匹配成功(lastSeen==0)
};

// ============================================================================
// 辅助函数（header-only，避免新增源文件）
// ============================================================================

/**
 * @brief 判断点 (x,y) 是否位于凸四边形 q[4] 内（按顺序，逆/顺时针均可）
 *
 * 使用符号一致的叉积判定：若四边叉积同号（或为零）则点在四边形内/边界上。
 */
inline bool PointInQuad(const Point2D q[4], float x, float y)
{
    bool has_pos = false, has_neg = false;
    for (int i = 0; i < 4; ++i)
    {
        const Point2D& a = q[i];
        const Point2D& b = q[(i + 1) % 4];
        const float cross = (b.x - a.x) * (y - a.y) - (b.y - a.y) * (x - a.x);
        if (cross > 1e-6f)       has_pos = true;
        else if (cross < -1e-6f) has_neg = true;
        if (has_pos && has_neg)  return false;
    }
    return true;
}

/**
 * @brief 将凸四边形 rasterize 到 Grid（以 cell 中心采样判断覆盖）
 *
 * @param q[in]        4 角点（当前雷达系）
 * @param cfg[in]      Grid 配置
 * @param effective_roi_x_min[in]  BuildGrid 计算出的有效 ROI X 起点
 * @param rows[in]     Grid 行数
 * @param cols[in]     Grid 列数
 * @param out_cells[out] 被覆盖的 cell 线性索引（row * cols + col）
 */
inline void RasterizeQuadToGrid(const Point2D q[4],
                                const ElevationGridConfig& cfg,
                                float effective_roi_x_min,
                                int rows, int cols,
                                std::vector<int>& out_cells)
{
    out_cells.clear();
    if (rows <= 0 || cols <= 0) return;

    float minx = q[0].x, maxx = q[0].x, miny = q[0].y, maxy = q[0].y;
    for (int i = 1; i < 4; ++i)
    {
        minx = std::min(minx, q[i].x); maxx = std::max(maxx, q[i].x);
        miny = std::min(miny, q[i].y); maxy = std::max(maxy, q[i].y);
    }

    const float res = cfg.grid_resolution;
    if (res <= 0.0f) return;
    const float inv = 1.0f / res;

    int c0 = static_cast<int>(std::floor((minx - effective_roi_x_min) * inv));
    int c1 = static_cast<int>(std::floor((maxx - effective_roi_x_min) * inv));
    int r0 = static_cast<int>(std::floor((miny - cfg.roi_y_min) * inv));
    int r1 = static_cast<int>(std::floor((maxy - cfg.roi_y_min) * inv));

    c0 = std::max(0, c0); c1 = std::min(cols - 1, c1);
    r0 = std::max(0, r0); r1 = std::min(rows - 1, r1);
    if (c0 > c1 || r0 > r1) return;

    for (int r = r0; r <= r1; ++r)
    {
        for (int c = c0; c <= c1; ++c)
        {
            const float cx = effective_roi_x_min + (static_cast<float>(c) + 0.5f) * res;
            const float cy = cfg.roi_y_min + (static_cast<float>(r) + 0.5f) * res;
            if (PointInQuad(q, cx, cy))
            {
                out_cells.push_back(r * cols + c);
            }
        }
    }
}

} // namespace Lidar_Low_Detection

#endif // HISTORICAL_FEEDBACK_H
