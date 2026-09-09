#pragma once
#ifndef HISTORICAL_FEEDBACK_H
#define HISTORICAL_FEEDBACK_H

/**
 * @file historical_feedback.h
 * @brief Phase 2/3 Quick Validation —— Historical Feedback 实验性数据结构与辅助函数
 *
 * 目的（Quick Validation，非正式 Phase 3）：
 *   验证 "上一帧 Historical Track → Map Frame → 当前 Radar Frame → Current Grid"
 *   这一基本闭环是否能为当前帧因 LiDAR 视角变化导致的漏检提供稳定的空间先验。
 *
 * 本次明确不做：
 *   - 不修改 BuildGrid
 *   - 不实现 Raw Point Image / 5cm Image / CSR / Raw Point minAreaRect
 *   - 不实现 Historical Image / 5-frame history / Historical Contour / Morphological Fusion
 *   - 不重写 Tracker、不修改最终 UDP 输出
 *
 * 本次只做：
 *   - 复用已有 GridCluster / TrackedObstacle / Localization / CoordinateTransformer
 *   - 将历史 Track 锚定到地图系（外部侧表，不修改 TrackedObstacle）
 *   - 投影回当前雷达系并 rasterize 到当前 Grid → Historical Feedback Region
 *   - Current ∪ Historical → Fused Debug Region
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
    std::vector<int> projected_cell_indices; ///< 当前 Grid 被覆盖的 cell（线性索引）
    bool   valid = false;            ///< 是否投影成功且覆盖了至少 1 个 cell

    // 与当前 Cluster 的重叠统计（Quick Validation 日志/可视化用）
    int current_cells     = 0;       ///< 匹配到的当前 Cluster cell 数
    int historical_cells  = 0;       ///< Historical 覆盖 cell 数
    int overlap_cells     = 0;       ///< 与当前 Cluster 重叠 cell 数
    int fused_cells       = 0;       ///< current ∪ historical cell 数
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
