#pragma once
#ifndef HISTORICAL_GEOMETRY_H
#define HISTORICAL_GEOMETRY_H

/**
 * @file historical_geometry.h
 * @brief Phase 3-B —— STATIC Historical Geometry（Historical cell 补充 + 可选 weighted OBB）
 *
 * 只处理：STATIC + 当前 Cluster 存在 + Phase 2 已关联到 Historical Track。
 * 复用 Phase 2 已产出的 HistoricalFeedbackRegion：
 *   - projected_cell_indices（历史 OBB 投影到当前 Grid = historical predicted cells）
 *   - association_cluster_id / association_reason / association_distance
 *   - corners（历史 OBB 在当前雷达系，用于历史 yaw）
 * 不新增第二套 historical table / association / 3×3 search / 投影。
 *
 * 方向恒定：Historical → Map → Current Radar → Current Grid → 与 Current cells 比较。
 * Current Detection 始终是主数据，历史几何只做“补缺”，且必须通过全部安全规则。
 *
 * 明确不做（后续阶段）：STATIC+MISS/Coast、MOVING/UNKNOWN 融合、Raw Point/5cm/CSR、
 * 多帧 contour fusion、Track lifetime 重设计、修改 ComputeClusterOBB。
 *
 * 开关（§35/§51）：
 *   kEnableHistoricalCellSupplement = true  → 3-B-1（cell 补充 + 日志，不改几何）
 *   kEnableHistoricalFusedOBB       = false → 3-B-2（加权 OBB 写回 vtrackings）
 */

#include "historical_feedback.h"   // HistoricalFeedbackRegion / kHistMatch* / kHistAssociationMaxDistM
#include <vector>
#include <cmath>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>

namespace Lidar_Low_Detection
{

// ---- 开关 ----
constexpr bool kEnableHistoricalCellSupplement = true;
constexpr bool kEnableHistoricalFusedOBB       = false;
constexpr bool kHistoricalGeometryVerboseCells = false;  // 逐 cell reject 原因日志（调试用，默认关）

// ---- 参数 ----
constexpr float kHistoricalSupplementMaxDistM = 0.20f;  // 候选 cell 距当前 cluster 最近 cell 的上限
constexpr float kHistoricalMaxAreaRatio       = 1.5f;   // fused/current 面积比上限
constexpr float kHistCurrentWeight            = 1.0f;   // Current cell 权重
constexpr float kHistSupplementWeight         = 0.5f;   // Historical supplement cell 权重
constexpr float kHistYawMinConfidence         = 0.30f;  // 低于此置信度保留历史 yaw
constexpr int   kHistMinCellsForPCA           = 3;      // 少于 3 cell 不做 PCA
constexpr float kHistGeomPi                   = 3.14159265358979323846f;

/// 加权 OBB 结果
struct WeightedOBB
{
    bool  ok         = false;
    float cx         = 0.0f;
    float cy         = 0.0f;
    float length     = 0.0f;
    float width      = 0.0f;
    float yaw_deg    = 0.0f;   // 已 fold 到 [-90, 90)
    float confidence = 0.0f;   // (λ1-λ2)/(λ1+λ2)
};

/// 单次 STATIC Historical Geometry 的评估结果（Debug / 日志用）
struct HistoricalGeometryResult
{
    bool  valid = false;        ///< 是否走到了融合评估（有 association）
    bool  fused = false;        ///< 是否接受过 supplement cell
    const char* skip_reason = ""; ///< 未融合原因：NO_TRACK/NOT_STATIC/AMBIGUOUS/ASSOC_FAR 等

    int track_id = -1;
    int cluster_id = -1;
    int motion_state = 0;       ///< MotionState (int)
    int association_reason = -1;

    int current_cells = 0;
    int historical_cells = 0;   ///< projected cells（去重）
    int missing_cells = 0;      ///< historical - current
    int accepted_cells = 0;

    int rejected_out_of_grid = 0;
    int rejected_other_cluster = 0;
    int rejected_distance = 0;
    int rejected_connectivity = 0;
    int rejected_area = 0;

    int max_supplement = 0;
    float area_ratio = 1.0f;

    std::vector<int> supplement_cells;

    bool  obb_updated = false;
    float center_x = 0.0f;
    float center_y = 0.0f;
    float length = 0.0f;
    float width = 0.0f;
    float yaw_deg = 0.0f;
    float orientation_confidence = 0.0f;
};

// ============================================================================
// 辅助函数
// ============================================================================

/// Grid cell 线性索引 → 世界坐标中心（与 BuildGrid / GridIndexToWorld 同一约定）
inline void HistCellCenter(int idx, int cols, float eff_x_min, float roi_y_min, float res,
                           float& cx, float& cy)
{
    const int r = (cols > 0) ? (idx / cols) : 0;
    const int c = (cols > 0) ? (idx % cols) : 0;
    cx = eff_x_min + (static_cast<float>(c) + 0.5f) * res;
    cy = roi_y_min + (static_cast<float>(r) + 0.5f) * res;
}

/// yaw 折叠到 [-90°, 90°)（长轴无向：θ 与 θ±180° 等价）
inline float HistFoldYawDeg(float deg)
{
    while (deg >= 90.0f)  deg -= 180.0f;
    while (deg < -90.0f)  deg += 180.0f;
    return deg;
}

/// 加权 2D PCA OBB（等价于 ComputeClusterOBB 的 PCA，但支持权重；不改动原函数）
inline WeightedOBB ComputeWeightedOBB(const std::vector<Point2D>& pts,
                                      const std::vector<float>& w)
{
    WeightedOBB o;
    const int n = static_cast<int>(pts.size());
    if (n < kHistMinCellsForPCA || static_cast<int>(w.size()) != n) return o;

    float sw = 0.0f;
    for (float wi : w) sw += wi;
    if (sw <= 1e-6f) return o;

    float mx = 0.0f, my = 0.0f;
    for (int i = 0; i < n; ++i) { mx += w[i] * pts[i].x; my += w[i] * pts[i].y; }
    mx /= sw; my /= sw;

    float a = 0.0f, b = 0.0f, c = 0.0f;
    for (int i = 0; i < n; ++i)
    {
        const float dx = pts[i].x - mx;
        const float dy = pts[i].y - my;
        a += w[i] * dx * dx;
        b += w[i] * dx * dy;
        c += w[i] * dy * dy;
    }
    a /= sw; b /= sw; c /= sw;

    const float tr  = a + c;
    const float det = a * c - b * b;
    float disc = tr * tr * 0.25f - det;
    if (disc < 0.0f) disc = 0.0f;
    const float s  = std::sqrt(disc);
    const float l1 = tr * 0.5f + s;   // λmax
    const float l2 = tr * 0.5f - s;   // λmin

    // 主轴（λmax 对应特征向量）
    float ux, uy;
    if (std::fabs(b) > 1e-9f) { ux = b; uy = l1 - a; }
    else if (a >= c)          { ux = 1.0f; uy = 0.0f; }
    else                      { ux = 0.0f; uy = 1.0f; }
    float un = std::sqrt(ux * ux + uy * uy);
    if (un < 1e-9f) { ux = 1.0f; uy = 0.0f; } else { ux /= un; uy /= un; }
    const float vx = -uy, vy = ux;

    float min_u = 1e9f, max_u = -1e9f, min_v = 1e9f, max_v = -1e9f;
    for (int i = 0; i < n; ++i)
    {
        const float dx = pts[i].x - mx;
        const float dy = pts[i].y - my;
        const float pu = dx * ux + dy * uy;
        const float pv = dx * vx + dy * vy;
        min_u = std::min(min_u, pu); max_u = std::max(max_u, pu);
        min_v = std::min(min_v, pv); max_v = std::max(max_v, pv);
    }
    const float len_u = max_u - min_u;
    const float len_v = max_v - min_v;

    float ax = ux, ay = uy;
    if (len_u >= len_v) { o.length = len_u; o.width = len_v; }
    else                { o.length = len_v; o.width = len_u; ax = vx; ay = vy; }

    o.cx = mx; o.cy = my;
    o.yaw_deg = HistFoldYawDeg(std::atan2(ay, ax) * 180.0f / kHistGeomPi);
    o.confidence = (l1 + l2 > 1e-9f) ? (l1 - l2) / (l1 + l2) : 0.0f;
    o.ok = true;
    return o;
}

// ============================================================================
// 主入口：STATIC Historical Geometry Fusion
// ============================================================================

/**
 * @brief 对 Phase 2 已关联的 historical track 执行 STATIC cell 补充（可选 weighted OBB）
 *
 * 只处理：association_cluster_id >= 0（当前 Cluster 存在且已关联）。
 * MOVING / UNKNOWN / 无关联 一律跳过（Phase 3-C / 后续阶段）。
 * 写回：仅当 kEnableHistoricalFusedOBB 且存在 accepted supplement 时，更新
 *       m_tracker.vtrackings 中对应 Track 的 pos_x、pos_y、depth、width、corners；
 *       不改 map 字段、motion 字段、age、lastSeen、vx、vy。
 */
inline void FuseHistoricalGeometry(
    const std::vector<GridCluster>& clusters,
    const std::vector<HistoricalFeedbackRegion>& feedback,
    std::vector<TrackedObstacle>& tracks,
    const ElevationGridConfig& cfg,
    int rows, int cols, float eff_x_min,
    std::vector<HistoricalGeometryResult>& out)
{
    out.clear();
    if (!kEnableHistoricalCellSupplement) return;

    const float res = cfg.grid_resolution;
    if (res <= 0.0f || rows <= 0 || cols <= 0) return;

    // ---- 当前帧索引 ----
    std::unordered_map<int, int> cell_owner;                 // cell → cluster id (first-wins)
    std::unordered_map<int, const GridCluster*> cluster_by_id;
    for (const auto& cl : clusters)
    {
        cluster_by_id[cl.id] = &cl;
        for (int idx : cl.cell_indices)
        {
            if (cell_owner.find(idx) == cell_owner.end()) cell_owner[idx] = cl.id;
        }
    }

    std::unordered_map<int, MotionState>       state_by_track;
    std::unordered_map<int, TrackedObstacle*>  track_by_id;
    for (auto& t : tracks)
    {
        state_by_track[t.id] = t.motion_state;
        track_by_id[t.id]    = &t;
    }

    // ---- 排他性：一个 current cluster 只能被一个 historical track 补充 ----
    std::unordered_map<int, std::unordered_set<int>> cluster_tracks;
    for (const auto& r : feedback)
    {
        if (r.association_cluster_id < 0) continue;
        if (r.association_reason != kHistMatchSameTrackId &&
            r.association_reason != kHistMatchGeometric) continue;
        cluster_tracks[r.association_cluster_id].insert(r.track_id);
    }

    for (const auto& r : feedback)
    {
        const int cid = r.association_cluster_id;
        if (cid < 0) continue;   // 完全漏检 → Phase 3-C，本阶段不处理

        auto it_cl = cluster_by_id.find(cid);
        if (it_cl == cluster_by_id.end()) continue;
        const GridCluster& cur = *it_cl->second;

        HistoricalGeometryResult g;
        g.valid = true;
        g.track_id = r.track_id;
        g.cluster_id = cid;
        g.association_reason = r.association_reason;
        g.current_cells = static_cast<int>(cur.cell_indices.size());

        // ---- gate 1: Track 存在 + STATIC ----
        auto it_state = state_by_track.find(r.track_id);
        if (it_state == state_by_track.end())
        {
            g.skip_reason = "NO_TRACK";
            out.push_back(g); continue;
        }
        g.motion_state = static_cast<int>(it_state->second);
        if (it_state->second != MotionState::STATIC)
        {
            g.skip_reason = "NOT_STATIC";
            out.push_back(g); continue;
        }

        // ---- gate 2: 一个 cluster 只能被一个 historical track 补充 ----
        auto it_ct = cluster_tracks.find(cid);
        if (it_ct != cluster_tracks.end() && it_ct->second.size() > 1)
        {
            g.skip_reason = "AMBIGUOUS";
            out.push_back(g); continue;
        }

        // ---- gate 3: association 距离沿用 Phase 2 约束（0.5m）----
        if (r.association_distance >= 0.0f && r.association_distance > kHistAssociationMaxDistM)
        {
            g.skip_reason = "ASSOC_FAR";
            out.push_back(g); continue;
        }

        // ---- historical predicted cells（去重）----
        std::unordered_set<int> hist_set(r.projected_cell_indices.begin(),
                                         r.projected_cell_indices.end());
        g.historical_cells = static_cast<int>(hist_set.size());

        std::unordered_set<int> cur_set(cur.cell_indices.begin(), cur.cell_indices.end());

        // ---- missing candidates + 过滤 1/2/3 ----
        struct Cand { int idx; float dist; };
        std::vector<Cand> cands;
        const int total = rows * cols;
        for (int idx : hist_set)
        {
            if (cur_set.count(idx)) continue;                    // 已是当前 cell
            if (idx < 0 || idx >= total) { g.rejected_out_of_grid++; continue; }
            const int rr = idx / cols, cc = idx % cols;
            if (rr < 0 || rr >= rows || cc < 0 || cc >= cols)
            { g.rejected_out_of_grid++; continue; }

            auto it_o = cell_owner.find(idx);
            if (it_o != cell_owner.end() && it_o->second != cid)
            { g.rejected_other_cluster++; continue; }            // 属于其他 cluster → 禁止

            float cx = 0.0f, cy = 0.0f;
            HistCellCenter(idx, cols, eff_x_min, cfg.roi_y_min, res, cx, cy);
            float best = 1e9f;
            for (int cidx : cur.cell_indices)
            {
                float ox = 0.0f, oy = 0.0f;
                HistCellCenter(cidx, cols, eff_x_min, cfg.roi_y_min, res, ox, oy);
                const float dx = cx - ox, dy = cy - oy;
                const float d = std::sqrt(dx * dx + dy * dy);
                if (d < best) best = d;
            }
            if (best > kHistoricalSupplementMaxDistM)
            { g.rejected_distance++; continue; }                 // 太远 → 拒绝

            cands.push_back(Cand{ idx, best });
        }
        g.missing_cells = static_cast<int>(cands.size());
        std::sort(cands.begin(), cands.end(),
                  [](const Cand& a, const Cand& b) { return a.dist < b.dist; });

        // ---- 面积策略（含小目标例外）----
        int max_supp = static_cast<int>(std::floor(
            static_cast<float>(g.current_cells) * (kHistoricalMaxAreaRatio - 1.0f)));
        if (g.current_cells <= 2) max_supp = 1;   // 小目标：允许且仅允许 +1（否则完全补不了）
        if (max_supp < 0) max_supp = 0;
        g.max_supplement = max_supp;

        // ---- 过滤 4/5：连通性 + 面积 ----
        std::unordered_set<int> fused = cur_set;
        for (const auto& cand : cands)
        {
            if (g.accepted_cells >= max_supp)
            {
                g.rejected_area++;
                if (kHistoricalGeometryVerboseCells)
                    LOG_RAW("[HistoricalGeometry]   candidate=(%d,%d) reason=AREA_LIMIT\n",
                            cand.idx / cols, cand.idx % cols);
                continue;
            }
            const int rr = cand.idx / cols, cc = cand.idx % cols;
            bool adj = false;
            for (int dr = -1; dr <= 1 && !adj; ++dr)
            {
                for (int dc = -1; dc <= 1 && !adj; ++dc)
                {
                    if (dr == 0 && dc == 0) continue;
                    const int nr = rr + dr, nc = cc + dc;
                    if (nr < 0 || nr >= rows || nc < 0 || nc >= cols) continue;
                    if (fused.count(nr * cols + nc)) adj = true;
                }
            }
            if (!adj)
            {
                g.rejected_connectivity++;
                if (kHistoricalGeometryVerboseCells)
                    LOG_RAW("[HistoricalGeometry]   candidate=(%d,%d) reason=DISCONNECTED\n", rr, cc);
                continue;
            }
            fused.insert(cand.idx);
            g.supplement_cells.push_back(cand.idx);
            g.accepted_cells++;
        }

        g.fused = (g.accepted_cells > 0);
        g.area_ratio = (g.current_cells > 0)
                     ? static_cast<float>(g.current_cells + g.accepted_cells)
                       / static_cast<float>(g.current_cells)
                     : 1.0f;

        // ---- 3-B-2（可选）：加权 OBB 写回 ----
        if (kEnableHistoricalFusedOBB && g.fused)
        {
            std::vector<Point2D> pts;
            std::vector<float>   wt;
            pts.reserve(cur.cell_indices.size() + g.supplement_cells.size());
            wt.reserve(pts.capacity());
            for (int idx : cur.cell_indices)
            {
                float cx = 0.0f, cy = 0.0f;
                HistCellCenter(idx, cols, eff_x_min, cfg.roi_y_min, res, cx, cy);
                pts.push_back(Point2D{ cx, cy });
                wt.push_back(kHistCurrentWeight);
            }
            for (int idx : g.supplement_cells)
            {
                float cx = 0.0f, cy = 0.0f;
                HistCellCenter(idx, cols, eff_x_min, cfg.roi_y_min, res, cx, cy);
                pts.push_back(Point2D{ cx, cy });
                wt.push_back(kHistSupplementWeight);
            }

            WeightedOBB obb = ComputeWeightedOBB(pts, wt);
            if (obb.ok)
            {
                // yaw 策略：置信度低 → 保留历史 yaw（STATIC prior）
                const float hx = r.corners[1].x - r.corners[0].x;
                const float hy = r.corners[1].y - r.corners[0].y;
                const float hist_yaw = HistFoldYawDeg(std::atan2(hy, hx) * 180.0f / kHistGeomPi);
                if (obb.confidence < kHistYawMinConfidence) obb.yaw_deg = hist_yaw;

                auto it_tr = track_by_id.find(r.track_id);
                if (it_tr != track_by_id.end() && it_tr->second != nullptr)
                {
                    TrackedObstacle& t = *it_tr->second;
                    t.pos_x = obb.cx;
                    t.pos_y = obb.cy;
                    t.depth = obb.length;
                    t.width = obb.width;

                    const float c = std::cos(obb.yaw_deg * kHistGeomPi / 180.0f);
                    const float s = std::sin(obb.yaw_deg * kHistGeomPi / 180.0f);
                    const float hl = obb.length * 0.5f;
                    const float hw = obb.width  * 0.5f;
                    // 顺序保持 左下→右下→右上→左上（与 ComputeClusterOBB 一致）
                    t.corners[0] = Point2D{ obb.cx - c * hl + s * hw, obb.cy - s * hl - c * hw };
                    t.corners[1] = Point2D{ obb.cx + c * hl + s * hw, obb.cy + s * hl - c * hw };
                    t.corners[2] = Point2D{ obb.cx + c * hl - s * hw, obb.cy + s * hl + c * hw };
                    t.corners[3] = Point2D{ obb.cx - c * hl - s * hw, obb.cy - s * hl + c * hw };
                    g.obb_updated = true;
                }
                g.center_x = obb.cx;
                g.center_y = obb.cy;
                g.length   = obb.length;
                g.width    = obb.width;
                g.yaw_deg  = obb.yaw_deg;
                g.orientation_confidence = obb.confidence;
            }
            else
            {
                g.skip_reason = "PCA_TOO_FEW_CELLS";  // 保留当前几何
            }
        }

        out.push_back(g);
    }
}

} // namespace Lidar_Low_Detection

#endif // HISTORICAL_GEOMETRY_H
