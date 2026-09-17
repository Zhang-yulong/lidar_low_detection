#pragma once
#ifndef STATIC_OBB_REFINEMENT_H
#define STATIC_OBB_REFINEMENT_H

/**
 * @file static_obb_refinement.h
 * @brief Phase 3-B —— STATIC Track OBB Geometry Refinement（header-only，基于 Cell 几何方向可观测性）
 *
 * 目标（只做一件事）：
 *   消除 STATIC 目标因当前帧 cell 排列变化（3 cells ↔ 4 cells ↔ 5 cells …）
 *   导致的 PCA 主方向随机跳变。
 *
 * 决策（只对 motion_state == STATIC + 本帧匹配成功 + pose 有效的 Track）：
 *
 *   yaw 来源三选一：
 *     ① PCA     ：当前 cells 的 PCA 主方向"可观测"（λ1/λ2 ≥ 阈值）且与历史 yaw 连续
 *     ② HISTORY ：PCA 不可观测（如 2×2 / 十字 / L 型），或 PCA 可观测但与历史 yaw 明显冲突
 *     ③ RAW     ：既不可观测又没有可用历史 yaw（不凭空创造方向）
 *
 *   length / width ：始终用最终 yaw 在当前帧 cell centers 上重新投影得到（不使用历史 L/W）
 *   center         ：始终使用当前 RAW center（不做 EMA / 不做历史位置修正）
 *
 * 本文件明确不做：
 *   历史 cell union / 历史轮廓融合 / 历史 L/W 平滑 / center 平滑 /
 *   候选角搜索（0°~180° 搜索、±15° 最小面积矩形）/ 代价函数优化 /
 *   Kalman / Hungarian / JPDA / MHT / 第二套 tracker。
 *
 * 复用关系：
 *   - cell 中心坐标复用 historical_geometry.h 的 HistCellCenter()（同一 Grid 约定，不重复实现）
 *   - 度制的折角函数 HistFoldYawDeg() 属旧 3-B（已停用）；本文件提供唯一的【弧度制】实现
 *     NormalizeYaw180()，Phase 3-B 全部使用它（项目 OBB 的 obb_angle 本身就是弧度）。
 */

#include "Type.h"        // Point2D

#include <cmath>
#include <cfloat>
#include <limits>
#include <vector>
#include <algorithm>

namespace Lidar_Low_Detection
{

// ============================================================================
// 开关与阈值（每个阈值都有独立物理含义；均为 initial tuning value，需用日志分布标定）
// ============================================================================

/// Phase 3-B 总开关（A/B 用）：false → 输出恒为 RAW
constexpr bool kObbRefineEnable = true;

/// 逐 STATIC Track 日志开关（默认开，便于验证"到底是 PCA 被接受还是 history fallback"）
constexpr bool kObbRefineVerbose = true;

/**
 * 主阈值：PCA 主方向可观测性（λ1/λ2 下限）
 *   物理含义：最大特征值方向相比次方向必须"明显"占优。
 *
 *   本项目实测（0.1m grid，按 ComputeClusterOBB 的 cov/n 公式计算）：
 *     ── 可观测（λ2 = 0，共线/准共线，方向非常明确）──
 *       3~5 cells 直线（横/纵/对角/斜向）        λ2 = 0        → +inf
 *       4 cells "3+1 在端部"（薄 L）             6.00
 *       4 cells 折线（staircase）               6.85
 *       5 cells T 型                            6.85
 *       4 直线 + 1 个偏置                        12.36
 *     ── 不可观测（方向缺乏区分度）──
 *       3 cells L 型 / "stair" / "2x2+1"        3.00
 *       5 cells L 型                            3.57
 *       4 cells T 型（竖在中间）                 2.67
 *       4 cells 2×2 方块                        1.00
 *       5 cells 十字型                           1.00
 *
 *   实测分布呈双峰，间隙落在 [3.57, 6.00]，取 4.0 落在间隙内部（两侧均有余量）。
 *   注意：这与"cell 数量门槛"无关（cells >= 3/4/5 都不构成方向可靠的充分条件）。
 *   若后续日志显示过多 STATIC 目标长期停在 "noUsefulYaw"，再考虑下调到 ~3.0
 *   （代价：3-cell L 型会正好落在边界上，区分度变差）。
 */
constexpr float kObbEigenRatioThreshold = 4.0f;

/**
 * 连续性阈值：当前 PCA yaw 与历史 yaw 的最大允许无向轴夹角（度）
 *   物理含义：对 STATIC 目标而言，"可靠方向"在帧间不应发生这么大幅度的变化；
 *             超过该值视为与历史明显冲突 → 保守使用历史 yaw。
 */
constexpr float kObbYawJumpMaxDeg = 30.0f;

/// 数值退化保护：λ1 小于该值时认为 cluster 几何完全退化（所有 cell 中心几乎重合）
constexpr float kObbLambdaDegenerate = 1e-9f;

/// PCA 有效所需的最小 cell 数（与上游 ComputeClusterOBB 的 n<3 → has_obb=false 一致；必要非充分）
constexpr int kObbMinCellsForPCA = 3;

constexpr float kObbPi     = 3.14159265358979323846f;
constexpr float kObbHalfPi = kObbPi * 0.5f;

// ============================================================================
// yaw 工具（无向轴：yaw ≡ yaw ± π）
// ============================================================================

/**
 * @brief 把无向轴角度归一化到 [-π/2, π/2)
 *
 * 例：π/2 → -π/2；-3π/2 → -π/2；π → 0
 * 非有限值（NaN/Inf）返回 0，避免调用方出现死循环或传播 NaN。
 */
inline float NormalizeYaw180(float yaw)
{
    if (!std::isfinite(yaw)) return 0.0f;
    while (yaw >=  kObbHalfPi) yaw -= kObbPi;
    while (yaw <  -kObbHalfPi) yaw += kObbPi;
    return yaw;
}

/**
 * @brief 两个无向轴的夹角，返回 [0, π/2]
 *
 * 禁止用 fabs(yaw1 - yaw2)：
 *   1° vs 179°  → 应为 2°（不是 178°）
 *   88° vs -89° → 应为 3°（不是 177°）
 * 同时天然覆盖 eigenvector 的 ±v 符号歧义（+10° 与 -170° 视为同一主轴方向）。
 */
inline float AngularDistance180(float yaw1, float yaw2)
{
    float d = NormalizeYaw180(yaw1 - yaw2);
    return (d < 0.0f) ? -d : d;
}

/// 弧度 → 度（仅日志/显示用）
inline float ObbRadToDeg(float rad) { return rad * 180.0f / kObbPi; }

// ============================================================================
// 方向可观测性
// ============================================================================

/**
 * @brief 当前 cluster 的 cell 几何是否能够提供"具有区分度的 PCA 主方向"
 *
 * 只回答几何问题，不回答"这是什么目标"。
 *
 * 关键点：
 *   - λ2 ≈ 0 【不是】"不可靠"：共线结构（3~4 个共线 cell，含对角共线）方向非常明确 → 返回 true。
 *   - 真正的退化（所有 cell 中心几乎重合 / NaN）才返回 false。
 *   - cell_count 只作为"PCA 是否成立"的必要条件（< 3 无 PCA），不作为可靠性充分条件，
 *     也不设置 3/4/5 之类的硬门槛。
 */
inline bool IsOrientationObservable(float lambda_max, float lambda_min, int cell_count)
{
    if (cell_count < kObbMinCellsForPCA)      return false;  // 必要非充分
    if (!(lambda_max > kObbLambdaDegenerate)) return false;  // 退化 / NaN
    if (lambda_min <= 0.0f)                   return true;   // 严格共线 → 强方向（绝非"不可靠"）
    return (lambda_max / lambda_min) >= kObbEigenRatioThreshold;
}

/// 观测到的主轴各向异性比 λ1/λ2（仅日志用；λ2=0 时为 +inf）
inline float EigenRatio(float lambda_max, float lambda_min)
{
    if (!(lambda_max > 0.0f)) return 0.0f;
    if (lambda_min <= 0.0f)   return std::numeric_limits<float>::infinity();
    return lambda_max / lambda_min;
}

// ============================================================================
// yaw 决策
// ============================================================================

enum class ObbYawSource : int
{
    RAW     = 0,   ///< 不做 refinement（输出 RAW）
    PCA     = 1,   ///< 采用当前帧 PCA 主方向
    HISTORY = 2    ///< 采用 STATIC Track 的历史 yaw
};

inline const char* ObbYawSourceName(ObbYawSource s)
{
    switch (s)
    {
        case ObbYawSource::PCA:     return "PCA";
        case ObbYawSource::HISTORY: return "HISTORY";
        default:                    return "RAW";
    }
}

/// yaw 决策的跳过原因（封闭枚举，便于脚本聚合）
enum ObbRefineSkip
{
    kObbSkipNone            = 0,
    kObbSkipNotStatic       = 1,   ///< motion_state != STATIC
    kObbSkipNoMatch         = 2,   ///< lastSeen != 0（本帧无匹配检测）
    kObbSkipNoCluster       = 3,   ///< 找不到本帧匹配的 Cluster
    kObbSkipNoObb           = 4,   ///< cluster.has_obb == false
    kObbSkipNoPose          = 5,   ///< pose 无效（无法做 map↔radar 的 yaw 投影）
    kObbSkipNoUsefulYaw     = 6    ///< 方向不可观测且无历史 yaw → 不凭空创造方向
};

/// 单 Track 决策结果
struct StaticObbDecision
{
    bool  refined    = false;      ///< false → 输出 RAW
    int   skip       = kObbSkipNone;
    ObbYawSource yaw_source = ObbYawSource::RAW;

    float yaw        = 0.0f;       ///< 最终 yaw（弧度，雷达系，已归一化到 [-π/2, π/2)）
    float pca_yaw    = 0.0f;       ///< 当前帧 PCA yaw（弧度）
    float hist_yaw   = 0.0f;       ///< 历史 yaw 投影到当前雷达系（弧度；无历史时无意义）
    bool  has_hist   = false;
    float yaw_delta  = 0.0f;       ///< AngularDistance180(pca, hist) ∈ [0, π/2]
    bool  observable = false;
    float eigen_ratio = 0.0f;
    bool  update_history = false;  ///< 是否需要把最终 yaw 写回历史（雷达系 → 地图系）
};

/**
 * @brief 纯决策：根据当前 PCA 与历史 yaw 决定最终 yaw 来源
 *
 *   可观测 + 无历史            → PCA（并初始化历史）
 *   可观测 + 与历史连续        → PCA（并更新历史）
 *   可观测 + 与历史明显冲突    → HISTORY（保守；历史不变）
 *   不可观测 + 有历史          → HISTORY（本阶段的核心功能）
 *   不可观测 + 无历史          → RAW（不凭空创造方向）
 */
inline StaticObbDecision DecideStaticObbYaw(float pca_yaw,
                                            float lambda_max,
                                            float lambda_min,
                                            int   cell_count,
                                            bool  has_hist,
                                            float hist_yaw)
{
    StaticObbDecision d;
    d.pca_yaw     = NormalizeYaw180(pca_yaw);
    d.hist_yaw    = hist_yaw;
    d.has_hist    = has_hist;
    d.observable  = IsOrientationObservable(lambda_max, lambda_min, cell_count);
    d.eigen_ratio = EigenRatio(lambda_max, lambda_min);

    const float kJumpMaxRad = kObbYawJumpMaxDeg * kObbPi / 180.0f;

    if (d.observable)
    {
        if (!has_hist)
        {
            d.yaw = d.pca_yaw;
            d.yaw_source = ObbYawSource::PCA;
            d.update_history = true;
            d.refined = true;
            return d;
        }

        d.yaw_delta = AngularDistance180(d.pca_yaw, hist_yaw);
        if (d.yaw_delta <= kJumpMaxRad)
        {
            d.yaw = d.pca_yaw;
            d.yaw_source = ObbYawSource::PCA;
            d.update_history = true;
            d.refined = true;
        }
        else
        {
            // 不因为 Eigen 给出了主方向就认定目标真实旋转了 180°/大角度 → 保守
            d.yaw = NormalizeYaw180(hist_yaw);
            d.yaw_source = ObbYawSource::HISTORY;
            d.update_history = false;
            d.refined = true;
        }
    }
    else
    {
        if (has_hist)
        {
            d.yaw = NormalizeYaw180(hist_yaw);
            d.yaw_source = ObbYawSource::HISTORY;
            d.update_history = false;
            d.refined = true;
        }
        else
        {
            d.refined = false;
            d.skip = kObbSkipNoUsefulYaw;
        }
    }
    return d;
}

// ============================================================================
// 几何：用 final yaw 在当前 cells 上重新投影得到 L/W，并重建 corners
// ============================================================================

/**
 * @brief 以给定 yaw 为轴，计算当前帧 cell centers 的投影范围
 *
 *   U = (cos yaw,  sin yaw)   → length 方向
 *   V = (-sin yaw, cos yaw)   → width  方向
 *   u = dot(center, U), v = dot(center, V)
 *   length = max_u - min_u,  width = max_v - min_v
 *
 * 尺寸语义与现有 ComputeClusterOBB 完全一致（只覆盖 cell 中心，不含 ±半格 padding），
 * 不改变项目既有的 OBB 尺寸定义。
 */
inline void ComputeYawAlignedExtents(const std::vector<Point2D>& centers,
                                     float  yaw,
                                     float& length,
                                     float& width)
{
    length = 0.0f;
    width  = 0.0f;
    if (centers.empty()) return;

    const float c = std::cos(yaw);
    const float s = std::sin(yaw);

    float min_u =  FLT_MAX, max_u = -FLT_MAX;
    float min_v =  FLT_MAX, max_v = -FLT_MAX;

    for (const auto& p : centers)
    {
        const float u =  p.x * c + p.y * s;
        const float v = -p.x * s + p.y * c;
        min_u = std::min(min_u, u); max_u = std::max(max_u, u);
        min_v = std::min(min_v, v); max_v = std::max(max_v, v);
    }

    length = max_u - min_u;
    width  = max_v - min_v;
}

/**
 * @brief 用 (center, yaw, length, width) 重建 4 角点
 *
 * 顺序与 ComputeClusterOBB / TrackedObstacle / S2obstacleBox 保持一致：
 *   左下 → 右下 → 右上 → 左上（corner0→corner1 沿长轴方向）
 */
inline void BuildObbCorners(float cx, float cy, float yaw,
                            float length, float width,
                            Point2D out_corners[4])
{
    const float c  = std::cos(yaw);
    const float s  = std::sin(yaw);
    const float hl = length * 0.5f;
    const float hw = width  * 0.5f;

    out_corners[0] = Point2D{ cx - c * hl + s * hw, cy - s * hl - c * hw };  // 左下
    out_corners[1] = Point2D{ cx + c * hl + s * hw, cy + s * hl - c * hw };  // 右下
    out_corners[2] = Point2D{ cx + c * hl - s * hw, cy + s * hl + c * hw };  // 右上
    out_corners[3] = Point2D{ cx - c * hl - s * hw, cy - s * hl + c * hw };  // 左上
}

} // namespace Lidar_Low_Detection

#endif // STATIC_OBB_REFINEMENT_H
