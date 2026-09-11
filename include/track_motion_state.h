#pragma once
#ifndef TRACK_MOTION_STATE_H
#define TRACK_MOTION_STATE_H

/**
 * @file track_motion_state.h
 * @brief Phase 3-A —— Track 运动状态（UNKNOWN / STATIC / MOVING）定义与阈值常量（header-only）
 *
 * 设计目的（详见 docs/Phase3A_MotionState_CodeAnalysis.md）：
 *   Motion State 判断的不是“物理目标有没有动”，而是
 *   “该 Track 的历史信息是否足够稳定、是否可以信任”。
 *
 *     UNKNOWN = 历史证据不足，暂不信任运动/静止判断（新 Track 默认）
 *     STATIC  = Map frame 中位置基本稳定（有足够历史 + 无持续同向运动证据）
 *     MOVING  = Map frame 中存在持续、同向、超过 grid/localization jitter 的明显运动
 *
 * 判断必须在 Map frame（vehicleToMap）进行，以消除自车运动。
 *
 * 本文件不含任何算法实现，仅提供枚举 + 阈值常量 + 名称，供 track.h / SuTengDriver 使用。
 * 阈值语义见 docs/Phase3A_MotionState_CodeAnalysis.md §6，均可在实现处调整。
 */

namespace Lidar_Low_Detection
{

/// Track 运动状态（写入 TrackedObstacle.motion_state，可从 m_tracker.vtrackings 读取）
enum class MotionState : int
{
    UNKNOWN = 0,   ///< 历史证据不足
    STATIC  = 1,   ///< 地图系位置基本稳定
    MOVING  = 2    ///< 地图系持续同向明显位移
};

// ---- 阈值（语义分组，非物理真值；Grid resolution = 0.1m）----
constexpr float kMotionStaticStepMax   = 0.08f;  // 单帧位移 <= 此值 → 静止抖动证据
constexpr float kMotionMoveStepMin     = 0.12f;  // 单帧位移 >= 此值 → 一步明显位移（> 1 Grid）
constexpr float kMotionMoveNetMin      = 0.30f;  // 观察窗口内净位移下限（排除来回抖动）
constexpr int   kMotionMoveWin         = 3;      // 净位移观察窗口（步数）
constexpr int   kMotionMMove           = 3;      // 进入 MOVING 所需的连续运动证据帧数
constexpr float kMotionDirTolDeg       = 60.0f;  // 运动方向一致性容差（度）
constexpr int   kMotionUnknownToStatic = 4;      // UNKNOWN → STATIC 所需连续静止证据帧数
constexpr int   kMotionMovingToStatic  = 6;      // MOVING → STATIC 所需连续静止证据帧数（迟滞）
constexpr float kMotionVEmaAlpha       = 0.3f;   // 地图系速度 EMA 平滑系数
constexpr int   kMotionHistoryCap      = 8;      // 最近有效 map position 上限

inline const char* MotionStateName(MotionState s)
{
    switch (s)
    {
        case MotionState::STATIC: return "STATIC";
        case MotionState::MOVING: return "MOVING";
        default:                  return "UNKNOWN";
    }
}

} // namespace Lidar_Low_Detection

#endif // TRACK_MOTION_STATE_H
