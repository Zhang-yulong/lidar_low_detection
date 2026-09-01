#pragma once
#ifndef _DEBUG_FRAME_H_
#define _DEBUG_FRAME_H_

#include <vector>

#include "Type.h"
#include "ElevationMapGroundFilter.h"
#include "track.h"

namespace Lidar_Low_Detection
{

/**
 * @brief 一帧低矮检测算法处理完成后的调试快照（DebugFrame）
 *
 * DebugFrame 不是新的算法数据结构，也不改变任何算法数据结构。
 * 它只负责保存后续 Foxglove 调试显示需要的数据，作为"一帧算法结果"的独立快照。
 *
 * 数据来源与生命周期（必须与算法线程缓冲隔离，禁止保存算法临时对象的裸指针）：
 *   - timestamp_ms : 值保存，来自 rec_timestamp_ms（雷达包时间戳，ms）
 *   - pointcloud   : 方案 B —— 构造时对 pFilteredPointCloud 做一次独立深拷贝
 *                    （std::make_shared<PointCloud2Intensity>(*pFilteredPointCloud)）。
 *                    DebugFrame 用 shared_ptr 持有独立快照，绝不保存算法线程正在
 *                    复用的 pcl::PointCloud 对象的裸指针或同一 shared_ptr
 *                    （否则下一帧 clear() 会就地改写，Consumer 读到被污染数据）。
 *   - clusters     : 按值保存 outputClusters（含 HDMap 标签），源是每帧新 vector
 *   - trackings    : 按值保存 m_tracker.vtrackings（update 每帧就地改写，必须拷贝）
 */
struct DebugFrame
{
    unsigned long long timestamp_ms = 0;          // = rec_timestamp_ms（ms）

    PointCloud2Intensity::Ptr pointcloud;         // 独立深拷贝快照（shared_ptr 持有）

    std::vector<GridCluster>      clusters;       // = outputClusters（按值）
    std::vector<TrackedObstacle>  trackings;      // = m_tracker.vtrackings（按值）
};

} // namespace Lidar_Low_Detection

#endif // _DEBUG_FRAME_H_
