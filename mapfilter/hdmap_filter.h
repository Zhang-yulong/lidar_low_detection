#ifndef LOW_DETECTION_HDMAP_FILTER_H
#define LOW_DETECTION_HDMAP_FILTER_H

#include <cstdint>
#include <vector>

#include "localization_manager.h"
#include "hdmap_manager.h"

namespace Lidar_Low_Detection
{

// 前向声明（完整定义在 .cpp 中 include ElevationMapGroundFilter.h）
struct GridCluster;

/**
 * @brief HDMap 过滤：Cluster 级软约束（第一阶段只做标签，不删除 Cluster）
 *
 * 输入：GridCluster 列表 + 车辆位姿 + HDMap
 * 输出：为每个 Cluster 打上标签
 *   - in_road       : 是否位于地图可行驶(道路)区域
 *   - map_valid     : 本次过滤时 定位+地图 是否有效
 *   - map_confidence: 地图置信度（1.0=道路内；低值=道路外/未知）
 *
 * 安全策略（见迁移文档第 21/22/27/28 章）：
 *   - 定位无效 / 地图未加载 / 查询失败 → 全部 Cluster 保留，map_valid=false
 *   - 默认只标记（filterMode=0），不做 clusters.erase()
 *   - 地图只是"额外空间先验"，不是"道路=无障碍"的真值
 *
 * 位置：ProcessPcapCloud 中 Cluster 生成之后、Tracker 之前。
 */
class HDMapFilter
{
public:
    struct Config
    {
        bool  enabled        = false;   ///< 总开关
        int   filterMode     = 0;       ///< 0=仅标记(默认) 1=软约束(降置信度)
        float expandDistance = 0.5f;    ///< 道路边界外扩距离(m)，防误删
        int   logEveryN      = 50;      ///< 每 N 帧打印一次 cluster 级日志
    };

    void configure(const Config& cfg);
    const Config& config() const { return cfg_; }

    /**
     * @brief 对 Cluster 列表做地图软约束
     * @param clusters  GridCluster 列表（就地修改 in_road/map_valid/map_confidence）
     * @param pose      车辆位姿（LocalizationManager::Pose）
     * @param map       HDMapManager
     * @param frame_id  帧号（调试日志用）
     * @return true 表示本次执行了地图过滤（定位+地图有效）
     */
    bool filterClusters(std::vector<GridCluster>& clusters,
                        const LocalizationManager::Pose& pose,
                        const HDMapManager& map,
                        unsigned long long frame_id);

private:
    void logDisabledOnce(const char* reason) const;

    Config   cfg_;
    mutable bool disabled_logged_ = false;   // 日志缓存（mutable 便于 const 方法打印）
    uint64_t frame_counter_   = 0;
};

} // namespace Lidar_Low_Detection

#endif // LOW_DETECTION_HDMAP_FILTER_H
