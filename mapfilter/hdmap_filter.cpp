#include "hdmap_filter.h"
#include "coordinate_transformer.h"
#include "ElevationMapGroundFilter.h"   // GridCluster 完整定义

#include <cstdio>

namespace Lidar_Low_Detection
{

void HDMapFilter::configure(const Config& cfg)
{
    cfg_ = cfg;
    disabled_logged_ = false;
    frame_counter_   = 0;

    printf("[HDMapFilter] configured: enabled=%d mode=%s expand=%.2fm logEveryN=%d\n",
           cfg_.enabled ? 1 : 0,
           cfg_.filterMode == 1 ? "cluster_soft" : "cluster_mark",
           cfg_.expandDistance, cfg_.logEveryN);
}

void HDMapFilter::logDisabledOnce(const char* reason) const
{
    if (!disabled_logged_)
    {
        LOG_RAW("[HDMapFilter] disabled: %s\n", reason);
        disabled_logged_ = true;
    }
}

bool HDMapFilter::filterClusters(std::vector<GridCluster>& clusters,
                                 const LocalizationManager::Pose& pose,
                                 const HDMapManager& map,
                                 unsigned long long frame_id)
{
    ++frame_counter_;

    // ---- 降级策略：不满足条件时保留全部 Cluster，仅标记 ----
    if (!cfg_.enabled)
    {
        logDisabledOnce("filter disabled by config");
        return false;
    }
    if (!map.isLoaded())
    {
        logDisabledOnce("hdmap not loaded");
        for (auto& c : clusters)
        {
            c.map_valid = false;
        }
        return false;
    }
    if (!pose.valid)
    {
        logDisabledOnce("localization invalid");
        for (auto& c : clusters)
        {
            c.map_valid = false;
        }
        return false;
    }

    disabled_logged_ = false;   // 恢复后允许重新打印原因

    // ---- 构建车辆附近的可行驶区域多边形（地图系，每帧一次）----
    std::vector<std::vector<STR_POINT2F>> polygons;
    int section_count = 0;
    const bool poly_ok = map.buildDrivablePolygons(pose.x, pose.y, polygons, &section_count);

    if (!poly_ok)
    {
        // 车辆不在地图范围内 / 查询失败 → 保留全部 Cluster
        logDisabledOnce("vehicle outside map / no section found");
        for (auto& c : clusters)
        {
            c.map_valid = false;
        }
        return false;
    }

    const bool do_log = (cfg_.logEveryN > 0) && (frame_counter_ % cfg_.logEveryN == 0);

    // ---- 对每个 Cluster 打标签 ----
    for (size_t i = 0; i < clusters.size(); ++i)
    {
        GridCluster& c = clusters[i];

        // Cluster 中心（车体系，x前 y左）。优先 OBB 中心（转弯时更准确）
        const double veh_x = c.has_obb ? c.obb_center_x : c.center_x;
        const double veh_y = c.has_obb ? c.obb_center_y : c.center_y;

        // LocalizationManager::Pose -> VehiclePose（两者字段结构一致）
        VehiclePose vpose;
        vpose.x          = pose.x;
        vpose.y          = pose.y;
        vpose.heading_deg = pose.heading_deg;
        // 注：主雷达系(Grid)↔融合IMU系 航向补偿已在 CoordinateTransformer::vehicleToMap 内部应用
        //（main.cpp 启动时由 lidar.cfg fLidar2Vehicle_Heading 计算设置），与可视化一致，勿在此重复加。

        // 车辆系（其实是主激光系） → 地图系
        double map_x = 0.0, map_y = 0.0;
        CoordinateTransformer::vehicleToMap(veh_x, veh_y, vpose, map_x, map_y);

        // 点面测试（射线法 + 边界外扩）
        bool in_road = false;
        for (const auto& poly : polygons)
        {
            if (HDMapManager::pointInPolygon(map_x, map_y, poly))
            {
                in_road = true;
                break;
            }
        }
        if (!in_road && cfg_.expandDistance > 0.0)
        {
            for (const auto& poly : polygons)
            {
                if (HDMapManager::distanceToPolygon(map_x, map_y, poly) <=
                    cfg_.expandDistance)
                {
                    in_road = true;
                    break;
                }
            }
        }

        c.map_valid = true;
        c.in_road   = in_road;
        // 软约束置信度：道路内 1.0；道路外 0.3（仅标记，不删除）
        c.map_confidence = in_road ? 1.0f : 0.3f;

        if (do_log)
        {
            LOG_RAW("[HDMapFilter] frame=%llu cluster=%d center_lidar=(%.2f,%.2f) "
                   "center_map=(%.3f,%.3f) in_road=%d map_valid=%d confidence=%.2f\n",
                   static_cast<unsigned long long>(frame_id),
                   c.id, veh_x, veh_y, map_x, map_y,
                   c.in_road ? 1 : 0, c.map_valid ? 1 : 0, c.map_confidence);
        }
    }

    return true;
}

} // namespace Lidar_Low_Detection
