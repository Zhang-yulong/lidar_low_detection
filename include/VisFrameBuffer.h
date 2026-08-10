#pragma once
#ifndef VIS_FRAME_BUFFER_H
#define VIS_FRAME_BUFFER_H

#include <mutex>
#include <atomic>
#include <vector>
#include "Type.h"
#include "track.h"
#include "ReadYamlFile.h"
#include "ElevationMapGroundFilter.h"

namespace Lidar_Low_Detection
{

// ============================================================================
// 帧缓冲：工作线程 → 主线程 传递可视化数据
// 利用 shared_ptr 实现零拷贝的指针交换
// ============================================================================
class VisFrameBuffer
{
public:
    // 工作线程调用：发布最新一帧数据
    void Publish(const PointCloud2Intensity::Ptr& ground,
                 const PointCloud2Intensity::Ptr& obstacle,
                 const std::vector<TrackedObstacle>& tracks)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        ground_    = ground;
        obstacle_  = obstacle;
        tracks_    = tracks;
        has_new_.store(true, std::memory_order_release);
    }

    // 主线程调用：消费最新帧（有则返回 true）
    bool Consume(PointCloud2Intensity::Ptr& ground,
                 PointCloud2Intensity::Ptr& obstacle,
                 std::vector<TrackedObstacle>& tracks)
    {
        if (!has_new_.load(std::memory_order_acquire))
            return false;

        std::lock_guard<std::mutex> lock(mtx_);
        ground   = ground_;
        obstacle = obstacle_;
        tracks   = tracks_;
        has_new_.store(false, std::memory_order_release);
        return true;
    }

private:
    PointCloud2Intensity::Ptr    ground_;
    PointCloud2Intensity::Ptr    obstacle_;
    std::vector<TrackedObstacle> tracks_;
    std::mutex                   mtx_;
    std::atomic<bool>            has_new_{false};
};



// ============================================================================
// 可视化状态（主线程独占，无需加锁）
// ============================================================================
static std::shared_ptr<pcl::visualization::PCLVisualizer> ground_viewer = nullptr;
static int  g_lastClusterCount = 0;
static int  g_lastTrackCount   = 0;
static bool g_carBodyDrawn     = false;

// ---- 车身矩形轮廓 ----
static void DrawCarBodyOutlines(const SELF_DEBUG_CONFIG&              cfg,
                                const STR_ALL_LIDAR_CONFIG_INFO&      lidarInfo)
{
    if (g_carBodyDrawn || !ground_viewer) return;
    g_carBodyDrawn = true;

    float orig_hx = cfg.car_half_x;
    float orig_hy = cfg.car_half_y;
    float hz      = lidarInfo.toCarInfo.reviseGroudHeight;
    float exp_hx  = cfg.body_filter_x_threshold + cfg.car_half_x;
    float exp_hy  = cfg.body_filter_y_threshold + cfg.car_half_y;

    auto drawRect = [&](float hx, float hy, float hz, double r, double g, double b,
                        const std::string& prefix)
    {
        pcl::PointXYZ p0(-hx, -hy, hz);
        pcl::PointXYZ p1( hx, -hy, hz);
        pcl::PointXYZ p2( hx,  hy, hz);
        pcl::PointXYZ p3(-hx,  hy, hz);
        ground_viewer->addLine(p0, p1, r, g, b, prefix + "_L0");
        ground_viewer->addLine(p1, p2, r, g, b, prefix + "_L1");
        ground_viewer->addLine(p2, p3, r, g, b, prefix + "_L2");
        ground_viewer->addLine(p3, p0, r, g, b, prefix + "_L3");
    };
    drawRect(orig_hx, orig_hy, hz, 1.0, 0.0, 0.0, "car_orig");   // 红色
    drawRect(exp_hx,  exp_hy,  hz, 1.0, 1.0, 1.0, "car_exp");    // 白色
}

// ---- 初始化 viewer ----
static void InitGroundViewer()
{
    if (ground_viewer) return;
    ground_viewer = std::make_shared<pcl::visualization::PCLVisualizer>("GroundFilterViewer");
    ground_viewer->setBackgroundColor(0.05, 0.05, 0.05);
    ground_viewer->addCoordinateSystem(2.0);
}

// ---- 更新显示 (GridCluster 版本) ----
static void UpdateGroundViewer(const PointCloud2Intensity::Ptr&       pGroundCloud,
                               const PointCloud2Intensity::Ptr&       pObstacleCloud,
                               const std::vector<GridCluster>&        clusters)
{
    if (!ground_viewer) return;

    // 地面 (绿)
    {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> c(pGroundCloud, 0, 255, 0);
        if (!ground_viewer->updatePointCloud<pcl::PointXYZI>(pGroundCloud, c, "ground"))
        {
            ground_viewer->addPointCloud<pcl::PointXYZI>(pGroundCloud, c, "ground");
            ground_viewer->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "ground");
        }
    }
    // 障碍物 (红)
    {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> c(pObstacleCloud, 255, 0, 0);
        if (!ground_viewer->updatePointCloud<pcl::PointXYZI>(pObstacleCloud, c, "obstacle"))
        {
            ground_viewer->addPointCloud<pcl::PointXYZI>(pObstacleCloud, c, "obstacle");
            ground_viewer->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "obstacle");
        }
    }
    // cluster 包围盒
    for (int i = 0; i < g_lastClusterCount; ++i)
    {
        ground_viewer->removeShape("cluster_box_" + std::to_string(i));
        ground_viewer->removeShape("cluster_center_" + std::to_string(i));
    }
    for (size_t i = 0; i < clusters.size(); ++i)
    {
        const auto& c = clusters[i];
        std::string box_id    = "cluster_box_"    + std::to_string(i);
        std::string center_id = "cluster_center_" + std::to_string(i);
        ground_viewer->addCube(c.min_x, c.max_x, c.min_y, c.max_y, c.min_z, c.max_z,
                               1.0, 1.0, 1.0, box_id);
        ground_viewer->setShapeRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
            pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, box_id);
        ground_viewer->addSphere(pcl::PointXYZ(c.center_x, c.center_y, c.center_z),
                                 0.15, 1.0, 1.0, 0.0, center_id);
    }
    g_lastClusterCount = static_cast<int>(clusters.size());
}

// ---- 更新显示 (TrackedObstacle 版本) ----
static void UpdateGroundViewer(const PointCloud2Intensity::Ptr&       pGroundCloud,
                               const PointCloud2Intensity::Ptr&       pObstacleCloud,
                               const std::vector<TrackedObstacle>&    tracks)
{
    if (!ground_viewer) return;

    // 地面 (绿)
    {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> c(pGroundCloud, 0, 255, 0);
        if (!ground_viewer->updatePointCloud<pcl::PointXYZI>(pGroundCloud, c, "ground"))
        {
            ground_viewer->addPointCloud<pcl::PointXYZI>(pGroundCloud, c, "ground");
            ground_viewer->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "ground");
        }
    }
    // 障碍物 (红)
    {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> c(pObstacleCloud, 255, 0, 0);
        if (!ground_viewer->updatePointCloud<pcl::PointXYZI>(pObstacleCloud, c, "obstacle"))
        {
            ground_viewer->addPointCloud<pcl::PointXYZI>(pObstacleCloud, c, "obstacle");
            ground_viewer->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "obstacle");
        }
    }
    // tracked obstacle 包围盒
    for (int i = 0; i < g_lastTrackCount; ++i)
    {
        ground_viewer->removeShape("track_box_" + std::to_string(i));
        ground_viewer->removeShape("track_center_" + std::to_string(i));
        ground_viewer->removeText3D("track_id_" + std::to_string(i));
    }
    for (size_t i = 0; i < tracks.size(); ++i)
    {
        const auto& t = tracks[i];
        float half_h = t.height * 0.5f;
        float min_z = t.pos_z - half_h;
        float max_z = t.pos_z + half_h;
        float min_x = t.corners[0].x, max_x = t.corners[0].x;
        float min_y = t.corners[0].y, max_y = t.corners[0].y;
        for (int c = 1; c < 4; ++c)
        {
            if (t.corners[c].x < min_x) min_x = t.corners[c].x;
            if (t.corners[c].x > max_x) max_x = t.corners[c].x;
            if (t.corners[c].y < min_y) min_y = t.corners[c].y;
            if (t.corners[c].y > max_y) max_y = t.corners[c].y;
        }
        std::string box_id    = "track_box_"    + std::to_string(i);
        std::string center_id = "track_center_" + std::to_string(i);
        ground_viewer->addCube(min_x, max_x, min_y, max_y, min_z, max_z,
                               1.0, 1.0, 1.0, box_id);
        ground_viewer->setShapeRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
            pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, box_id);
        ground_viewer->addSphere(pcl::PointXYZ(t.pos_x, t.pos_y, t.pos_z),
                                 0.2, 1.0, 0.5, 0.0, center_id);
    }
    g_lastTrackCount = static_cast<int>(tracks.size());
}

// ---- 非阻塞 spin ----
static bool SpinGroundViewerOnce()
{
    if (!ground_viewer || ground_viewer->wasStopped())
        return false;
    ground_viewer->spinOnce(10);
    return !ground_viewer->wasStopped();
}



} // namespace Lidar_Low_Detection

#endif
