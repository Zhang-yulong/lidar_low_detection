#pragma once
#ifndef SUTENG_DRIVER_H
#define SUTENG_DRIVER_H

#include "BaseDriver.h"
#include "CommonGroundDetection.h"
#include "LidarCurbDetection.h"
#include "ElevationMapGroundFilter.h"
#include "DebugViewer.h"
#include "track.h"
#include "VisFrameBuffer.h"
#include "hdmap_manager.h"
#include "hdmap_filter.h"
#include "historical_feedback.h"
#include <thread>
#include <atomic>
#include "ReadYamlFile.h"
// #include <3rdparty/rs_driver/api/lidar_driver.hpp>
// #include <3rdparty/rs_driver/msg/pcl_point_cloud_msg.hpp>

#include <rs_driver/api/lidar_driver.hpp>
#include <rs_driver/msg/point_cloud_msg.hpp>
#include <rs_driver/common/rs_common.hpp>
#include <rs_driver/common/error_code.hpp>
using namespace robosense::lidar;




// class LidarDriver;

namespace Lidar_Low_Detection
{

typedef PointXYZI PointT;
typedef PointCloudT<PointT> PointCloudMsg;

class SutengDriver: public BaseDriver{

public:
    SutengDriver();
    SutengDriver(const SELF_DEBUG_CONFIG &config, const STR_ALL_LIDAR_CONFIG_INFO &allLidarTransInfo);
    ~SutengDriver();
    void Start();
    int Init();
    void Stop();
    void Free(); 

    // int InitParams(const int MsopPort, const int DifopPort, const std::string &LidarType);
    bool loadPointCloud(const std::string& file_path, PointCloud2Intensity::Ptr& cloud);
    void PointCloudTransform(PointCloud2Intensity::Ptr &pInputCloud, const Eigen::Matrix4f &R_Combined, PointCloud2Intensity::Ptr &pOutputCloud);
    void SavePcd(const PointCloud2Intensity::Ptr &InputCloud);

    std::shared_ptr<PointCloudMsg> driverGetPointCloudFromCallerCallback(void);
    void driverReturnPointCloudToCallerCallback(std::shared_ptr<PointCloudMsg> msg);
    void exceptionCallback(const robosense::lidar::Error& code);
    
    void ProcessPcapCloud();

    // 帧缓冲：工作线程发布数据，主线程消费渲染
    VisFrameBuffer& GetVisBuffer() { return m_visBuffer; }

    /* ---- 可视化已迁移至 main.cpp（注释保留以备后用） ----
    // 共用可视化接口：初始化 / 更新 / 交互循环
    void InitGroundViewer();
    void UpdateGroundViewer(const PointCloud2Intensity::Ptr &pGroundCloud,
                            const PointCloud2Intensity::Ptr &pObstacleCloud,
                            const std::vector<GridCluster> &clusters);
    void UpdateGroundViewer(const PointCloud2Intensity::Ptr &pGroundCloud,
                            const PointCloud2Intensity::Ptr &pObstacleCloud,
                            const std::vector<TrackedObstacle> &tracks);
    bool SpinGroundViewerOnce();  // 返回 true 表示窗口还在运行

    // 一次性绘制车身矩形轮廓（原始红色 + 外扩白色，z=0）
    void DrawCarBodyOutlines();
    ---- */

    int sendUdpMsg(int sk, unsigned char *pstrMsg, unsigned short usLen, unsigned int IpAddr, unsigned short usPort);

private:

    const SELF_DEBUG_CONFIG *m_ST_Config;
    const STR_ALL_LIDAR_CONFIG_INFO *m_ST_AllLidarTransfromInfo;


    RSDriverParam m_rs_param; 
    LidarDriver<PointCloudMsg> m_driver;
    int m_iMsopPort; //数据端口
    int m_iDifopPort; //设备端口
    std::string m_sSuTengType;
    SyncQueue<std::shared_ptr<PointCloudMsg>> free_cloud_queue;
    SyncQueue<std::shared_ptr<PointCloudMsg>> stuffed_cloud_queue;
    std::thread m_processThread;


    std::atomic<bool> m_running{false};
    std::thread m_SutengDriverThread;

    std::unique_ptr<ElevationMapGroundFilter> m_pElevationMapGroundFilter;

    std::unique_ptr<DebugViewer> m_debugViewer;        // 调试可视化模块
    
    CommonGroundDetection *m_pCommonGroundDetection;

    // ---- HDMap + 定位（迁移设计文档 Phase 1：Cluster 级软约束）----
    HDMapManager  m_hdmapManager;   // HDMap 管理器（启动时加载一次，只读共享）
    HDMapFilter   m_hdmapFilter;    // HDMap Cluster 级软约束（Cluster 后、Tracker 前）
    bool          m_hdmapEnabled = false;  // 本实例是否启用 HDMap 过滤（可配置关闭）

    // 跟踪器：为障碍物分配稳定的跨帧 ID（从 9999 起始）
    SimpleTracker m_tracker;

    // GridCluster → TrackedObstacle 转换
    void ConvertClustersToTrackedObstacles(
        const std::vector<GridCluster>& clusters,
        std::vector<TrackedObstacle>& out) const;

    // ---- Historical Feedback（Phase 2/3 Quick Validation，实验性，不影响正式输出）----
    // 地图系锚定的历史 Track 侧表（每帧 Track 匹配成功时更新，miss 时冻结）
    std::vector<MapAnchoredTrack> m_mapTracks;
    // 本帧投影到当前 Grid 的 Historical Feedback Region（仅供 Debug 可视化/日志）
    std::vector<HistoricalFeedbackRegion> m_historicalFeedback;

    /// 将历史 Map Track 投影到当前雷达系得预测位置 A → A 转 Base Cell → 3×3 邻居搜索
    /// → 候选当前 Cluster → 关联（SAME_TRACK_ID / 几何门控）；打印 [HistoricalAssociation] 日志
    void ComputeHistoricalFeedback(
        const std::vector<GridCluster>& clusters,
        const LocalizationManager::Pose& pose,
        bool pose_valid,
        std::vector<HistoricalFeedbackRegion>& out);

    /// 每帧结束后用当前 Track + 当前位姿维护地图锚点侧表（miss 时冻结）
    void UpdateMapAnchors(const LocalizationManager::Pose& pose, bool pose_valid);

    /// Phase 3-A：在 m_tracker.update() 之后，用地图系位置历史维护每个 Track 的
    /// Motion State（UNKNOWN/STATIC/MOVING）、map position、map 速度/方向。
    /// 只读 pose + track 位置，写回 m_tracker.vtrackings 的 Phase 3-A 新增字段；
    /// 不修改 tracker 匹配/删除，不修改 Phase 2 关联。
    void UpdateTrackMotionStates(const LocalizationManager::Pose& pose,
                                 bool pose_valid,
                                 unsigned long long rec_timestamp_ms);

    VisFrameBuffer m_visBuffer;    // 帧缓冲：工作线程 → 主线程

    /* ---- 可视化成员已迁移至 main.cpp（注释保留） ----
    std::shared_ptr<pcl::visualization::PCLVisualizer> st_pcl_viewer = nullptr;

    std::shared_ptr<pcl::visualization::PCLVisualizer> ground_viewer = nullptr;
    int m_lastClusterCount = 0;  // 上一帧绘制的 cluster 数量，用于清理旧 shape
    int m_lastTrackCount = 0;    // 上一帧绘制的 tracked obstacle 数量，用于清理旧 shape
    bool m_carBodyDrawn = false; // 车身矩形是否已绘制（只画一次）
    ---- */

};
}
#endif