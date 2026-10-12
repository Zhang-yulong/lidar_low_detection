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
#include "historical_geometry.h"   // Phase 3-B: STATIC Historical Geometry
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

    int test_Frame_count =0;

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

    // ---- Phase 3-B: STATIC Historical Geometry ----
    // 对 STATIC + 已关联的 historical track，用历史 cell footprint 安全补充当前 Cluster。
    // 复用 Phase 2 的 m_historicalFeedback；必须在 UpdateMapAnchors() 之后调用，
    // 以保证 Phase 2 锚点始终基于原始检测几何（不被 fused 几何污染）。
    std::vector<HistoricalGeometryResult> m_historicalGeometry;
    void ApplyHistoricalGeometry(const std::vector<GridCluster>& clusters);
    // ---- Phase 3-B (v2): STATIC Track OBB Geometry Refinement -------------
    // 只对 STATIC + 本帧匹配成功 + pose 有效 的 Track：
    //   yaw    = 当前 PCA yaw（方向可观测且与历史连续）
    //            / 历史 yaw（方向不可观测，或可观测但与历史明显冲突）
    //            / RAW（不可观测且无历史 yaw，不凭空创造方向）
    //   L/W    = 用最终 yaw 在当前帧 cells 上重新投影（不使用历史 L/W）
    //   center = 当前 RAW center（不做任何位置平滑）
    // 结果只写入 outTracks（RAW 快照副本），【不回写 m_tracker】；
    // 必须在 UpdateMapAnchors()（基于 RAW 几何）之后调用，避免形成反馈回路。
    void RefineStaticObbGeometry(const std::vector<GridCluster>& clusters,
                                 const LocalizationManager::Pose& pose,
                                 bool pose_valid,
                                 std::vector<TrackedObstacle>& outTracks);

    // ---- Phase 3-B': 1cm Raw Point Image OBB（实验，仅 Debug 可视化）------------
    // 把本帧【匹配成功】的 Track 对应 Cluster 的 img_corners 写入 outTracks 副本的
    // 实验字段 TrackedObstacle::new_corners / has_new_corners。
    // ⚠ 只写这两个实验字段：
    //    - 不写 corners / depth / width / pos_*（现有输出 / UDP 语义完全不变）
    //    - 不写 m_tracker.vtrackings（Tracker State 永远不含实验字段）
    //    - 不写 detections（tracker 关联 / 速度 / ID 与 baseline 完全一致）
    void AttachRawImageObbs(const std::vector<GridCluster>& clusters,
                            std::vector<TrackedObstacle>& outTracks) const;
    VisFrameBuffer m_visBuffer;    // 帧缓冲：工作线程 → 主线程

    // ========================================================================
    // Local Missing Target Recheck（第一版，STATIC only）
    //
    // 职责（极窄）：对【已经存在、本帧被正常 Tracker 判为 miss 的 STATIC Track】，
    //   在其预测位置的 3×3 Grid ROI 内做一次局部 raw point recheck；
    //   若局部原始点云仍提供存在性证据 → 把该 Track 标记为 RECOVERED。
    //
    // 它【不是】第二个检测器 / 第二个 Tracker / OBB 重建器：
    //   - 不重跑 m_tracker.update()，不构造 detection，不新增 ID
    //   - 不修改 lastSeen / age / vx,vy / geometry / map anchor / m_mapTracks
    //   - 不写 outputClusters（Recovered 不进入 Cluster 数据）
    //   - 不修改 Ground Filter / Grid / Cluster / HDMap / Localization / UDP
    //
    // 时序（必须）：m_tracker.update() → UpdateTrackMotionStates() → UpdateMapAnchors()
    //               → 【本函数】 → RefineStaticObbGeometry()
    //   - 在 UpdateTrackMotionStates() 之后：miss Track 的 pos_x/pos_y 已是
    //     “地图锚点 → 当前帧雷达系”的预测位置（本函数直接用它做 ROI 中心）
    //   - 在 UpdateMapAnchors() 之后：该函数的 miss 语义完全不变
    //   - 在 RefineStaticObbGeometry() 之前：RECOVERED 的 current_exist/几何能被
    //     outTracks 快照带到 Debug / visBuffer / UDP
    // ========================================================================

    /// 每个 Recheck 候选 Track 的工作槽（固定小尺寸，无堆分配）
    struct RecheckTrackSlot
    {
        int      track_index = -1;   ///< m_tracker.vtrackings 下标
        uint32_t bit         = 0;    ///< 该槽在 m_recheckMask 中的 bit（1u << slot）
        int      center_row  = 0;    ///< ROI 中心 cell（Grid row/col）
        int      center_col  = 0;

        // ---- Ring Ground（5×5 \ 3×3 中被正常 Ground Filter 判为 ground 的 cell）----
        bool  ground_valid      = false; ///< ring ground cell 数 >= kRecheckMinRingGroundCells
        int   ring_ground_cells = 0;
        float local_ground_z    = 0.0f;  ///< ring ground cell 的 min_z 中位数

        // ---- 3×3 ROI 内的 raw point 证据（行主序：(dr+1)*3 + (dc+1)）----
        uint16_t roi_evidence[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
        int      evidence_points = 0;    ///< 异常于局部地面的点数
        int      current_cells   = 0;    ///< 被上述点占用的 ROI cell 数

        // ---- 判定 ----
        int     prev_normal_cells = -1;  ///< 参考基准（最近一次 NORMAL detection 的 cluster cell 数）
        int     cell_diff         = 0;   ///< current_cells - prev_normal_cells
        uint8_t result            = 0;   ///< RecheckResult（仅日志/诊断）
    };

    /// Recheck 结果（日志用；REJECT_* 表示本次不恢复，Track 状态保持不变）
    enum class RecheckResult : uint8_t
    {
        NOT_RUN = 0,
        RECOVERED,
        REJECT_NO_RING_GROUND,   ///< ring 内可靠 ground cell < 3（不做任何 fallback）
        REJECT_NO_EVIDENCE,      ///< 3×3 ROI 内没有当前证据（如 current_cells == 0）
        REJECT_NO_REFERENCE,     ///< 尚无 NORMAL detection 的 cell 数参考基准
        REJECT_CELL_COUNT_JUMP   ///< |current - previous| > kRecheckMaxCellDiff
    };
    static const char* RecheckResultName(RecheckResult r);

    /// Recheck 入口（在 UpdateMapAnchors() 之后、RefineStaticObbGeometry() 之前调用）
    void RunLocalMissingTargetRecheck(const pcl::PointCloud<pcl::PointXYZI>& rawCloud,
                                      const std::vector<GridCluster>& clusters,
                                      bool pose_valid);

    /// 记录本帧【正常 detection】的 cluster cell 数（Recheck 的稳定性参考基准）。
    /// ⚠ 只由 NORMAL detection 更新：RECOVERED 帧不更新（防止参考基准自我强化漂移）。
    void RecordNormalDetectionHistory(const std::vector<GridCluster>& clusters);

    /// 收集候选 Track + 构建 3×3 recheckMask + ring ground；返回候选数
    int CollectRecheckTracksAndBuildMasks(const std::vector<GridCluster>& clusters);

    /// 一次 raw point scan：点 → Grid cell → recheckMask → 累加到对应 Track 的 evidence
    void ScanRecheckRawPointsOnce(const pcl::PointCloud<pcl::PointXYZI>& rawCloud);

    /// 判定 + 写回 RECOVERED 状态（失败时不动 Track 的任何状态）
    void EvaluateAndApplyRecheckResults();

    static constexpr int kRecheckMaxTracks = 32;  ///< m_recheckMask 为 uint32_t，最多 32 个候选

    // ---- 跨帧复用缓冲（避免逐帧堆分配）----
    std::vector<uint32_t>         m_recheckMask;   ///< rows*cols，每个 bit = 一个 Recheck slot
    std::vector<uint8_t>          m_recheckClaimedCells; ///< rows*cols，1 = 已被本帧正常 Cluster 占据
    std::vector<RecheckTrackSlot> m_recheckSlots;  ///< 本帧候选（每帧重建，尺寸很小）
    
    ///< 第一版开关：false → 完全绕过（零行为变化）
    bool m_localRecheckEnable = true;       
           
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