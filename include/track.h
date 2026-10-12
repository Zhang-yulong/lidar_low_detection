#pragma once
#ifndef _TRACK_H_
#define _TRACK_H_


#include <cmath>
#include <algorithm>
#include <sys/time.h>
#include <atomic>
#include <chrono>
#include <cstdint>

#include <ctime> // 必须包含，提供 time_t 和 struct tm

#include "opencv2/opencv.hpp" 

#include "ulog_api.h"
#include "Type.h"
#include "track_motion_state.h"   // Phase 3-A: MotionState 枚举 + 阈值常量
namespace Lidar_Low_Detection
{
extern int send_fd;
extern unsigned int ip;
extern unsigned short port;


// 简化的卡尔曼滤波器类，用于二维恒定速度模型
class KalmanFilter2D {
public:
    // 状态向量: [pos_x, pos_y, vel_x, vel_y]
    std::array<float, 4> state;
    // 协方差矩阵 (简化为对角矩阵)
    std::array<float, 4> P;
    // 过程噪声协方差
    std::array<float, 4> Q;
    // 测量噪声协方差
    std::array<float, 2> R;

    KalmanFilter2D(float init_x, float init_y);
        
    // 预测步骤
    void predict(float dt = 1.0f);

    // 更新步骤
    void update(float meas_x, float meas_y);

    float getPosX() const { return state[0]; }
    float getPosY() const { return state[1]; }
};


// ============================================================================
// Local Missing Target Recheck（第一版）：Track 本帧“观测来源”标记
//
//   NORMAL    = 本帧由正常链路得到：GroundFilter → Cluster → SimpleTracker 关联匹配
//   RECOVERED = 本帧 Tracker 判定 miss，但 Local Missing Target Recheck 在
//               预测位置 3×3 ROI 的 raw point 中找到了存在性证据，
//               对【已存在的 Track】做 post-association repair

//   NORMAL_MISS = 本帧 Tracker 判定 miss,没有进入Local Missing Target Recheck阶段
//   RECOVERED_MISS = 本帧Local Missing Target Rechec阶段 没有找到存在性证据


//
// ⚠ 语义边界（重要）：
//   - 它只描述“本帧这条 Track 的观测来自哪里”，不参与任何生命周期决策；
//   - Recheck 成功后【不修改 lastSeen】：lastSeen 始终表示
//     “正常 Tracker detection/association 意义上的连续漏检帧数”；
//   - 删除仍然只由 SimpleTracker::removeLongLostTargets() 负责。
// ============================================================================
enum class DetectionSource : uint8_t
{
    NORMAL    = 0,   ///< 正常 detection
    RECOVERED = 1,    ///< Recheck 恢复（existence recovery）
    NORMAL_MISS = 2,            // 
    RECOVERED_MISS = 3
};

// 定义一个带ID的障碍物结构体
struct TrackedObstacle {
    int cluster_id = -1;   // 对应的簇 ID（可选，用于调试或溯源）
    int id = -1;           // 跟踪ID
    float pos_x = 0.0f;    // 中心点X
    float pos_y = 0.0f;    // 中心点Y
    float pos_z = 0.0f;    // 中心点 Z
    Point2D corners[4] = {}; 	//二维平面上的4个顶点
    float depth = 0.0f;    // 深度
    float width = 0.0f;    // 宽度
    float height = 0.0f;   // 高度
    int age = 0;           // 存活帧数（用于调试或清理）
    int lastSeen = 0;      // 最后一次见到的帧数（用于清理消失的目标）
    float Translation[3] = {0.0f, 0.0f, 0.0f};
    float Rotation[9] ={0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};     //!< 旋转矩阵

    float vx = 0.0f;
    float vy = 0.0f;

    // ========================================================================
    // Phase 3-A: Motion State（地图系运动状态，判断“历史信息是否可信”）
    //   ⚠ 注意：以下字段必须在【拷贝构造 / 拷贝赋值】中同步拷贝（见下方手写实现）
    //   vx/vy 保持【主雷达系】语义不变，地图系速度单独用 map_vx/map_vy。
    // ========================================================================
    MotionState motion_state = MotionState::UNKNOWN; ///< UNKNOWN / STATIC / MOVING
    bool  has_map_pos        = false;    ///< 是否已有有效 matched map position
    float map_x = 0.0f;                  ///< 最近一次有效 map position X (地图系 ENU)
    float map_y = 0.0f;                  ///< 最近一次有效 map position Y (地图系 ENU)
    // 最近一次 matched 时的 OBB 长轴朝向（地图系 ENU, deg, 无向 [-180,180)）。
    // 用途：miss 帧的可视化需要 雷达系 yaw = f(地图系 yaw, 当前 pose)，
    //       否则自车转向时用旧雷达系 yaw 重建的框方向会漂（“已知残余误差”）。
    float map_yaw_deg = 0.0f;            ///< 地图系 OBB 长轴朝向 (deg)
    bool  has_map_yaw = false;           ///< map_yaw_deg 是否有效
    float map_vx = 0.0f;                 ///< 地图系速度 X (m/s, EMA)
    float map_vy = 0.0f;                 ///< 地图系速度 Y (m/s, EMA)
    float motion_step = 0.0f;            ///< 最近一次地图系帧间位移 (m)
    float motion_net  = 0.0f;            ///< 观察窗口内净位移 (m)
    float motion_dir_deg = 0.0f;         ///< 最近一次位移方向 (deg)
    int   motion_static_run = 0;         ///< 连续静止证据计数
    int   motion_moving_run = 0;         ///< 连续运动证据计数
    std::vector<Point2D> recent_map_positions; ///< 最近 kMotionHistoryCap 个有效 matched map position

    //==================
    // Phase 3-B' 实验字段：1cm Raw Point Image + Grid ROI + minAreaRect 得到的 OBB
    //   ⚠ 仅用于 Debug 可视化 A/B 对比：不参与任何算法决策
    //   （不写 detections、不影响 tracker 关联/速度/motion_state、不进 UDP）
    //   数据来源：GridCluster.img_corners（见 ElevationMapGroundFilter::ComputeRawImageOBB）
    //   必须与下方【拷贝构造 / 拷贝赋值】同步，否则 outTracks 副本会丢字段
    //==================
    Point2D new_corners[4] = {};        // 左下→右下→右上→左上，C0→C1 = 长轴
    bool    has_new_corners = false;    // 实验 OBB 是否有效

    //==================
    // DrawMapAndAllOverlay函数可视化用，有些(may丢失）,需要根据当前帧定位更新位置
    //==================
    bool current_exist = true;

    // ========================================================================
    // Local Missing Target Recheck（第一版，STATIC only）
    //   ⚠ 以下字段必须在【拷贝构造 / 拷贝赋值】中同步拷贝（见下方手写实现），
    //     否则 outTracks = m_tracker.vtrackings 的副本会丢字段
    // ========================================================================
    DetectionSource detection_source = DetectionSource::NORMAL; ///< 本帧观测来源（NORMAL/RECOVERED）

    /// 最近一次【正常 Cluster detection】的占用 cell 数（Recheck 的稳定性参考基准）。
    /// -1 = 尚无正常 detection 记录。
    /// ⚠ 只能由正常 detection 更新：RECOVERED 帧【不】更新它，
    ///   否则参考基准会被 Recheck 自己的结果逐帧带偏（自我强化漂移）。
    int normal_occupied_cells = -1;

    // ---- Recheck 诊断快照（仅在 Recheck 成功的帧写入；纯诊断，不参与任何算法决策）----
    int   recheck_evidence_points   = 0;    ///< 3×3 ROI 内异常于局部地面的 raw 点数
    int   recheck_current_cells     = 0;    ///< 3×3 ROI 内被上述点占用的 cell 数
    int   recheck_ring_ground_cells = 0;    ///< ring 中可靠 ground cell 数
    float recheck_local_ground_z    = 0.0f; ///< ring ground cell 的 min_z 中位数
    int   recheck_cell_diff         = 0;    ///< current_cells - normal_occupied_cells

    // 新增：指向卡尔曼滤波器的智能指针
    std::unique_ptr<KalmanFilter2D> kf;

    // 新增：构造函数，用于初始化卡尔曼滤波器
    TrackedObstacle(float x, float y, float vx, float vy) : pos_x(x), pos_y(y), vx(vx), vy(vy) {
        kf = std::make_unique<KalmanFilter2D>(x, y);
    }

    // 必须提供一个默认构造函数，因为 std::vector 等容器需要它
    TrackedObstacle() : pos_x(0), pos_y(0), vx(0), vy(0) {}

    // --- 新增：拷贝构造函数 ---
    TrackedObstacle(const TrackedObstacle& other) {
        // 1. 拷贝所有基本数据类型成员
        cluster_id = other.cluster_id;
        id = other.id;
        pos_x = other.pos_x;
        pos_y = other.pos_y;
        pos_z = other.pos_z;
        depth = other.depth;
        width = other.width;
        height = other.height;
        age = other.age;
        lastSeen = other.lastSeen;
        // ⚠ current_exist 必须同步拷贝，否则 outTracks = m_tracker.vtrackings 的副本里
        //   恒为默认值 true → DebugViewer 的 miss 分支永不执行
        current_exist = other.current_exist;
        // Local Missing Target Recheck：来源标记 + 诊断字段（漏拷 → 输出副本丢 RECOVERED 标记）
        detection_source        = other.detection_source;
        normal_occupied_cells   = other.normal_occupied_cells;
        recheck_evidence_points = other.recheck_evidence_points;
        recheck_current_cells   = other.recheck_current_cells;
        recheck_ring_ground_cells = other.recheck_ring_ground_cells;
        recheck_local_ground_z  = other.recheck_local_ground_z;
        recheck_cell_diff       = other.recheck_cell_diff;
        for(int i = 0; i < 3; i++) Translation[i] = other.Translation[i];
        for(int i = 0; i < 9; i++) Rotation[i] = other.Rotation[i];
        for(int i = 0; i < 4; i++) corners[i] = other.corners[i];

        vx = other.vx;
        vy = other.vy;

        // Phase 3-A: Motion State 字段必须同步拷贝
        motion_state      = other.motion_state;
        has_map_pos       = other.has_map_pos;
        map_x             = other.map_x;
        map_y             = other.map_y;
        map_yaw_deg       = other.map_yaw_deg;
        has_map_yaw       = other.has_map_yaw;
        map_vx            = other.map_vx;
        map_vy            = other.map_vy;
        motion_step       = other.motion_step;
        motion_net        = other.motion_net;
        motion_dir_deg    = other.motion_dir_deg;
        motion_static_run = other.motion_static_run;
        motion_moving_run = other.motion_moving_run;
        recent_map_positions = other.recent_map_positions;

        // Phase 3-B': 1cm Raw Point Image 实验字段必须同步拷贝
        has_new_corners = other.has_new_corners;
        for (int i = 0; i < 4; i++) new_corners[i] = other.new_corners[i];

        // 2. 深拷贝 unique_ptr 成员
        if (other.kf) {
            // 创建一个新的 KalmanFilter2D，并用 other.kf 的当前状态初始化它
            // 这要求 KalmanFilter2D 有一个可以接受初始状态的构造函数，或者我们手动设置其状态
            kf = std::make_unique<KalmanFilter2D>(other.kf->getPosX(), other.kf->getPosY());
            // 手动同步状态向量的其余部分（速度）和协方差，以确保完全复制
            kf->state = other.kf->state;
            kf->P = other.kf->P;
            // 如果需要，也可以复制 Q 和 R，但通常它们是固定的参数
        }
    }

    // --- 新增：拷贝赋值运算符 ---
    TrackedObstacle& operator=(const TrackedObstacle& other) {
        // 1. 防止自我赋值 (a = a)
        if (this == &other) {
            return *this;
        }

        // 2. 拷贝所有基本数据类型成员
        cluster_id = other.cluster_id;
        id = other.id;
        pos_x = other.pos_x;
        pos_y = other.pos_y;
        pos_z = other.pos_z;
        depth = other.depth;
        width = other.width;
        height = other.height;
        age = other.age;
        lastSeen = other.lastSeen;
        // ⚠ current_exist 必须同步拷贝，否则 outTracks = m_tracker.vtrackings 的副本里
        //   恒为默认值 true → DebugViewer 的 miss 分支永不执行
        current_exist = other.current_exist;
        // Local Missing Target Recheck：来源标记 + 诊断字段（漏拷 → 输出副本丢 RECOVERED 标记）
        detection_source        = other.detection_source;
        normal_occupied_cells   = other.normal_occupied_cells;
        recheck_evidence_points = other.recheck_evidence_points;
        recheck_current_cells   = other.recheck_current_cells;
        recheck_ring_ground_cells = other.recheck_ring_ground_cells;
        recheck_local_ground_z  = other.recheck_local_ground_z;
        recheck_cell_diff       = other.recheck_cell_diff;
        for(int i = 0; i < 3; i++) Translation[i] = other.Translation[i];
        for(int i = 0; i < 9; i++) Rotation[i] = other.Rotation[i];
        for(int i = 0; i < 4; i++) corners[i] = other.corners[i];

        vx = other.vx;
        vy = other.vy;

        // Phase 3-A: Motion State 字段必须同步拷贝
        motion_state      = other.motion_state;
        has_map_pos       = other.has_map_pos;
        map_x             = other.map_x;
        map_y             = other.map_y;
        map_yaw_deg       = other.map_yaw_deg;
        has_map_yaw       = other.has_map_yaw;
        map_vx            = other.map_vx;
        map_vy            = other.map_vy;
        motion_step       = other.motion_step;
        motion_net        = other.motion_net;
        motion_dir_deg    = other.motion_dir_deg;
        motion_static_run = other.motion_static_run;
        motion_moving_run = other.motion_moving_run;
        recent_map_positions = other.recent_map_positions;

        // Phase 3-B': 1cm Raw Point Image 实验字段必须同步拷贝
        has_new_corners = other.has_new_corners;
        for (int i = 0; i < 4; i++) new_corners[i] = other.new_corners[i];

        // 3. 深拷贝 unique_ptr 成员
        if (other.kf) {
            // 如果目标对象已有 kf，先释放它（unique_ptr 会自动处理）
            // 然后创建一个新的 KalmanFilter2D 并复制状态
            kf = std::make_unique<KalmanFilter2D>(other.kf->getPosX(), other.kf->getPosY());
            kf->state = other.kf->state;
            kf->P = other.kf->P;
        } else {
            // 如果源对象的 kf 为空，则重置目标对象的 kf
            kf.reset();
        }

        return *this;
    }


};

extern uint64_t nextTrackID;

class SimpleTracker {
public:
    std::vector<TrackedObstacle> vtrackings; // 存储所有正在跟踪的目标
    // int next_id = 9999;             // 下一个可用的ID
    float match_threshold = 0.5f;//0.3f;      // 匹配阈值，单位：米。可以根据实际情况调整

    // 更新跟踪列表
    // detections: 当前帧从相机获取的障碍物列表（未带ID）
    void update(const std::vector<TrackedObstacle>& detections,  const unsigned long long &time);

    void update_V2(const std::vector<TrackedObstacle>& detections, const unsigned long long &time);

    void hungarianAssignment(
        const std::vector<std::vector<float>>& cost_matrix,
        std::vector<int>& track_to_detection
    );

private:
    void removeLongLostTargets();
};




unsigned long long GetCurrentTimestamp();
std::string timestampToTimeString(unsigned long long timestamp_in_milliseconds);

}

#endif
