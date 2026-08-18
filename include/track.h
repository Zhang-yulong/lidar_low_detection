#pragma once
#ifndef _TRACK_H_
#define _TRACK_H_


#include <cmath>
#include <algorithm>
#include <sys/time.h>
#include <atomic>
#include <chrono>

#include <ctime> // 必须包含，提供 time_t 和 struct tm

#include "opencv2/opencv.hpp" 

#include "ulog_api.h"
#include "Type.h"
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
        for(int i = 0; i < 3; i++) Translation[i] = other.Translation[i];
        for(int i = 0; i < 9; i++) Rotation[i] = other.Rotation[i];
        for(int i = 0; i < 4; i++) corners[i] = other.corners[i];

        vx = other.vx;
        vy = other.vy;

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
        for(int i = 0; i < 3; i++) Translation[i] = other.Translation[i];
        for(int i = 0; i < 9; i++) Rotation[i] = other.Rotation[i];
        for(int i = 0; i < 4; i++) corners[i] = other.corners[i];

        vx = other.vx;
        vy = other.vy;

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
    void removeLostTargets();
};




unsigned long long GetCurrentTimestamp();
std::string timestampToTimeString(unsigned long long timestamp_in_milliseconds);

}

#endif
