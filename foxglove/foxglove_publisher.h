#pragma once
#ifndef _FOXGLOVE_PUBLISHER_H_
#define _FOXGLOVE_PUBLISHER_H_

/**
 * @file    foxglove_publisher.h
 * @brief   FoxglovePublisher — 基于 Foxglove 官方 C SDK 的发布器（低矮障碍物检测项目）
 *
 * 复用相机项目（mrdvs_camera_newsdk_26year_callback_v2.1/foxglove/）已验证的
 * Foxglove 发布机制，并适配低矮检测项目：
 *   - 点云通道直接发布项目已有的 pcl::PointCloud<pcl::PointXYZI>
 *     （4 字段 x/y/z/intensity，point_stride = sizeof(pcl::PointXYZI)，零转换）
 *   - 不引入相机项目的 CompactPoint / PointCloudMsg / FrameMatcher 架构
 *   - 本阶段（Phase 1）只建立 Publisher 基础设施，不接收任何算法数据
 *
 * 生命周期:
 *   1. FoxglovePublisher 构造
 *   2. Init(host, port, name)       — 创建 SDK context
 *   3. StartServer()                — 启动 WebSocket Server（默认 8765）
 *   4. CreatePointCloudChannel()    — 创建 /low_obstacle/pointcloud 通道
 *   5. PublishPointCloud(cloud, ts) — 发布点云（后续 Phase 3 由消费者线程调用）
 *   6. Stop()                       — 停止 Server，释放 channel/context
 *
 * 线程模型:
 *   - 生产: 算法线程 → (Phase 2: DebugFrame) → (Phase 2: DebugFrameQueue)
 *   - 消费: FoxglovePublisherThread() 读取最新帧并发布
 *   - Phase 1 中 Publisher 仅被 FoxglovePublisherThread 使用；
 *     SDK 发送只发生在消费者线程，绝不阻塞算法线程。
 */

#include <string>
#include <array>
#include <atomic>
#include <cstdint>

#include "foxglove-c.h"

// 点云类型：直接使用项目已有的 PCL 点云（Type.h 中 typedef PointCloud2Intensity）
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace Lidar_Low_Detection
{

class FoxglovePublisher
{
public:
    FoxglovePublisher();
    ~FoxglovePublisher();

    // 禁止拷贝
    FoxglovePublisher(const FoxglovePublisher&) = delete;
    FoxglovePublisher& operator=(const FoxglovePublisher&) = delete;

    // =========================================================================
    // 核心接口
    // =========================================================================

    /**
     * @brief 创建 Foxglove SDK context
     * @param host  监听地址（如 "0.0.0.0"）
     * @param port  WebSocket 服务端口（如 8765）
     * @param name  服务器名称（如 "low_detection"）
     */
    bool Init(const std::string& host, uint16_t port, const std::string& name);

    /**
     * @brief 启动 WebSocket Server
     * @note  启动后 Foxglove Studio 可连接 ws://<host>:<port>
     */
    bool StartServer();

    /**
     * @brief 创建点云 Channel
     * @param topic 通道名（如 "/low_obstacle/pointcloud"）
     * @note  SDK 自动完成 Schema 注册 / Advertise / Channel 管理
     */
    bool CreatePointCloudChannel(const std::string& topic);

    /**
     * @brief 发布一帧点云（线程安全；后续仅由 FoxglovePublisherThread 调用）
     * @param cloud        滤波后点云（pcl::PointCloud<pcl::PointXYZI>）
     * @param timestamp_us 帧时间戳（微秒），用于 Foxglove log_time / 时间轴
     *
     * 内部构建 foxglove_point_cloud 并调用 foxglove_channel_log_point_cloud。
     * pcl::PointXYZI 内存布局（含 16 字节对齐填充）直接作为 point_stride，
     * 无需逐点转换。
     */
    bool PublishPointCloud(const pcl::PointCloud<pcl::PointXYZI>& cloud, uint64_t timestamp_us);

    /**
     * @brief 停止 WebSocket Server 并释放资源
     */
    void Stop();

    // =========================================================================
    // 状态查询
    // =========================================================================
    bool   IsRunning()      const { return server_running_.load(std::memory_order_acquire); }
    bool   IsChannelReady() const { return channel_created_.load(std::memory_order_acquire); }
    size_t GetClientCount() const;
    uint16_t GetPort() const;

private:
    // =========================================================================
    // 内部辅助
    // =========================================================================

    /** @brief 时间戳(μs) → foxglove_timestamp {sec, nsec} */
    static foxglove_timestamp MakeTimestamp(uint64_t timestamp_us);

    /** @brief 构建 foxglove_packed_element_field[4] (x,y,z,intensity) */
    void BuildFieldDescriptors();

    // =========================================================================
    // SDK 对象（生命周期: Init → Stop）
    // =========================================================================
    const struct foxglove_context*    context_;   // foxglove_context_new()
    struct foxglove_websocket_server* server_;    // foxglove_server_start()
    const struct foxglove_channel*    channel_;   // foxglove_channel_create_point_cloud()

    // =========================================================================
    // 字符串存储 — foxglove_string.data 指向此处, 必须比 foxglove_string 长寿
    // =========================================================================
    std::string name_;
    std::string host_;
    std::string topic_;
    std::string frame_id_;

    /** @brief 4 个字段名: "x","y","z","intensity" */
    std::array<std::string, 4> field_names_;

    /** @brief 预构建的字段描述符数组, 指针指向 field_names_ */
    std::array<struct foxglove_packed_element_field, 4> fields_;

    // =========================================================================
    // 服务端配置
    // =========================================================================
    std::string server_name_;
    uint16_t    port_;
    std::string listen_host_;

    // =========================================================================
    // 状态
    // =========================================================================
    std::atomic<bool> server_running_;
    std::atomic<bool> channel_created_;
};

// ============================================================================
// 未来 DebugFrameQueue 语义（Phase 2 实现，此处先明确接口约定）
// ============================================================================
// Foxglove 是实时调试显示，不是历史数据回放器，因此队列采用 latest-frame
// oriented（面向最新帧）而非 strict FIFO historical playback（严格逐帧补发）：
//
//   生产者（算法线程）：
//     lock
//     queue.push_back(newest_frame)
//     if (queue.size() >= MAX) queue.pop_front();   // 超容量丢最旧帧
//     unlock; cv.notify_one()
//
//   消费者（FoxglovePublisherThread）：
//     lock; cv.wait(有帧 或 退出)
//     frame = queue.back();     // 永远获取“最新一帧”
//     unlock                     // 取到 shared_ptr 后必须释放锁再发布
//     publish(frame)            // 不 pop —— back() 是“查看”而非“取出删除”
//
// 效果：Publisher 慢时队列只保留最近若干帧，Foxglove 始终显示最新帧（103→105→106），
//       不会逐帧补发（103→104→105→106）。
// ============================================================================

/**
 * @brief Foxglove 发布消费者线程函数
 * @param publisher 已 Init + StartServer + CreatePointCloudChannel 的 FoxglovePublisher
 *
 * Phase 1：无 DebugFrame 数据，线程仅轮询保活，等待 publisher->IsRunning()==false 退出。
 *         保证 Publisher/Server 存活，且不阻塞算法线程。
 *
 * Phase 2 接入 DebugFrameQueue 后改为：
 *   1. wait on DebugFrameQueue 的 condition_variable
 *   2. 读取 queue.back()（最新帧，不 pop，见上方语义约定）
 *   3. 发布该帧的点云 / Cluster / Tracking
 *   4. 退出标志置位时退出
 */
void FoxglovePublisherThread(FoxglovePublisher* publisher);

} // namespace Lidar_Low_Detection

#endif // _FOXGLOVE_PUBLISHER_H_
