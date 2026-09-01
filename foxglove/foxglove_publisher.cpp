/**
 * @file    foxglove_publisher.cpp
 * @brief   FoxglovePublisher 实现 — 基于 Foxglove 官方 C SDK
 *
 * 复用相机项目（mrdvs_camera_newsdk_26year_callback_v2.1/foxglove/foxglove_publisher.cpp）
 * 已验证的 SDK 调用模式：
 *   foxglove_context_new / foxglove_server_start / foxglove_channel_create_point_cloud
 *   / foxglove_channel_log_point_cloud / foxglove_server_stop / foxglove_context_free
 *
 * 禁止自实现 WebSocket / SHA1 / Base64 / Protobuf —— 全部由 SDK 完成。
 */

#include "foxglove_publisher.h"
#include "debug_frame_queue.h"

#include <cstdio>
#include <cstring>
#include <cstddef>   // offsetof

namespace Lidar_Low_Detection
{

// =============================================================================
// 辅助: 错误信息打印
// =============================================================================
static void LogError(const char* func, foxglove_error err)
{
    fprintf(stderr, "[FoxglovePublisher] %s failed, foxglove_error=%d\n",
            func, static_cast<int>(err));
}

// =============================================================================
// FoxglovePublisher 构造 / 析构
// =============================================================================

FoxglovePublisher::FoxglovePublisher()
    : context_(nullptr)
    , server_(nullptr)
    , channel_(nullptr)
    , frame_id_("base_link")     // 点云所在坐标系：车体系
    , port_(0)
    , server_running_(false)
    , channel_created_(false)
{
    // 预构建 4 个字段描述符（只需做一次）
    BuildFieldDescriptors();
}

FoxglovePublisher::~FoxglovePublisher()
{
    Stop();
}

// =============================================================================
// Init — 创建 SDK Context
// =============================================================================

bool FoxglovePublisher::Init(const std::string& host, uint16_t port, const std::string& name)
{
    listen_host_ = host;
    port_        = port;
    server_name_ = name;

    context_ = foxglove_context_new();
    if (context_ == nullptr)
    {
        fprintf(stderr, "[FoxglovePublisher] foxglove_context_new() returned NULL\n");
        return false;
    }

    printf("[FoxglovePublisher] Init OK: context=%p\n", static_cast<const void*>(context_));
    return true;
}

// =============================================================================
// StartServer — 启动 WebSocket Server
// =============================================================================

bool FoxglovePublisher::StartServer()
{
    if (context_ == nullptr)
    {
        fprintf(stderr, "[FoxglovePublisher] StartServer: context is NULL, call Init() first\n");
        return false;
    }
    if (server_running_.load(std::memory_order_acquire))
    {
        fprintf(stderr, "[FoxglovePublisher] StartServer: server already running\n");
        return true;  // 幂等
    }

    // 构建 foxglove_string（data 指针须在 options 生命周期内有效）
    foxglove_string f_name = { server_name_.data(), server_name_.size() };
    foxglove_string f_host = { listen_host_.data(), listen_host_.size() };

    foxglove_server_options options = {};
    options.context = context_;
    options.name    = f_name;
    options.host    = f_host;
    options.port    = port_;
    // callbacks = nullptr, capabilities = 0（纯发布模式）, server_info = nullptr 等默认即可

    foxglove_error err = foxglove_server_start(&options, &server_);
    if (err != FOXGLOVE_ERROR_OK)
    {
        LogError("foxglove_server_start", err);
        return false;
    }

    uint16_t actual_port = foxglove_server_get_port(server_);
    printf("[FoxglovePublisher] WebSocket Server started on ws://%s:%u\n",
           listen_host_.c_str(), static_cast<unsigned>(actual_port));

    server_running_.store(true, std::memory_order_release);
    return true;
}

// =============================================================================
// CreatePointCloudChannel — 创建点云 Channel
// =============================================================================

bool FoxglovePublisher::CreatePointCloudChannel(const std::string& topic)
{
    if (!server_running_.load(std::memory_order_acquire))
    {
        fprintf(stderr, "[FoxglovePublisher] CreatePointCloudChannel: server not running\n");
        return false;
    }

    topic_ = topic;  // 保存, 确保 foxglove_string.data 有效

    foxglove_string f_topic = { topic_.data(), topic_.size() };

    foxglove_error err = foxglove_channel_create_point_cloud(f_topic, context_, &channel_);
    if (err != FOXGLOVE_ERROR_OK)
    {
        LogError("foxglove_channel_create_point_cloud", err);
        return false;
    }

    channel_created_.store(true, std::memory_order_release);
    printf("[FoxglovePublisher] PointCloud channel created: topic=\"%s\"\n", topic_.c_str());
    return true;
}

// =============================================================================
// PublishPointCloud — 发布一帧点云
// =============================================================================

bool FoxglovePublisher::PublishPointCloud(const pcl::PointCloud<pcl::PointXYZI>& cloud,
                                          uint64_t timestamp_us)
{
    if (!channel_created_.load(std::memory_order_acquire))
    {
        return false;  // channel 未创建或 server 未启动
    }
    if (cloud.empty())
    {
        return false;
    }

    // ---- Step 1: 构建 foxglove_timestamp (μs → {sec, nsec}) ----
    foxglove_timestamp f_ts = MakeTimestamp(timestamp_us);

    // ---- Step 2: 构建 foxglove_point_cloud ----
    // pcl::PointXYZI 内存布局（含 16 字节对齐，PCL 1.8 下通常为 32 字节）：
    //   x@0, y@4, z@8, [padding@12], intensity@16
    // 字段偏移用 offsetof 动态计算，不写死，兼容不同 PCL 版本布局。
    foxglove_point_cloud pcd = {};
    pcd.timestamp    = &f_ts;
    pcd.frame_id     = { frame_id_.data(), frame_id_.size() };   // foxglove_string
    pcd.pose         = nullptr;                                   // 无位姿（车体系）
    pcd.point_stride = static_cast<uint32_t>(sizeof(pcl::PointXYZI));
    pcd.fields       = fields_.data();
    pcd.fields_count = fields_.size();
    pcd.data         = reinterpret_cast<const unsigned char*>(cloud.points.data());
    pcd.data_len     = cloud.points.size() * sizeof(pcl::PointXYZI);

    // ---- Step 3: log_time — 转换为纳秒 ----
    uint64_t log_time_ns = timestamp_us * 1000ULL;

    // ---- Step 4: 调用 SDK 发布 ----
    foxglove_error err = foxglove_channel_log_point_cloud(
        channel_,
        &pcd,
        &log_time_ns,
        0  // sink_id=0 表示发布到所有连接的 sink（WebSocket clients）
    );

    if (err != FOXGLOVE_ERROR_OK)
    {
        LogError("foxglove_channel_log_point_cloud", err);
        return false;
    }
    return true;
}

// =============================================================================
// Stop — 停止 Server 并释放资源
// =============================================================================

void FoxglovePublisher::Stop()
{
    // 1. 置标志位（消费者线程据此退出）
    server_running_.store(false, std::memory_order_release);
    channel_created_.store(false, std::memory_order_release);

    // 2. 停止 Server（内部关闭 WebSocket 连接、释放 IO 线程）
    if (server_ != nullptr)
    {
        foxglove_error err = foxglove_server_stop(server_);
        if (err != FOXGLOVE_ERROR_OK)
        {
            LogError("foxglove_server_stop", err);
        }
        server_ = nullptr;
        printf("[FoxglovePublisher] WebSocket Server stopped\n");
    }

    // 3. channel_ 由 context 管理, 无需单独释放（SDK 在 context_free 时处理）

    // 4. 释放 Context
    if (context_ != nullptr)
    {
        foxglove_context_free(context_);
        context_ = nullptr;
        printf("[FoxglovePublisher] Context freed\n");
    }

    printf("[FoxglovePublisher] Stopped\n");
}

// =============================================================================
// GetClientCount / GetPort
// =============================================================================

size_t FoxglovePublisher::GetClientCount() const
{
    if (server_ == nullptr) return 0;
    return foxglove_server_get_client_count(server_);
}

uint16_t FoxglovePublisher::GetPort() const
{
    if (server_ == nullptr) return 0;
    return foxglove_server_get_port(server_);
}

// =============================================================================
// 内部辅助函数
// =============================================================================

foxglove_timestamp FoxglovePublisher::MakeTimestamp(uint64_t timestamp_us)
{
    foxglove_timestamp ts;
    ts.sec  = static_cast<uint32_t>(timestamp_us / 1000000ULL);
    ts.nsec = static_cast<uint32_t>((timestamp_us % 1000000ULL) * 1000ULL);
    return ts;
}

void FoxglovePublisher::BuildFieldDescriptors()
{
    // pcl::PointXYZI 字段偏移：用 offsetof 动态计算，兼容不同 PCL 版本布局
    //   x@0, y@4, z@8, [padding@12], intensity@16 （PCL 1.8 下通常如此）
    field_names_ = { "x", "y", "z", "intensity" };

    const uint32_t offsets[] = {
        static_cast<uint32_t>(offsetof(pcl::PointXYZI, x)),
        static_cast<uint32_t>(offsetof(pcl::PointXYZI, y)),
        static_cast<uint32_t>(offsetof(pcl::PointXYZI, z)),
        static_cast<uint32_t>(offsetof(pcl::PointXYZI, intensity)),
    };
    const foxglove_numeric_type types[] = {
        FOXGLOVE_NUMERIC_TYPE_FLOAT32,  // x
        FOXGLOVE_NUMERIC_TYPE_FLOAT32,  // y
        FOXGLOVE_NUMERIC_TYPE_FLOAT32,  // z
        FOXGLOVE_NUMERIC_TYPE_FLOAT32,  // intensity
    };

    for (size_t i = 0; i < field_names_.size(); ++i)
    {
        fields_[i].name   = { field_names_[i].data(), field_names_[i].size() };
        fields_[i].offset = offsets[i];
        fields_[i].type   = types[i];
    }
}

// =============================================================================
// FoxglovePublisherThread — 消费者线程
// Phase 2：cv.wait 阻塞等待 DebugFrameQueue → back() 获取最新帧（不 pop）→ 最小日志。
//          本阶段不发布任何 Foxglove 数据；Phase 3/4/5 将在锁外发布。
// =============================================================================

void FoxglovePublisherThread(FoxglovePublisher* publisher)
{
    if (publisher == nullptr)
    {
        fprintf(stderr, "[FoxglovePublisherThread] publisher is NULL, thread exit\n");
        return;
    }

    printf("[FoxglovePublisherThread] Started\n");

    while (true)
    {
        std::shared_ptr<DebugFrame> frame;
        {
            // 锁只保护 Queue 数据访问；无新帧时阻塞等待（不再是 10ms 轮询）
            std::unique_lock<std::mutex> lock(g_debug_frame_mutex);
            g_debug_frame_cv.wait(lock, [] {
                return g_debug_frame_stopped.load(std::memory_order_acquire) ||
                       !g_debug_frame_queue.empty();
            });

            if (g_debug_frame_stopped.load(std::memory_order_acquire))
            {
                break;  // 程序退出
            }

            // back() = 当前最新 Frame；锁内复制 shared_ptr（refcount+1），不 pop。
            // 解锁后 frame 持有独立、安全的 DebugFrame 副本，Producer pop_front 不影响它。
            frame = g_debug_frame_queue.back();
        }   // 解锁：Publish 必须在锁外执行（Phase 3 起）

        // ── Phase 2：最小日志验证握手；本阶段不发布任何 Foxglove 数据 ──
        if (frame)
        {
            printf("[Foxglove] DebugFrame consumed, timestamp=%llu, "
                   "clusters=%zu, trackings=%zu\n",
                   frame->timestamp_ms,
                   frame->clusters.size(),
                   frame->trackings.size());
        }
    }

    printf("[FoxglovePublisherThread] Exited\n");
}

} // namespace Lidar_Low_Detection
