#include "debug_frame_queue.h"

namespace Lidar_Low_Detection
{

std::deque<std::shared_ptr<DebugFrame>> g_debug_frame_queue;
std::mutex                              g_debug_frame_mutex;
std::condition_variable                 g_debug_frame_cv;
std::atomic<bool>                       g_debug_frame_stopped{false};

// ============================================================================
// PushDebugFrame — 生产者（算法线程调用）
// ============================================================================
void PushDebugFrame(std::shared_ptr<DebugFrame> frame)
{
    if (!frame)
    {
        return;
    }
    if (g_debug_frame_stopped.load(std::memory_order_acquire))
    {
        return;  // 已停止，不再入队
    }

    {
        std::lock_guard<std::mutex> lock(g_debug_frame_mutex);

        // 有界 + latest-frame 语义：
        //   push_back 最新帧；超过容量时 pop_front 丢弃最旧帧。
        //   pop_front() 只用于丢弃旧帧，绝不是消费者正常取帧操作。
        g_debug_frame_queue.push_back(std::move(frame));
        if (g_debug_frame_queue.size() > MAX_DEBUG_FRAME_CACHE_SIZE)
        {
            g_debug_frame_queue.pop_front();
        }
    }

    // 唤醒消费者（cv.wait 中的 FoxglovePublisherThread）
    g_debug_frame_cv.notify_one();
}

// ============================================================================
// PeekLatestDebugFrame — 消费者（FoxglovePublisherThread 调用）
// ============================================================================
std::shared_ptr<DebugFrame> PeekLatestDebugFrame()
{
    std::lock_guard<std::mutex> lock(g_debug_frame_mutex);

    if (g_debug_frame_queue.empty())
    {
        return nullptr;
    }

    // back() = 当前最新 Frame。复制 shared_ptr（引用计数 +1），不 pop。
    // 解锁后调用方持有的 shared_ptr 使该 DebugFrame 保活，
    // Producer 之后 pop_front() 不会影响它（无悬空引用 / use-after-free）。
    return g_debug_frame_queue.back();
}

// ============================================================================
// StopDebugFrameQueue — 停止队列并唤醒消费者线程
// ============================================================================
void StopDebugFrameQueue()
{
    g_debug_frame_stopped.store(true, std::memory_order_release);
    g_debug_frame_cv.notify_all();
}

} // namespace Lidar_Low_Detection
