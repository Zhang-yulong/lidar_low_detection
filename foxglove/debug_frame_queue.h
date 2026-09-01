#pragma once
#ifndef _DEBUG_FRAME_QUEUE_H_
#define _DEBUG_FRAME_QUEUE_H_

#include <deque>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <memory>

#include "debug_frame.h"

namespace Lidar_Low_Detection
{

// ============================================================================
// DebugFrameQueue —— 有界、latest-frame oriented（面向最新帧）
// ============================================================================
// Foxglove 是实时调试显示，不是历史数据回放器。因此本队列【不是】传统 FIFO
// 历史回放队列，而是：
//
//   生产者（算法线程）：
//     lock
//       queue.push_back(newest_frame)
//       if (queue.size() > MAX_DEBUG_FRAME_CACHE_SIZE)
//           queue.pop_front();        // 只用于"丢弃最旧帧"，不是消费者取帧行为
//     unlock
//     cv.notify_one()                 // 唤醒消费者
//
//   消费者（FoxglovePublisherThread）：
//     lock
//       cv.wait(有帧 或 stopped)
//       frame = queue.back();         // = 当前最新 Frame（shared_ptr，refcount+1）
//     unlock                          // 解锁后 frame 仍安全（持有独立 shared_ptr 副本）
//     consume/publish(frame)          // 不 pop —— back() 是"查看最新"，不是"取出并删除"
//
// 效果：Queue 始终保留最近 MAX_DEBUG_FRAME_CACHE_SIZE 帧；Publisher 慢时自动
//       丢弃最旧帧，始终显示最新状态，允许跳帧（例如 103 → 105 → 106）。
// ============================================================================

constexpr size_t MAX_DEBUG_FRAME_CACHE_SIZE = 3;

extern std::deque<std::shared_ptr<DebugFrame>> g_debug_frame_queue;
extern std::mutex                              g_debug_frame_mutex;
extern std::condition_variable                 g_debug_frame_cv;
extern std::atomic<bool>                       g_debug_frame_stopped;

/**
 * @brief 生产者：推入一帧调试快照（算法线程调用）
 *
 * 有界 + 丢最旧 + notify_one。
 * stopped 之后直接返回，不再入队。
 */
void PushDebugFrame(std::shared_ptr<DebugFrame> frame);

/**
 * @brief 消费者：读取最新一帧（back()），不 pop
 *
 * 锁内复制 shared_ptr（引用计数 +1），解锁后调用方持有的是独立、安全的
 * DebugFrame 副本，不持有 Queue 内部元素的引用（Producer pop_front 不影响它）。
 *
 * @return 最新 DebugFrame 的 shared_ptr；队列为空返回 nullptr
 */
std::shared_ptr<DebugFrame> PeekLatestDebugFrame();

/**
 * @brief 停止队列：置 stopped=true 并 notify_all，唤醒等待中的消费者线程
 *
 * 用于程序退出路径，配合 thread.join() 与 FoxglovePublisher::Stop()：
 *   1. StopDebugFrameQueue()      -> 线程被唤醒并看到 stopped，退出
 *   2. thread.join()              -> 确认线程退出
 *   3. FoxglovePublisher::Stop()  -> 之后才释放 SDK 资源（线程不再访问 SDK）
 */
void StopDebugFrameQueue();

} // namespace Lidar_Low_Detection

#endif // _DEBUG_FRAME_QUEUE_H_
