#include "localization_manager.h"

#include <cstdio>
#include <cstring>
#include <chrono>

// comm 定位结构（MC 0x0a @ 9110）
#include "struct_typedef.h"

namespace Lidar_Low_Detection
{

namespace
{

uint64_t nowWallMs()
{
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(now).count());
}

} // namespace

LocalizationManager& LocalizationManager::instance()
{
    static LocalizationManager s_instance;
    return s_instance;
}

void LocalizationManager::configure(bool enable, uint32_t timeout_ms,
                                    bool debug_enable,
                                    double debug_x, double debug_y,
                                    double debug_heading_deg)
{
    std::lock_guard<std::mutex> lock(mutex_);
    enable_     = enable;
    timeout_ms_ = (timeout_ms == 0) ? 1000 : timeout_ms;
    debug_enable_    = debug_enable;
    debug_x_         = debug_x;
    debug_y_         = debug_y;
    debug_heading_deg_ = debug_heading_deg;
    // 初始状态未打印：首帧（无论 valid/invalid）都会打印一次，便于诊断
    last_valid_logged_  = false;
    status_logged_once_ = false;

    if (enable_)
    {
        printf("[Localization] enabled (timeout=%u ms)", timeout_ms_);
        if (debug_enable_)
        {
            printf(" DEBUG_POSE=(%.3f, %.3f, %.2fdeg)",
                   debug_x_, debug_y_, debug_heading_deg_);
        }
        printf("\n");
        printf("[Localization] initialized\n");
    }
    else
    {
        printf("[Localization] disabled\n");
    }
}

bool LocalizationManager::enabled() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return enable_;
}

void LocalizationManager::onDataReceived(const unsigned char* pData, int len)
{
    if (!enable_)
    {
        return;
    }
    if (NULL == pData || len < static_cast<int>(sizeof(STR_FUSIONLOC)))
    {
        printf("[Localization] callback received but len=%d too short\n", len);
        return;
    }

    const STR_FUSIONLOC* loc = reinterpret_cast<const STR_FUSIONLOC*>(pData);

    std::lock_guard<std::mutex> lock(mutex_);
    if (received_count_ == 0)
    {
        printf("[Localization] callback received (first)\n");
    }
    ++received_count_;

    latest_.x           = static_cast<double>(loc->strPoint3fInMap.fX);
    latest_.y           = static_cast<double>(loc->strPoint3fInMap.fY);
    latest_.heading_deg = static_cast<double>(loc->fHeadingInMap);
    latest_.timestamp_ms = loc->ullTimestampModule;
    latest_.valid       = (loc->ullTimestampModule > 0);
    last_ts_ms_          = loc->ullTimestampModule;
    last_wall_ms_        = nowWallMs();
    has_data_            = true;

    if (latest_.valid)
    {
        last_valid_ = latest_;
    }
}

bool LocalizationManager::getPose(Pose& out)
{
    std::lock_guard<std::mutex> lock(mutex_);

    // 离线调试位姿：始终有效（仅用于 pcap 回放验证 HDMap 过滤链路）
    if (enable_ && debug_enable_)
    {
        out.x           = debug_x_;
        out.y           = debug_y_;
        out.heading_deg = debug_heading_deg_;
        out.timestamp_ms = 0;
        out.valid       = true;
        logStatusChanged(true, debug_x_, debug_y_, debug_heading_deg_);
        return true;
    }

    const uint64_t now_ms = nowWallMs();

    // 帧级有效性：已收到数据 且 新鲜（最近 timeout_ms 内有更新）且 时间戳有效
    out = latest_;
    if (has_data_ && latest_.valid &&
        (now_ms - last_wall_ms_) <= timeout_ms_)
    {
        out.valid = true;
    }
    else
    {
        out.valid = false;
    }

    // 状态变化时打印一次
    logStatusChanged(out.valid, latest_.x, latest_.y, latest_.heading_deg);

    return has_data_;
}

bool LocalizationManager::isValid() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (enable_ && debug_enable_)
    {
        return true;
    }
    if (!has_data_)
    {
        return false;
    }
    const uint64_t now_ms = nowWallMs();
    return latest_.valid && (now_ms - last_wall_ms_) <= timeout_ms_;
}

uint64_t LocalizationManager::lastUpdateAgeMs() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (enable_ && debug_enable_)
    {
        return 0;
    }
    if (!has_data_)
    {
        return UINT64_MAX;
    }
    const uint64_t now_ms = nowWallMs();
    return (now_ms >= last_wall_ms_) ? (now_ms - last_wall_ms_) : 0;
}

LocalizationManager::Pose LocalizationManager::lastValidPose() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (enable_ && debug_enable_)
    {
        Pose p;
        p.x           = debug_x_;
        p.y           = debug_y_;
        p.heading_deg = debug_heading_deg_;
        p.valid       = true;
        return p;
    }
    return last_valid_;
}

uint64_t LocalizationManager::receivedCount() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return received_count_;
}

void LocalizationManager::reset()
{
    std::lock_guard<std::mutex> lock(mutex_);
    has_data_       = false;
    last_ts_ms_     = 0;
    last_wall_ms_   = 0;
    received_count_ = 0;
    latest_         = Pose();
    last_valid_     = Pose();
    last_valid_logged_  = false;
    status_logged_once_ = false;
}

void LocalizationManager::logStatusChanged(bool valid, double x, double y,
                                           double heading_deg) const
{
    if (valid)
    {
        // 每帧都打印有效定位（诊断用），便于与点云帧时间戳对比
        printf("[Localization] valid x=%.3f y=%.3f yaw=%.2f timestamp=%llu\n",
               x, y, heading_deg,
               static_cast<unsigned long long>(latest_.timestamp_ms));
        last_valid_logged_  = true;
        status_logged_once_ = true;
    }
    else
    {
        // // 无效状态仅在首次/跳变时打印，避免刷屏
        // if (!status_logged_once_ || last_valid_logged_)
        // {
        //     printf("[Localization] INVALID (no fresh data)\n");
        // }
        // last_valid_logged_  = false;
        // status_logged_once_ = true;

        // 无效状态打印，每帧都打印
        if (!status_logged_once_ || !last_valid_logged_)
        {
            printf("[Localization] INVALID (no fresh data)\n");
        }
        last_valid_logged_  = false;
        status_logged_once_ = true;
    }
}

} // namespace Lidar_Low_Detection
