#ifndef LOW_DETECTION_LOCALIZATION_MANAGER_H
#define LOW_DETECTION_LOCALIZATION_MANAGER_H

#include <cstdint>
#include <mutex>

namespace Lidar_Low_Detection
{

/**
 * @brief 定位管理器：封装公司内部定位数据源（MC 0x0a @ 9110 -> STR_FUSIONLOC）
 *
 * 数据链路（与项目 A / C 一致，见 Project_C_Localization_Queue_Analysis.md）：
 *   外部融合定位模块
 *     → MC 组播 0x0a @ 9110
 *     → OpenfusionLocMCClient + fusionLoc_set_callback(fusionLoc_set)
 *     → SetFusionLocation（写全局，带 mutex）
 *     → LocalizationManager::onDataReceived()   ← 由 fusionLoc_set 调用
 *     → 算法线程 getPose() 读取最新位姿
 *
 * 有效性判定（降级策略，见迁移文档第 21/27 章）：
 *   - 已收到过定位（timestamp > 0）且最近 timeout_ms 内有新数据 → 有效
 *   - 定位长时间不更新 / 未收到 → isValid()==false，HDMapFilter 自动关闭
 *   - 短时丢失：getPose() 返回 lastValidPose()，由调用方决定是否沿用
 *
 * 线程安全：回调线程（写）与算法线程（读）通过内部互斥锁保护。
 * 定位失败不影响低矮检测主流程（见迁移文档第 27 章）。
 */
class LocalizationManager
{
public:
    /// 车辆位姿（车辆地图坐标 x/y 米，heading 度 北=0 顺时针）
    struct Pose
    {
        double       x           = 0.0;   ///< 车辆地图 X（m）
        double       y           = 0.0;   ///< 车辆地图 Y（m）
        double       heading_deg = 0.0;   ///< 车辆地图航向（度）
        uint64_t     timestamp_ms = 0;    ///< 定位模块时间戳（ms）
        bool         valid       = false; ///< 该帧定位是否有效
    };

    static LocalizationManager& instance();

    /**
     * @brief 配置
     * @param enable      是否启用定位
     * @param timeout_ms  定位新鲜度阈值（超过判为无效）
     * @param debug_enable 离线调试：使用固定位姿（无真实定位时验证 HDMap 过滤链路用）
     * @param debug_x/y/heading_deg 调试位姿（地图坐标，单位米/度）
     */
    void configure(bool enable, uint32_t timeout_ms,
                   bool debug_enable = false,
                   double debug_x = 0.0, double debug_y = 0.0,
                   double debug_heading_deg = 0.0);

    bool enabled() const;

    /// 由定位回调(fusionLoc_set)调用：写入最新定位（内部解析 STR_FUSIONLOC）
    void onDataReceived(const unsigned char* pData, int len);

    /// 读取最新位姿（算法线程，每帧调用）
    /// @return true 表示内部已收到过至少一帧定位（out.valid 为帧级有效性）
    bool getPose(Pose& out);

    /// 是否有效（最近 timeout_ms 内有新定位）
    bool isValid() const;

    /// 最近一次定位更新的墙钟新鲜度(ms)：当前墙钟 - 上次收到定位的墙钟（诊断用）
    uint64_t lastUpdateAgeMs() const;

    /// 最近一次有效位姿（短时丢失时供调用方沿用）
    Pose lastValidPose() const;

    /// 累计收到定位帧数（用于调试）
    uint64_t receivedCount() const;

    /// 重置内部状态（重新开始计数）
    void reset();

private:
    LocalizationManager() = default;
    ~LocalizationManager() = default;
    LocalizationManager(const LocalizationManager&) = delete;
    LocalizationManager& operator=(const LocalizationManager&) = delete;

    void logStatusChanged(bool valid, double x, double y, double heading_deg) const;

    mutable std::mutex mutex_;
    bool         enable_      = false;
    uint32_t     timeout_ms_  = 1000;

    // 离线调试固定位姿（仅用于 pcap 回放验证，无真实定位时）
    bool         debug_enable_    = false;
    double       debug_x_         = 0.0;
    double       debug_y_         = 0.0;
    double       debug_heading_deg_ = 0.0;

    bool         has_data_    = false;   ///< 是否收到过有效定位
    uint64_t     last_ts_ms_  = 0;       ///< 最近一次定位时间戳
    uint64_t     last_wall_ms_= 0;       ///< 最近一次收到定位的墙钟时间(ms)
    uint64_t     received_count_ = 0;
    mutable bool last_valid_logged_ = false;   // 日志缓存（mutable 便于 const 方法打印）
    mutable bool status_logged_once_ = false;  // 是否已打印过初始状态（保证首帧必打印）

    Pose         latest_;                ///< 最新定位（帧级 valid 标记）
    Pose         last_valid_;            ///< 最近一次有效定位
};

} // namespace Lidar_Low_Detection

#endif // LOW_DETECTION_LOCALIZATION_MANAGER_H
