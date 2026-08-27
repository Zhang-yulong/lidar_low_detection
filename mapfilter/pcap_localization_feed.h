#ifndef LOW_DETECTION_PCAP_LOCALIZATION_FEED_H
#define LOW_DETECTION_PCAP_LOCALIZATION_FEED_H

#include <atomic>
#include <string>
#include <thread>

namespace Lidar_Low_Detection
{

/**
 * @brief pcap 定位喂入器（仅 pcap 回放模式使用）
 *
 * 背景：rs_driver 的 pcap 回放（InputPcapJumbo）只在进程内离线读取雷达报文
 * （msop/difop 端口），从不把数据包注入内核网络栈。因此 OpenfusionLocMCClient
 * 的真实 UDP 组播 socket（MC 0x0a @ 9110）在 pcap 模式下收不到任何定位数据，
 * HDMap 过滤会因 localization invalid 自动关闭。
 *
 * 本类用独立线程解析与雷达回放相同的 pcap 文件中的 9110 报文：
 *   报文 = STR_MC_HEAD(8B) + STR_FUSIONLOC(120B) [+ 1B 尾部校验]
 * 剥离 STR_MC_HEAD 后调用 LocalizationManager::onDataReceived()，并按 pcap 包
 * 时间戳做实时节奏回放，保证算法帧能拿到"新鲜"定位（timeout_ms 内）。
 *
 * 注意：仅 pcapRunningModel=1 时启用；onlineModel 下 9110 由真实网络组播经
 * OpenfusionLocMCClient 送入，无需（也不应）启用本模块。
 */
class PcapLocalizationFeed
{
public:
    PcapLocalizationFeed() = default;
    ~PcapLocalizationFeed(); // 自动 stop()

    PcapLocalizationFeed(const PcapLocalizationFeed&) = delete;
    PcapLocalizationFeed& operator=(const PcapLocalizationFeed&) = delete;

    /// 启动后台线程读取 pcap 中 9110 报文并喂给 LocalizationManager
    /// @param pcap_path 与 rs_driver 回放相同的 pcap 文件
    /// @param rate      回放倍率（1.0=按真实采集速度）
    /// @return true=成功打开 pcap 并已启动线程
    bool start(const std::string& pcap_path, double rate = 1.0);

    /// 停止并回收线程（可重复调用；析构自动调用）
    void stop();

    /// 已喂入的定位帧数（诊断用）
    uint64_t fedCount() const;

private:
    void threadLoop();

    std::string           pcap_path_;
    double                rate_     = 1.0;
    std::atomic<bool>     running_{false};
    std::thread           thread_;
    std::atomic<uint64_t> fed_count_{0};
};

} // namespace Lidar_Low_Detection

#endif // LOW_DETECTION_PCAP_LOCALIZATION_FEED_H
