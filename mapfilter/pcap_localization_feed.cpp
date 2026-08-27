#include "pcap_localization_feed.h"

#include <chrono>
#include <cstdio>
#include <cstring>

#ifndef NO_USE_PCAP
#include <pcap.h>
#endif

#include "localization_manager.h"
#include "struct_typedef.h"

namespace Lidar_Low_Detection
{

namespace
{

constexpr uint16_t kEthTypeIpv4 = 0x0800; ///< 以太网类型：IPv4
constexpr uint16_t kEthTypeVlan = 0x8100; ///< 以太网类型：802.1Q VLAN
constexpr uint8_t  kIpProtoUdp  = 17;     ///< IP 协议号：UDP
constexpr uint8_t  kLocMsgId    = 0x0a;   ///< MC 消息 ID（FUSIONLOC_MSGID）
constexpr uint16_t kLocPort     = 9110;   ///< MC 定位端口（FUSIONLOC_MC_PORT）

/// 解析 以太网(可带VLAN) -> IPv4 -> UDP，返回 UDP 载荷指针/长度 与 目的端口
bool parseUdpPayload(const uint8_t* pkt, size_t pkt_len,
                     const uint8_t** out_payload, size_t* out_payload_len,
                     uint16_t* out_dst_port)
{
    if (pkt_len < 14)
    {
        return false;
    }
    size_t off = 14;
    uint16_t etype = static_cast<uint16_t>(pkt[12]) << 8 | pkt[13];
    if (etype == kEthTypeVlan)
    {
        if (pkt_len < off + 4)
        {
            return false;
        }
        etype = static_cast<uint16_t>(pkt[16]) << 8 | pkt[17];
        off += 4;
    }
    if (etype != kEthTypeIpv4)
    {
        return false;
    }
    if (pkt_len < off + 20)
    {
        return false;
    }
    const size_t ihl = static_cast<size_t>(pkt[off] & 0x0f) * 4;
    if (pkt_len < off + ihl + 8)
    {
        return false;
    }
    if (pkt[off + 9] != kIpProtoUdp)
    {
        return false;
    }
    const size_t udp_off = off + ihl;
    *out_dst_port = static_cast<uint16_t>(pkt[udp_off + 2]) << 8 | pkt[udp_off + 3];
    const uint16_t udp_len = static_cast<uint16_t>(pkt[udp_off + 4]) << 8 |
                             pkt[udp_off + 5];
    if (udp_len < 8)
    {
        return false;
    }
    const size_t payload_len = static_cast<size_t>(udp_len) - 8;
    if (pkt_len < udp_off + 8 + payload_len)
    {
        return false;
    }
    *out_payload     = pkt + udp_off + 8;
    *out_payload_len = payload_len;
    return true;
}

} // namespace

PcapLocalizationFeed::~PcapLocalizationFeed()
{
    stop();
}

bool PcapLocalizationFeed::start(const std::string& pcap_path, double rate)
{
    stop(); // 防止重复 start
    pcap_path_ = pcap_path;
    rate_      = (rate > 0.0) ? rate : 1.0;
    fed_count_ = 0;
    running_   = true;
    try
    {
        thread_ = std::thread(&PcapLocalizationFeed::threadLoop, this);
    }
    catch (...)
    {
        running_ = false;
        return false;
    }
    return true;
}

void PcapLocalizationFeed::stop()
{
    running_ = false;
    if (thread_.joinable())
    {
        thread_.join();
    }
}

uint64_t PcapLocalizationFeed::fedCount() const
{
    return fed_count_;
}

void PcapLocalizationFeed::threadLoop()
{
#ifndef NO_USE_PCAP
    char errbuf[PCAP_ERRBUF_SIZE] = {0};
    pcap_t* pcap = pcap_open_offline(pcap_path_.c_str(), errbuf);
    if (pcap == NULL)
    {
        printf("[Localization] pcap 9110 feed open failed: %s (err=%s)\n",
               pcap_path_.c_str(), errbuf);
        running_ = false;
        return;
    }

    // BPF 过滤：只匹配 9110 端口 UDP 报文（与 rs_driver 相同的离线过滤方式）
    struct bpf_program filter;
    memset(&filter, 0, sizeof(filter));
    if (pcap_compile(pcap, &filter, "udp port 9110", 1, 0xFFFFFFFF) < 0)
    {
        printf("[Localization] pcap 9110 feed BPF compile failed\n");
        pcap_close(pcap);
        running_ = false;
        return;
    }

    // 实时节奏回放：相对首个匹配包的时间戳，按 wall-clock 推进
    const auto t_start = std::chrono::steady_clock::now();
    struct timeval t0 = {0, 0};
    bool have_t0 = false;

    printf("[Localization] pcap 9110 feed started: %s\n", pcap_path_.c_str());

    while (running_.load())
    {
        struct pcap_pkthdr* header = NULL;
        const uint8_t* pkt = NULL;
        int ret = pcap_next_ex(pcap, &header, &pkt);
        if (ret < 0)
        {
            break; // 出错 / 文件尾
        }
        if (ret == 0)
        {
            continue; // 超时（离线一般不发生）
        }

        // 仅处理匹配 9110 的包
        if (pcap_offline_filter(&filter, header, pkt) == 0)
        {
            continue;
        }

        if (!have_t0)
        {
            t0 = header->ts;
            have_t0 = true;
        }

        // 节流：按 pcap 时间戳相对节奏 sleep，使 9110 与雷达回放大致同步
        const double target_s =
            (static_cast<double>(header->ts.tv_sec  - t0.tv_sec)) +
            (static_cast<double>(header->ts.tv_usec - t0.tv_usec)) / 1e6;
        const double elapsed_s =
            std::chrono::duration<double>(
                std::chrono::steady_clock::now() - t_start).count();
        const double wait_s = target_s / rate_ - elapsed_s;
        if (wait_s > 0.0)
        {
            std::this_thread::sleep_for(
                std::chrono::microseconds(static_cast<int64_t>(wait_s * 1e6)));
        }

        // 解析 UDP 载荷
        const uint8_t* payload = NULL;
        size_t payload_len = 0;
        uint16_t dst_port = 0;
        if (!parseUdpPayload(pkt, header->caplen, &payload, &payload_len, &dst_port))
        {
            continue;
        }
        if (dst_port != kLocPort)
        {
            continue;
        }
        if (payload_len < sizeof(STR_MC_HEAD) + sizeof(STR_FUSIONLOC))
        {
            continue;
        }

        // 校验 MC 头：msgId==0x0a 且 wLen >= 结构体大小
        STR_MC_HEAD head;
        std::memcpy(&head, payload, sizeof(STR_MC_HEAD));
        if (head.byMsgId != kLocMsgId || head.wLen < sizeof(STR_FUSIONLOC))
        {
            continue;
        }

        // 剥离 MC 头，把纯 STR_FUSIONLOC 喂给 LocalizationManager
        LocalizationManager::instance().onDataReceived(
            payload + sizeof(STR_MC_HEAD), static_cast<int>(sizeof(STR_FUSIONLOC)));
        ++fed_count_;
    }

    pcap_freecode(&filter);
    pcap_close(pcap);
    printf("[Localization] pcap 9110 feed done (fed=%llu)\n",
           static_cast<unsigned long long>(fed_count_.load()));
    running_ = false;
#else
    printf("[Localization] pcap 9110 feed skipped (NO_USE_PCAP)\n");
    running_ = false;
#endif
}

} // namespace Lidar_Low_Detection
