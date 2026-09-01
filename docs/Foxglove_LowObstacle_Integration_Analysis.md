# Foxglove 低矮障碍物检测集成分析（第一阶段）

> **阶段**：第一阶段 —— 仅阅读与分析，**未修改任何 `.h` / `.cpp` / `CMakeLists.txt` / `Makefile`**。
>
> **目标**：复用相机项目（`mrdvs_camera_newsdk_26year_callback_v2.1`）已经验证的 Foxglove **发布机制**，加上低矮检测项目**已有的数据结构**，在一帧算法处理完成的位置生成最小 `DebugFrame`，交给独立 `FoxglovePublisherThread` 消费。
>
> **核心原则**：
> - 不重构低矮检测算法（ElevationMapGroundFilter / Cluster / HDMap Filter / Tracker）。
> - 不搬相机项目的 Callback / PointCloudMsg / ObstacleMsg / FrameMatcher 架构。
> - 不新增 FrameMatcher / depth_frame_id / app_frame_id 等相机项目专属字段。
> - DebugFrame 只保存「这一轮低矮检测算法处理完成后的调试快照」。

---

## 0. 结论速览（TL;DR）

| 问题 | 结论 |
|---|---|
| `pFilteredPointCloud` 类型 | `PointCloud2Intensity::Ptr` = `std::shared_ptr<pcl::PointCloud<pcl::PointXYZI>>`（`Type.h` 别名） |
| `outputClusters` 类型 | `std::vector<GridCluster>`（`GridCluster` 定义于 `ElevationMapGroundFilter.h:258`） |
| `m_tracker.vtrackings` 类型 | `std::vector<TrackedObstacle>`（`track.h:159`；`TrackedObstacle` 定义于 `track.h:51`） |
| DebugFrame 构造位置 | `include/SuTengDriver.cpp` → `SutengDriver::ProcessPcapCloud()`，`m_tracker.update()` 之后（~L1066） |
| 点云是否需要深拷贝 | **推荐 0 拷贝**：把 `pFilteredPointCloud` 的分配移到帧循环内（每帧新对象），DebugFrame 直接持同一 `shared_ptr`；若不做该改动则需 1 次深拷贝（当前缓冲复用会跨帧改写，存在数据竞争） |
| Cluster / Track 存储方式 | 均按值拷贝进 DebugFrame（体积很小） |
| Queue 策略 | 复用相机项目「有界 deque + 生产端丢最旧 + 消费端读最新 `.back()`」模式，容量建议 2~3 |
| 相机项目 Foxglove 代码可复用度 | `foxglove_publisher` 类：**轻微修改**；`foxglove_queue`：**不能直接用**（仅复用其 DebugFrame 队列模式）；`foxglove_struct.h`：**不能直接用** |
| Foxglove SDK 库 | 头文件 `foxglove/foxglove-c.h` 已与相机项目**完全相同**；但现有 `libfoxglove.so` 是 **aarch64**，x86 开发构建需另行提供 x86 版本（详见 §15） |
| 是否阻塞算法 | 不会。算法线程只做有界 `push`（μs 级）；SDK 发送发生在 Foxglove 线程 |

---

## 1. 当前低矮检测单帧数据流

来源：`include/SuTengDriver.cpp` → `SutengDriver::ProcessPcapCloud()`（L883 起），一帧处理链路如下：

```
RS-LiDAR 原始点（robosense PointCloudMsg）
   │  SuTengDriver::driverReturnPointCloudToCallerCallback()
   │  （driver 内部解析线程 → stuffed_cloud_queue）
   ▼
pInputCloud                                  （L935~943 声明，L954 clear，L988 前逐点填充）
   │  PointCloudTransform(pInputCloud, R_Combined, pFilteredPointCloud)   ← L988
   │    1. 刚体变换 dst = R*src + t（雷达系 → 车体系）
   │    2. Body Filter（x <= expand_x 丢弃）
   │    3. ROI（x >= roi_x_max / y 越界丢弃）
   ▼
pFilteredPointCloud
   │  ElevationMapGroundFilter::ProcessWithObstacleDetection(...)         ← L1008
   │    └ BuildGrid → ComputeGroundHeight → ComputeSlope → RegionGrowing
   │      → GenerateGroundMask → ComputeGroundReference → AnalyzeVerticalOccupancy
   │      → ClusterObstacleGrid → clusters = ClusterObstacleGrid()
   ▼
outputClusters（std::vector<GridCluster>，L1006 声明，L1008 填充）
   │  m_hdmapFilter.filterClusters(outputClusters, ...)                   ← L1043（就地打标签）
   │    （仅写 in_road / map_valid / map_confidence，不删簇、不改几何）
   ▼
outputClusters（已含 HDMap 标签）
   │  ConvertClustersToTrackedObstacles(outputClusters, detections)       ← L1057（const 只读）
   ▼
detections（std::vector<TrackedObstacle>，无 id，L1056 声明）
   │  m_tracker.update(detections, rec_timestamp_ms)                      ← L1058（就地改 vtrackings）
   ▼
m_tracker.vtrackings（跨帧稳定 ID 的最终目标）
   │
   ├──► DebugViewer 叠加图（DrawClusterOverlay / DrawTrackOverlay，算法线程内同步执行）
   ├──► (pcap) SavePcd + m_visBuffer.Publish(...)                         ← L1115~1131（PCL 主线程渲染）
   └──► (online) ConvertTrackToS2ObstacleBox(...) → sendUdpMsg(...)       ← L1140~1147（UDP 下游）
```

> 重要：`ProcessWithObstacleDetection` 的输入参数为 `const PointCloud2Intensity::Ptr& cloud`（`ElevationMapGroundFilter.cpp:964`），**只读** `pFilteredPointCloud`，不修改；输出 `clusters` 通过 `clusters = ClusterObstacleGrid();`（L1027）整体 move 赋值。
>
> 另有 PCD 单帧调试路径（`SutengDriver::Start()`，L1300~1390），处理方式相同但只跑一帧；第一阶段以 `ProcessPcapCloud` 为唯一接入点，PCD 路径可选扩展。

---

## 2. 当前线程关系（仅与本链路相关）

```
┌──────────────────────── main 线程（main.cpp）─────────────────────────┐
│  stDriver.Init() + stDriver.Start()                                    │
│  pcap/pcd：PCL 可视化主循环（InitGroundViewer + visBuf.Consume 渲染）    │
│  online：sleep 保活                                                    │
│  SignalThread：SIGINT → TerminateFlag → 退出                            │
└───────────────────────────────────────────────────────────────────────┘

┌─ rs_driver 内部解析线程 ─┐      ┌─ m_SutengDriverThread（Start() 内创建）──────┐
│ 解析包 → 构造点云          │      │ pcap/online：m_driver.init + 创建 m_processThread │
│ driverReturnPointCloud    │      │             + m_driver.start()                    │
│   → stuffed_cloud_queue   │      └──────────────────────────────────────────────────┘
└───────────┬──────────────┘
            ▼ stuffed_cloud_queue（SyncQueue，线程安全）
┌─ m_processThread = Algorithm Thread（ProcessPcapCloud）────────────────┐
│  popWait → 坐标变换 → GroundFilter → Cluster → HDMap Filter → Tracker   │
│  → DebugViewer 同步绘图 → UDP 发送                                      │
└────────────────────────────────────────────────────────────────────────┘
```

**与 Foxglove 相关的线程只有一条：`m_processThread`（Algorithm Thread），运行 `ProcessPcapCloud`。**
- `m_tracker`（`SimpleTracker` 成员）只被 `m_processThread` 访问（单线程），无外部竞争。
- `LocalizationManager` 的定位喂入线程只影响 HDMap 标签，与 Foxglove 无关。
- 相机项目「SDK Callback → 队列 → TrackingThread → FoxgloveThread」的三线程结构**不需要**照搬；低矮检测只需要一个算法线程 + 一个新增 Foxglove 消费者线程。

---

## 3. 目标线程关系

```
┌─ Algorithm Thread（m_processThread，现有）─────────────────────────────┐
│  pFilteredPointCloud → ProcessWithObstacleDetection → outputClusters    │
│  → hdmapFilter → ConvertClustersToTrackedObstacles → m_tracker.update() │
│  → m_tracker.vtrackings                                                 │
│          │                                                              │
│          ├──────────────→ UDP（现有，不变）                              │
│          │                                                              │
│          └── 构造 DebugFrame ──► PushDebugFrame（有界队列，μs 级）         │
└──────────────────────────────┬──────────────────────────────────────────┘
                               ▼
                  ┌─ DebugFrameQueue（mutex + cv，生产端丢最旧）─┐
                               │
                               ▼
┌─ FoxglovePublisherThread（新增消费者线程）──────────────────────────────┐
│  wait cv → 读最新 DebugFrame（取 shared_ptr 后释放锁）                   │
│  → PublishPointCloud(pFilteredPointCloud)                               │
│  → PublishClusterMarkers(outputClusters)                                │
│  → PublishTrackMarkers(vtrackings)                                      │
└────────────────────────────────────────────────────────────────────────┘
```

**核心保证**：Foxglove 发布慢时，算法线程只经历一次 O(1) 的入队（拿锁 → push → 释放），**不会被长期阻塞**。

---

## 4. DebugFrame 最小设计

根据低矮检测项目的**实际类型**（非相机项目类型）设计：

```cpp
// 建议放入 low 项目 foxglove/ 目录（与 foxglove-c.h 同级）
namespace Lidar_Low_Detection {

struct DebugFrame {
    // 帧时间戳（唯一必要的标识字段，理由见下）
    unsigned long long timestamp_ms = 0;   // = rec_timestamp_ms（雷达包时间戳 ms），
                                           //   用于 Foxglove log_time / 时间排序

    // ① 处理后的点云：直接共享 PCL 点云的 shared_ptr（0 深拷贝方案见 §6/§17）
    PointCloud2Intensity::Ptr pointcloud;  // = pFilteredPointCloud

    // ② 检测阶段 Cluster：按值拷贝（体积小，避免与后续帧共享）
    std::vector<GridCluster> clusters;     // = outputClusters（含 HDMap 标签，见 §7）

    // ③ Tracking 最终目标：按值拷贝（vtrackings 每帧被 update() 就地改写，必须拷贝）
    std::vector<TrackedObstacle> trackings; // = m_tracker.vtrackings
};

}
```

### 字段逐个说明

| 字段 | 类型 | 数据来源 | 创建位置 | 生命周期 | 是否拷贝 |
|---|---|---|---|---|---|
| `timestamp_ms` | `unsigned long long` | `rec_timestamp_ms`（L1008 前已算好） | ProcessPcapCloud | 值语义 | 无需拷贝 |
| `pointcloud` | `PointCloud2Intensity::Ptr` | `pFilteredPointCloud` | ProcessPcapCloud（L935 声明） | 见 §6，**有数据竞争风险** | **推荐 0 拷贝（每帧新对象）**，兜底 1 次深拷贝 |
| `clusters` | `std::vector<GridCluster>` | `outputClusters` | ProcessPcapCloud（L1006 声明，循环内局部） | 每帧新 vector，局部，安全 | 按值拷贝（小） |
| `trackings` | `std::vector<TrackedObstacle>` | `m_tracker.vtrackings` | SimpleTracker 成员，跨帧累积 | **每帧被 update() 就地改写** | **必须按值拷贝** |

### 关于 `timestamp_ms` 的说明（唯一对「不新增字段」原则的例外）

用户明确不新增 `recv_timestamp` 等相机项目字段。但 Foxglove SDK 发布点云 / 消息时**必须**提供 `log_time`（纳秒），否则 Studio 无法正确排序/显示时间轴；点云消息还需要 `timestamp`。低矮检测里现成的、语义正确的帧时间就是 `rec_timestamp_ms`（雷达包时间戳）。只加这一个 `uint64` 字段即可满足发布需求，**不加** `sensor_timestamp` / `depth_frame_id` / `app_frame_id`。

> 若坚持 DebugFrame 完全不存时间戳，则需在 `PushDebugFrame` 处单独传时间戳给 publisher，等价但更绕。建议保留。

---

## 5. 数据分类

### 必须保存
- `pFilteredPointCloud` —— 展示 ROI / 地面 / 低矮障碍物 / 点云分布。
- `outputClusters` —— 展示「检测到了什么」（含 HDMap 标签）。
- `m_tracker.vtrackings` —— 展示「Tracking 最终保留了什么」。
- `timestamp_ms` —— 发布必需（见 §4）。

### 可选保存（本阶段不加入）
- `pGroundCloud` / `pObstacleCloud` —— 对地面/障碍物点分类有调试价值，但**与 `pFilteredPointCloud` 存在完全相同的缓冲复用竞争问题**（同为循环外声明、每帧 clear+重填），加入需同样处理（每帧新分配或深拷贝）。按用户要求，本阶段**不加入**。

### 暂时不保存
- Elevation Grid 全部 Cell / 每 Cell 的 `layer_histogram` / `slope` / `ground_reference_z` / `occupied_layers` / `label` / `is_ground` / `is_obstacle_candidate` —— 后续如需分析「坡面误检 / Ground Filter / Candidate Cell」再单独扩展。
- 原始点云 `pInputCloud`（未经变换，无调试价值）。
- UDP 数据、`newS2AviodObject`、HDMap polygons、定位位姿（DebugViewer 已另画）。
- 相机项目的 `FrameMatcher` / `PointCloudMsg` / `ObstacleMsg` / `raw_obstacles` / `TrackingResult` / `GroundPlane` —— **一律不引入**。

---

## 6. PointCloud 生命周期（重点）

### 6.1 现状（存在缓冲复用导致的潜在竞争）

`ProcessPcapCloud` 中：

```cpp
// L935~938：四个点云指针在 while 循环【外】声明一次
PointCloud2Intensity::Ptr pInputCloud(new PointCloud2Intensity);
PointCloud2Intensity::Ptr pFilteredPointCloud(new PointIntensity);   // 复用同一对象
...
while (m_running) {
    // L954：每帧 clear
    pFilteredPointCloud->clear();
    ...
    // L988：clear + 重新填充（PointCloudTransform 内部也 clear）
    PointCloudTransform(pInputCloud, R_CombinedTransMatrix, pFilteredPointCloud);
    // L1008：只读传入，不修改
    ProcessWithObstacleDetection(pFilteredPointCloud, ...);
}
```

**关键结论**：
- `pFilteredPointCloud` 指向的底层 `pcl::PointCloud` 对象**每一帧都被 `clear()` 后重新写入**。
- `ProcessWithObstacleDetection` 执行完成后，`pFilteredPointCloud` 仍然有效（未被改）。
- **但**：若 DebugFrame 直接持有这个 `shared_ptr`，下一帧 `clear()/重填` 会**就地改写同一对象** → 只要 Foxglove 线程还没发完这一帧（或队列里还留着它），就会读到被下一帧污染的数据 → **数据竞争 / 显示错帧**。

### 6.2 结论：DebugFrame.pointcloud 应该用什么

| 方案 | 做法 | 额外点云拷贝 | 算法改动 | 评价 |
|---|---|---|---|---|
| **A（推荐，0 拷贝）** | 把 `pFilteredPointCloud` 的**分配移到帧循环内**：每帧 `pFilteredPointCloud = std::make_shared<PointCloud2Intensity>();`（删掉 L954 的 clear），DebugFrame 直接持有同一 `shared_ptr` | **0 次** | 极小（挪 1 行声明、删 1 行 clear） | 每帧一个全新对象，天然无竞争；代价是每帧一次 `make_shared` + 内部 vector 分配，与算法自身 transform 的开销相当 |
| **B（保守兜底，1 次拷贝）** | 构造 DebugFrame 时 `debugFrame->pointcloud = std::make_shared<PointCloud2Intensity>(*pFilteredPointCloud);` | 1 次深拷贝 | 无 | 最安全、最不侵入；每帧多一次 ~N×16B 的 memcpy（滤波后通常几万点，约 0.2~0.8 MB） |
| C（不可行） | DebugFrame 直接存现有 `pFilteredPointCloud` 的 shared_ptr 且不改循环 | 0 | 无 | **有数据竞争，禁止** |

> 若追求「为 Foxglove 0 次额外 PointCloud 深拷贝」，**方案 A 是唯一能满足的路径**。若希望最小化算法线程改动，选方案 B（1 次拷贝，已明确为「现有生命周期确实要求」的情形，可接受）。
>
> 附：`pcl::PointXYZI` 因 16 字节对齐，实际 `sizeof` 为 **32 字节**（PCL 1.8，布局 x@0,y@4,z@8,[pad@12],intensity@16）。发布时用 `point_stride = sizeof(pcl::PointXYZI)` + `offsetof` 计算字段偏移，可直接把 `points.data()` 交给 SDK，**发布侧无需逐点转换/拷贝**（见 §12）。

---

## 7. Cluster 生命周期

- **创建**：`outputClusters` 是 `ProcessPcapCloud` **循环内局部变量**（L1006，每帧新 vector）；内容由 `clusters = ClusterObstacleGrid();`（`ElevationMapGroundFilter.cpp:1027`）move 赋值，每帧全新。
- **`hdmapFilter.filterClusters(outputClusters, ...)`（L1043）**：签名是 `std::vector<GridCluster>&`，**就地修改**（只写 `in_road / map_valid / map_confidence`），**不删簇、不改几何**（`hdmap_filter.h` 注释已明确「软约束，只打标签」）。
- **`ConvertClustersToTrackedObstacles(outputClusters, detections)`（L1057）**：入参 `const std::vector<GridCluster>&`，**只读不改**。
- 因此 `outputClusters` **没有跨帧共享问题**（每帧新 vector、循环内局部、生命周期安全），DebugFrame 直接**按值拷贝**即可（几十个簇，含 `cell_indices` 小 vector，开销可忽略）。

### 结论：DebugFrame 应该在哪构造（关于「原始 Cluster」的取舍）

用户倾向在 `ProcessWithObstacleDetection` 之后立刻保存「GroundFilter 原始 Cluster」。分析后：

- `filterClusters` **只加标签不改几何**，所以在 `m_tracker.update()` 之后保存 `outputClusters`，得到的**几何与「原始 Cluster」完全一致**，还额外带上了 `in_road / map_valid / map_confidence`（对 Foxglove 调试更有价值）。
- 若一定要在 filterClusters **之前**快照，需要提前拷贝一份、攒到帧尾再组装 DebugFrame，多一步拷贝且丢标签——**不推荐**。

**推荐**：单一构造点（见 §9），保存「已打 HDMap 标签的 outputClusters」，视作 GroundFilter 原始 Cluster（几何未变）。

---

## 8. Tracking 生命周期

`m_tracker`（`SimpleTracker`）是 `SutengDriver` 成员，`vtrackings` **跨帧累积**，`update()`（`track.cpp:110`）**每帧就地改写**：

| update() 行为 | 对 vtrackings 的影响 |
|---|---|
| 本帧无检测（detections 空） | 现有 track 按 `vx/vy` 预测位置，`lastSeen++`，随后 `removeLostTargets()` |
| 匹配成功 | 用检测值更新 `pos_x/y/z`、`depth/width/height`、`corners`，更新 `vx/vy`，`age++`，`lastSeen=0` |
| 匹配失败 | `lastSeen++`，位置不更新（保留预测值） |
| 未匹配检测 | 新建 track（`id = nextTrackID++`），push_back |
| `removeLostTargets()` | 删除 `lastSeen > 10` 的目标 |

**回答用户五个问题**：
1. 当前帧 Tracking 结果？ —— 是，包含本帧匹配/新建目标。
2. 当前所有 active track？ —— 是。
3. 包含历史未匹配 Track？ —— **是**（`lastSeen <= 10` 的 track 仍在列表里）。
4. 是否可能包含 missed Track？ —— **是**（`lastSeen > 0` 的 track 保留，位置为预测值，最多保留 10 帧）。
5. 每次 update 后是否变化？ —— **是，每帧就地改写**。

**因此**：
- DebugFrame **必须按值拷贝** `vtrackings`（否则下一帧 update() 会污染 DebugFrame 里的数据）。
- 拷贝成本极低（个位数目标）。
- Foxglove 显示时可直接用 `track.id`（稳定 ID）、`pos_x/y/z`、`depth/width/height`、`vx/vy`、`lastSeen`；`lastSeen>0` 的 missed track 可以用不同颜色/透明度区分（**只用已有数据，不重新计算**）。
- 构造位置必须在 `m_tracker.update()` **之后**。

---

## 9. DebugFrame 构造位置（最重要）

- **文件**：`include/SuTengDriver.cpp`
- **类**：`SutengDriver`
- **函数**：`ProcessPcapCloud()`
- **具体位置**：`m_tracker.update(detections, rec_timestamp_ms)`（**L1058**）之后、`vtrackings` 日志循环（L1062~1066）之后、DebugViewer `DrawTrackOverlay` / UDP 发送（L1090~1147）之前，即 Tracker 块 `{ ... }` 闭合处（约 **L1070**）。

```
PointCloudTransform()          L988
  ↓
ProcessWithObstacleDetection() L1008  → outputClusters
  ↓
hdmapFilter.filterClusters()   L1043  → outputClusters（含标签，几何不变）
  ↓
ConvertClustersToTrackedObstacles() L1057 → detections
  ↓
m_tracker.update()             L1058  → m_tracker.vtrackings
  ↓
★ 构造 DebugFrame 并 PushDebugFrame（约 L1070）   ← 三者在此刻同时有效且处于最终状态
  ↓
DebugViewer 叠加 / SavePcd / visBuffer / UDP（不变）
```

**原因**：
1. 这是**唯一**同时拿到 `pFilteredPointCloud`（最终变换+滤波后）、`outputClusters`（最终 Cluster 列表）、`m_tracker.vtrackings`（Tracking 最终结果）且三者都处于**最终状态**的点。
2. 与用户 §15 的目标位置完全一致。
3. 对 `outputClusters` 的「在 filterClusters 前快照」需求已分析：由于 `filterClusters` 不改几何，此处保存的即 GroundFilter 原始 Cluster 的几何（另含 HDMap 标签），无需提前快照（见 §7）。
4. PCD 单帧路径（`Start()` L1341/L1382）可后续按需加同一个调用；第一阶段只接入 `ProcessPcapCloud`。

---

## 10. DebugFrameQueue 设计

### 相机项目现有机制（已确认）
- `foxglove_queue.h/.cpp`：`g_pointcloud_queue`（`deque<shared_ptr<PointCloudMsg>>`，MAX=10）+ `PushPointCloud` + `FindPointCloud` —— **是 PointCloudMsg 专用的缓存队列**。
- 真正可复用的 DebugFrame 队列在相机项目 `src/track.h/.cpp`：
  ```cpp
  constexpr size_t MAX_DEBUG_FRAME_CACHE_SIZE = 10;
  extern std::deque<std::shared_ptr<DebugFrame>> g_debug_frame_queue;
  extern std::mutex                              g_debug_frame_mutex;
  extern std::condition_variable                 g_debug_frame_cv;
  void PushDebugFrame(std::shared_ptr<DebugFrame> frame);   // 生产端 FIFO 丢最旧 + notify
  ```
  - 消费者 `FoxglovePublisherThread`：`wait` → **读 `.back()`（最新帧），不 pop** → 持 shared_ptr 出锁发布。

### 低矮检测项目建议（第一阶段只分析，不实现）
- 直接套用上述「**有界 deque + mutex + cv + 生产端丢最旧 + 消费端读最新 `.back()`**」模式。
- **容量**：建议 `MAX_DEBUG_FRAME_CACHE_SIZE = 2~3`（Foxglove 是实时调试显示，不要求历史帧；容量小则内存有界、延迟低）。甚至可退化为「只保留最新（size=1）」，但 2~3 能吸收短时突发。
- 丢弃策略：生产端 `size() >= MAX` 时 `pop_front()`（丢最旧），消费者始终拿到最新帧 —— 对应相机项目 `queue_design_foxglove.md` 里推荐的「短队列 + Latest-Only」。
- 共享所有权：队列存 `shared_ptr<DebugFrame>`，算法线程入队后立即释放本帧引用；消费者持引用期间对象保活（结合 §6 方案 A 的点云所有权，安全）。
- **不复制**相机项目 `foxglove_queue`（PointCloudMsg 专用），也**不需要** `FindPointCloud`（本设计无跨队列帧匹配）。

---

## 11. FoxglovePublisherThread 设计

复用相机项目 `FoxglovePublisher` 类的生命周期接口，重写消费者线程：

```
FoxglovePublisherThread(publisher):
  while (g_running):
    { lock(mutex); cv.wait(有帧 或 退出); frame = g_debug_frame_queue.back(); }   // 出锁
    if (!frame || !publisher->IsRunning()) continue;
    publisher->PublishPointCloud(*frame->pointcloud);      // 点云（pFilteredPointCloud）
    publisher->PublishClusterMarkers(frame->clusters);     // Cluster 立方体/线框 + ID
    publisher->PublishTrackMarkers(frame->trackings);      // Track 立方体 + ID + 速度箭头
```

**关键点**：
- **必须在取到 `shared_ptr` 后释放队列锁再做 SDK 发布**（相机项目已如此）：SDK 发送可能慢（网络拥塞/浏览器慢），若持锁发布会把算法线程的 push 也阻塞住。
- 运行标志：用 `main.cpp` 已有的 `TerminateFlag`（或为其增加 `g_running` 别名），退出时置位并 `cv.notify_all()`。
- Publisher 生命周期（Init → StartServer → CreateXxxChannel → ... → Stop）由 **main.cpp** 管理（与 PCL 可视化主循环并列），或包一层小类。
- 发布线程的创建/join 也放 main.cpp（pcap / pcd / online 三种模式都启动；无客户端连接时 SDK 内部应可空转，必要时按 `IsRunning` 跳过）。

---

## 12. Foxglove 数据类型映射

### 12.1 `pFilteredPointCloud` → Foxglove PointCloud

- 通道：`foxglove_channel_create_point_cloud`，topic 建议 `/low_obstacle/pointcloud`，frame_id 建议 `base_link`（车体系）。
- 字段（`pcl::PointXYZI`，**point_stride = sizeof(pcl::PointXYZI)，PCL 1.8 下为 32 字节**；字段偏移用 `offsetof` 动态计算）：

| offset | name | type |
|---|---|---|
| offsetof(x)=0 | x | FLOAT32 |
| offsetof(y)=4 | y | FLOAT32 |
| offsetof(z)=8 | z | FLOAT32 |
| offsetof(intensity)=16 | intensity | FLOAT32 |

- 发布：`pcd.data = (uchar*)cloud->points.data(); pcd.data_len = n * sizeof(pcl::PointXYZI); pcd.point_stride = sizeof(pcl::PointXYZI);` —— **点云内存可直接零转换给 SDK**（无需逐点搬运）。
- 说明：相机项目 `CompactPoint` 是 RGB 7 字段（stride=16）；低矮项目没有 RGB，只有 intensity → 4 字段即可，不需要 RGB。如需按高度/强度着色，可在 Foxglove Studio 侧用 intensity 字段配色（无需改代码）。

### 12.2 `GridCluster` → Foxglove Marker（Scene Entity / Cube）

`GridCluster` 已有可直接映射的字段（`ElevationMapGroundFilter.h:258`）：`center_x/y/z`、`length/width/height`、`min/max`、`obb_*`、`id`、`in_road/map_valid/map_confidence`、`point_num`。

| GridCluster 字段 | Foxglove 表达 |
|---|---|
| `center_x/y/z` + `length/width/height` | `cube_primitive`（或 scene_entity 的 cube），尺寸 = (length, width, height) |
| `has_obb` + `obb_corners[4]` + `obb_angle` | 有 OBB 时画 4 角点 `line_primitive` 多边形（更贴合朝向） |
| `id` | `text_primitive` 标签 |
| `in_road / map_valid / map_confidence` | 可选：颜色区分（road 内绿、road 外红）或 metadata |
| `point_num` | 可选 metadata / 文本 |

- 通道选择：SDK 提供 `foxglove_channel_create_scene_entity`（一个 entity 里可同时含 cubes/lines/texts，且有 `id` 替换语义 + `scene_entity_deletion` 清场）或单类型通道（`cube_primitive` / `line_primitive` / `text_primitive`）。
- **建议**：用 `scene_entity` 通道，每帧发布一个 entity（cubes 数组 = 所有 cluster，texts = ID），配合 `foxglove_channel_create_scene_entity_deletion` 在 0 簇时清场。颜色建议：Cluster = 红色系（检测阶段）。

### 12.3 `m_tracker.vtrackings` → Foxglove Marker

`TrackedObstacle`（`track.h:51`）已有字段：`id`、`pos_x/y/z`、`depth/width/height`、`corners[4]`、`vx/vy`、`age`、`lastSeen`、`cluster_id`。

| TrackedObstacle 字段 | Foxglove 表达 |
|---|---|
| `id` | `text_primitive` 稳定 ID |
| `pos_x/y/z` + `depth/width/height` | `cube_primitive`（跟踪框） |
| `vx/vy` | `line_primitive` / `arrow_primitive` 速度向量 |
| `lastSeen` | 可选：missed track（>0）用不同颜色/透明度 |
| `age` | 可选 metadata |

- 与 Cluster 用**不同颜色**（如 Cluster=红，Track=绿/青），便于对比「检测到 vs 跟踪保留」。
- 只使用已有字段，**不重新计算** Tracking 信息。

### 12.4 小结映射

```
pFilteredPointCloud  → PointCloud 通道（4 字段，stride=16，零转换）
GridCluster          → Scene Entity / Cube / Line（center+尺寸 / OBB 角点 / id 文本）
TrackedObstacle      → Scene Entity / Cube / Line / Text（pos+尺寸 / id / 速度箭头）
```

---

## 13. 项目 A Foxglove 代码哪些可以直接复用

以相机项目 `foxglove/` 四个文件为对象：

| 文件 | 判定 | 说明 |
|---|---|---|
| `foxglove_publisher.h` | **轻微修改** | 类主体（Init / StartServer / CreatePointCloudChannel / PublishPointCloud / Stop / MakeTimestamp / GetClientCount）可直接搬。修改点：①字段描述符从 RGB 7 字段改为 intensity 4 字段；②topic / frame_id / server name；③新增 `CreateClusterChannel` / `CreateTrackChannel` / `PublishClusterMarkers` / `PublishTrackMarkers`（相机项目没有）；④`FoxglovePublisherThread` 改为发布「点云 + cluster + track」三路（现只发点云） |
| `foxglove_publisher.cpp` | **轻微修改** | 同上；SDK 调用模式（`foxglove_context_new` / `foxglove_server_start` / `foxglove_channel_log_*` / `foxglove_server_stop` / `foxglove_context_free`）原样复用 |
| `foxglove_queue.h/.cpp` | **不能直接使用** | `g_pointcloud_queue` / `PushPointCloud` / `FindPointCloud` 是相机 PointCloudMsg 专用。但其「有界 deque + 丢最旧 + shared_ptr」语义正是低矮项目要的，改造成 `DebugFrameQueue` 复用其**模式**（见 §10） |
| `foxglove_struct.h` | **不能直接使用** | `CompactPoint` / `PointCloudMsg` 是相机 SDK 类型。低矮项目已有自己的类型（`PointCloud2Intensity` / `GridCluster` / `TrackedObstacle`），DebugFrame 结构按 §4 新建 |

补充：
- `foxglove-c.h`：低矮项目 `foxglove/foxglove-c.h` 与相机项目 `include/foxglove-c.h` **逐字节相同**（已 `diff` 确认），SDK 头文件已就位，无需再拷贝。
- 相机项目的 DebugFrame 队列实现（`src/track.h/.cpp` 的 `PushDebugFrame`）是**直接可复用的范式**（约 20 行），低矮项目可照搬改写。

---

## 14. 项目 B 需要增加哪些文件（只列必要）

| 文件 | 必要性 | 说明 |
|---|---|---|
| `foxglove/debug_frame.h` | 新增 | `DebugFrame` 结构 + `MAX_DEBUG_FRAME_CACHE_SIZE` + 全局队列/mutex/cv + `PushDebugFrame` 声明 |
| `foxglove/debug_frame.cpp` | 新增 | 全局队列定义 + `PushDebugFrame` 实现（~20 行） |
| `foxglove/foxglove_publisher.h/.cpp` | 新增（从相机项目拷贝改造） | Publisher 类 + 消费者线程 |
| `src/main.cpp`（修改） | 修改 | 创建 publisher + 启动 FoxglovePublisherThread + 退出时 Stop/join |
| `include/SuTengDriver.cpp`（修改） | 修改 | `ProcessPcapCloud` 在 L1070 处构造 DebugFrame + PushDebugFrame（并按 §6 方案 A 调整点云生命周期） |

**不需要新增**：
- 不需要 `foxglove_struct.h`（类型已有）。
- 不需要新的 `PointCloudMsg` / `ObstacleMsg` / `FrameMatcher` 任何文件。
- 不需要复制相机项目的 `track.h` / `track.cpp`（低矮项目已有自己的 tracker）。
- 若已有合适承载点（如把 Foxglove 相关代码放进现有 `common/` 或 `include/`），可优先并入既有架构；但 `foxglove-c.h` 已在 `foxglove/` 目录，publisher/debug_frame 放同目录最自然。

---

## 15. CMake / Makefile 修改点（只分析，不修改）

### 15.1 CMakeLists.txt（x86 开发构建，`build/`）
1. **头文件**：`INCLUDE_DIRECTORIES(... ./foxglove ...)`（当前未包含）。
2. **源文件**：把 `foxglove/foxglove_publisher.cpp`、`foxglove/debug_frame.cpp` 加入编译（可 `AUX_SOURCE_DIRECTORY(./foxglove FOX)` 或显式列出）。
3. **链接库**：`LINK_DIRECTORIES(<含 libfoxglove 的目录>)` + `target_link_libraries(... foxglove ...)`（`.so` 或 `.a`）。

### 15.2 根 Makefile（ARM 交叉构建，`low_detection_arm`）
- 源文件列表加入 `foxglove/foxglove_publisher.cpp`、`foxglove/debug_frame.cpp`。
- `THIRDPARTY`/链接路径加入 libfoxglove，`LDLIBS` 加 `-lfoxglove`。

### 15.3 ⚠️ Foxglove 库的平台问题（关键前提）
- 现有唯一一份 SDK 库来自相机项目：`libfoxglove.so` / `libfoxglove.a`（`mrdvs_camera_newsdk_26year_callback_v2.1/lib/`）。
- `file` 检查结果：**该 `libfoxglove.so` 是 `ARM aarch64`**。
- 低矮检测项目两条构建路径：
  - **ARM 交叉（根 Makefile → `low_detection_arm`）**：可直接复用相机项目的 aarch64 `libfoxglove.so/.a`。
  - **x86 开发（CMake `build/low_detection`，`g++` x86-64）**：**当前没有 x86 版 libfoxglove**，需要自行获取/构建 x86 版 SDK（Foxglove 官方 SDK 或厂商提供）——这是 Phase 2 能否在 PC 上联调的**前置条件**。

> 第一阶段不修改任何构建文件；上表仅作为 Phase 2 Step 1 的依据。

---

## 16. 线程安全分析

| 对象 | 涉及线程 | 风险 | 措施 |
|---|---|---|---|
| `DebugFrameQueue` | 算法线程（生产）+ Foxglove 线程（消费） | 队列并发 | 沿用 mutex + cv；push/pop 都在锁内完成；生产端丢最旧 |
| 点云缓冲 `pFilteredPointCloud` | 算法线程写 + Foxglove 线程读 | **跨帧复用改写 → 数据竞争** | 采用 §6 方案 A（每帧新对象）或方案 B（拷贝）；DebugFrame 持 shared_ptr 保活 |
| `outputClusters` | 仅算法线程 | 无共享 | 循环内局部 + 按值拷贝进 DebugFrame |
| `m_tracker.vtrackings` | 仅算法线程 | 无共享 | 按值拷贝进 DebugFrame（`TrackedObstacle` 自带深拷贝 ctor/赋值，含 `unique_ptr<KalmanFilter2D>`） |
| SDK 发布（`foxglove_channel_log_*`） | 仅 Foxglove 线程 | 无共享 | publisher 只在 Foxglove 线程调用；`IsRunning`/`channel_created_` 用 atomic |
| Publisher 生命周期（Init/Start/Stop） | main 线程（创建/销毁）+ Foxglove 线程（用） | 析构竞态 | 先 `Stop()` 并 join 发布线程，再释放 publisher（与相机项目一致） |

---

## 17. 内存拷贝分析（重点回答：为 Foxglove 一帧点云最终几次 copy？）

| 数据 | 方案 A（推荐） | 方案 B（兜底） |
|---|---|---|
| `pFilteredPointCloud` → DebugFrame | **0 次额外 deep copy**（每帧新对象，同一 shared_ptr 移交；transform 是算法自身一次写入） | +1 次 deep copy（make_shared + copy） |
| `outputClusters` → DebugFrame | 1 次小 vector 拷贝（几十簇） | 同 |
| `vtrackings` → DebugFrame | 1 次小 vector 拷贝（个位目标） | 同 |
| 发布侧（Foxglove 线程） | SDK 序列化必然产生一次数据搬运（发生在消费者线程，**不计入算法侧**） | 同 |
| 队列峰值内存 | ≤ 2~3 帧 ×（点云 + 少量簇/轨） | 同（但每帧多一份点云拷贝，峰值略高） |

**结论**：
- **方案 A 可以把「为 Foxglove 增加的点云深拷贝」降到 0 次**，条件是允许把 `pFilteredPointCloud` 的分配挪进帧循环（§6）。
- 方案 B 是「0 额外拷贝做不到」时的说明：因为当前缓冲在帧间复用，DebugFrame 若持同一 shared_ptr 会被下一帧改写，必须隔离（拷贝），**无法在不改算法线程内存策略的前提下实现 0 拷贝**。
- `pcl::PointXYZI` 内存布局（16 字节对齐，PCL 1.8 下 sizeof=32）与 Foxglove 点云兼容：`point_stride=sizeof` + `offsetof` 字段，发布侧也**无需转换**，进一步省掉一次格式转换拷贝。

---

## 18. 实时性分析

| 问题 | 结论 |
|---|---|
| Foxglove 发布耗时是否影响算法？ | **不影响**。SDK 发送全部发生在 Foxglove 线程；算法线程只做一次有界 push（拿锁→入队→notify，μs 级），随后立即继续 UDP 发送 |
| DebugFrame 创建耗时？ | 方案 A：每帧一次 `make_shared`（点云分配）+ 两个小 vector 拷贝，约几 μs~几十 μs；与算法线程已有的逐帧 `LOG_RAW`、DebugViewer 绘图相比可忽略 |
| Queue 是否可能阻塞？ | 不会阻塞生产者。生产端 `size() >= MAX` 时丢最旧，O(1) 入队；消费者 `wait` 由 cv 唤醒。唯一需注意：消费者**取到 shared_ptr 后必须释放锁再发布**，否则慢发布会反压生产者 |
| 慢消费者影响 | 队列有界 → 内存有界；旧帧被丢弃，Foxglove 始终显示最新帧（实时调试语义） |
| 风险点 | 方案 A 每帧一次点云 malloc，可能引入轻微分配抖动；若在意可用 2~3 个缓冲的小型对象池/环形缓冲（Phase 2 可选优化） |

---

## 19. 第二阶段建议步骤（在确认本方案后执行）

```
Step 1  Foxglove SDK / Publisher 基础代码接入（先解决 x86 libfoxglove 前置条件，见 §15.3）
Step 2  增加最小 DebugFrame（foxglove/debug_frame.h/.cpp，§4）
Step 3  增加 DebugFrameQueue（复用相机项目模式，§10）
Step 4  在 ProcessPcapCloud L1070 构造 DebugFrame + PushDebugFrame（§9）
Step 5  FoxglovePublisherThread 消费 DebugFrame（§11）
Step 6  先显示 pFilteredPointCloud（4 字段点云，§12.1）
Step 7  增加 outputClusters（Scene Entity / Cube，§12.2）
Step 8  增加 m_tracker.vtrackings（§12.3）
Step 9  测试线程安全与实时性（§16/§17/§18）
```

> 每一步完成后按用户要求说明：改了什么 / 为何 / 改了哪些文件 / 是否改算法 / 是否影响 Tracking / UDP / 其他模块 / 是否增加 PointCloud copy / 是否增加阻塞风险 / 是否增加内存 / 如何验证。

---

## 20. 最终目标架构（确认）

```
                LiDAR PointCloud
                       │
                       ▼
            PointCloudTransform
                       │
                       ▼
              pFilteredPointCloud
                       │
                       ▼
   ElevationMapGroundFilter::ProcessWithObstacleDetection
                       │
                       ▼
                outputClusters
                       │
                       ▼
                HDMap Filter（只打标签）
                       │
                       ▼
              m_tracker.update()
                       │
                       ▼
              m_tracker.vtrackings
                       │
                       ├──────────────→ UDP（不变）
                       │
                       ▼
                  DebugFrame
            （pointcloud / clusters / trackings）
                       │
                       ▼
                  DebugFrameQueue（有界，丢最旧）
                       │
                       ▼
          FoxglovePublisherThread（独立线程）
                       │
                       ▼
                 Foxglove Studio
```

DebugFrame 只负责保存「一帧已完成低矮检测与 Tracking 的调试结果」，最小内容为 `pFilteredPointCloud` + `outputClusters` + `m_tracker.vtrackings`（+1 个发布必需的 timestamp）。不引入 FrameMatcher / PointCloudMsg 新架构 / ObstacleMsg / depth_frame_id / app_frame_id / sensor_timestamp。
