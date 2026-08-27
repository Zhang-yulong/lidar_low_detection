# HDMap + Localization 迁移实施文档（项目 B：低矮障碍物检测）

> 本文档记录将 **项目 A 的 HDMap 能力** 与 **项目 A/C 共用的车辆定位来源** 迁移到 **项目 B**（`/home/zyl/echiev_low_lidar_detection`）的**第一阶段**实施内容。
>
> 第一阶段目标（已完成）：
>
> ```text
> 车辆定位 (Localization)
> + HDMap (地图加载/解析/查询)
> + Coordinate Transform (车辆↔地图)
> + Cluster 级 HDMap 软约束
> ```
>
> **暂未实现**：Grid 级过滤、PointCloud 级过滤、Ground/Cluster/Tracker 算法修改。
> 实施原则：**最小修改、逐步验证、可关闭、可降级、不破坏原有低矮检测。**

---

## 1. 修改了哪些文件

### 1.1 新增文件（`mapfilter/` 目录）

| 文件 | 说明 |
|------|------|
| `mapfilter/coordinate_transformer.h/.cpp` | 车辆系 ↔ 地图系 坐标转换器（纯计算，无第三方依赖） |
| `mapfilter/localization_manager.h/.cpp` | 定位管理器（封装 MC 0x0a @ 9110 → STR_FUSIONLOC，有效性/降级/离线调试位姿） |
| `mapfilter/hdmap_manager.h/.cpp` | HDMap 管理器（封装移植的 read_hdmap，路径参数化加载、Section 查询、可行驶区域多边形、点面测试） |
| `mapfilter/hdmap_filter.h/.cpp` | HDMap 过滤（Cluster 级软约束，打标签不删除） |
| `mapfilter/test_coordinate_transform.cpp` | 坐标转换单元测试（vehicle→map→vehicle 互逆性） |

### 1.2 修改文件

| 文件 | 修改内容 |
|------|----------|
| `mapfilter/read_hdmap.h/.cpp` | 清理项目 A 特有依赖（`alarm_center.hpp`、`coordinate_convert_tool.h`/PROJ、`using namespace std`）；构造函数不再硬编码 MAPDIR 自动加载，新增 `load(path)` 路径参数化加载；补充 `g_dLon/g_dLat` 定义 |
| `mapfilter/convert.h` | `PI` 宏加 `#ifndef` 防重定义 |
| `include/ElevationMapGroundFilter.h` | `GridCluster` 增加 HDMap 软约束字段：`in_road`、`map_valid`、`map_confidence` |
| `common/ReadYamlFile.h/.cpp` | `SELF_DEBUG_CONFIG` 增加 `Localization` 节与 `HdmapFilter` 扩展字段；`HdmapFilter` 节改为可选；`mapPath` 为空回退默认地址 |
| `src/main.cpp` | `fusionLoc_set` 回调通知 `LocalizationManager`；定位订阅开关改为 `mapFilterModel && localizationEnable && (pcap|online) && !pcd`；定位订阅失败改为**降级不退出**；启动后 `configure` LocalizationManager |
| `include/SuTengDriver.h/.cpp` | 新增 `HDMapManager`/`HDMapFilter` 成员；`Init()` 中按配置加载 HDMap + 配置过滤器；`ProcessPcapCloud()` 中在 `Cluster` 之后、`Tracker` 之前调用 `filterClusters()` |
| `CMakeLists.txt` | 新增 `./mapfilter` 头文件路径、显式源码列表（不含项目 A 特有依赖的 get_roi/hdmap_roi_filter/coordinate_convert_tool）、坐标转换单测目标 |
| `config/debug_config.yaml` | 新增 `Localization` 节与 `HdmapFilter` 节（默认关闭） |
| `config/debug_config_EMX.yaml` | 新增 `Localization` 节，扩展 `HdmapFilter` 节 |
| `config/debug_config_E1.yaml` | 新增 `Localization` 节与 `HdmapFilter` 节（默认关闭） |

---

## 2. 每个文件为什么修改

- **`read_hdmap`（移植自项目 A）**：项目 A 的 `read_hdmap` 依赖 `alarm_center.hpp`（告警宏）、`coordinate_convert_tool.h`（PROJ 库）、`using namespace std`。项目 B 没有这些依赖，且设计文档明确"**不要把项目 A 的旧 Boost/PCL/PROJ 依赖整体带入项目 B**"。因此：
  - 删除 `alarm_center.hpp` 与 `SET_LIDAR_ALM_MAP_LOAD_FAIL` 调用；
  - 删除 `coordinate_convert_tool.h`（PROJ）——地图转换只用 RTKLIB ENU（`convert.cpp`）；
  - 构造函数不再自动加载 `MAPDIR`，改为 `load(path)` 显式加载并返回错误码（路径参数化）；
  - 头文件去掉 `using namespace std;`。
- **`GridCluster` 增加字段**：Cluster 级软约束需要在 Cluster 结构上打标签（`in_road`/`map_valid`/`map_confidence`），而不是 `clusters.erase()`。
- **`ReadYamlFile`**：新增 `Localization` / `HdmapFilter` 扩展配置；`mapPath` 为空回退默认（用户提出的方案：为空→默认地址，非空→使用指定地址，**可行且已实现**）。
- **`main.cpp`**：定位订阅按"需 HDMap 且非 pcd 模式"开启；`fusionLoc_set` 通知管理器；**订阅失败降级不退出**（见第 10 节原则）。
- **`SuTengDriver`**：持有 HDMap 模块，接入主检测流水线。
- **`CMakeLists.txt`**：把新源码纳入编译；`get_roi`/`hdmap_roi_filter`/`coordinate_convert_tool` 因依赖项目 A 的 `comment.h`/PCL/PROJ，第一阶段**不纳入编译**（需要时再迁移）。

---

## 3. LocalizationManager 如何工作

```
外部融合定位模块（组合导航 + 融合定位，独立进程）
   ↓ MC 组播 0x0a @ 9110           （与项目 A/C 同一数据源）
libcomm.so OpenfusionLocMCClient + fusionLoc_set_callback
   ↓ fusionLoc_set 回调（main.cpp，写全局 g_strFusionLocation + mutex）
LocalizationManager::onDataReceived()   （解析 STR_FUSIONLOC，记录时间戳/墙钟）
   ↓ 算法线程每帧
LocalizationManager::getPose(Pose&)      （读取最新位姿）
   ↓
Pose { x, y, heading_deg, timestamp_ms, valid }
```

- 数据结构：`STR_FUSIONLOC`（comm_2.7.1，与项目 A/C 逐字段一致），使用 `strPoint3fInMap.fX/fY`（地图坐标米）+ `fHeadingInMap`（度，北=0 顺时针）。
- 线程安全：回调线程（写）与算法线程（读）用内部 `std::mutex` 保护。
- 有效性：已收到定位（`ullTimestampModule>0`）且最近 `timeout_ms` 内有新数据 → 有效；否则 `isValid()==false`。
- 短时丢失：`lastValidPose()` 提供上一帧有效位姿（降级时沿用）。
- 降级：**定位无效不退出**，HDMap 过滤自动关闭，低矮检测照常运行。
- 离线调试：`Localization.debugEnable=1` 时使用固定调试位姿（仅 pcap 回放验证用）。

### 定位数据来源（严格按项目 C 分析文档）

- 端口/消息：`FUSIONLOC_MC_PORT 9110` / `FUSIONLOC_MSGID 0x0a`（comm_2.7.1 `common_drv.h`）。
- 回调：`OpenfusionLocMCClient(9110, 0)` + `fusionLoc_set_callback(fusionLoc_set)`。
- 结论：与项目 A/C 订阅的是 **同一个 MC 总线消息**（同一外部定位发布模块）。

---

## 4. HDMap 如何加载

```
SutengDriver::Init()
   ↓ 启用条件：mapFilterModel!=0 && localizationEnable!=0 && !pcd模式
HDMapManager::loadMap(mapPath)
   ↓ 封装 read_hdmap::load(path)
MallocMap() -> InitMap(path) -> ReadMapFile(path)（fopen/fread 顺序二进制解析）
   ↓ 成功判定：iCountLane > 0
```

- 路径：默认 `/etc/echiev/hdmap/hdmap.bin`；通过 `HdmapFilter.mapPath` 配置；**mapPath 为空时回退默认地址**（已在 `ReadYamlFile.cpp` 实现）。
- 只加载一次（启动时），之后只读共享，无需加锁。
- 地图锚点从文件头读取（`strAnchorLonLat`），不硬编码。
- 加载失败 → 打印日志，`isLoaded()==false`，HDMap 过滤自动关闭，低矮检测不受影响。

---

## 5. CoordinateTransformer 如何工作

坐标系约定（与项目 A/C 一致，见设计文档第 13/15 章）：

| 坐标系 | X | Y | Z | 单位 | 说明 |
|--------|---|---|---|------|------|
| 车辆系 | 前向 | 左 | 上 | 米 | `PointCloudTransform` 之后 |
| 地图系 | 东 E | 北 N | 上 U | 米 | ENU，原点为地图锚点 |
| 航向 | — | — | — | 度 | 北=0，顺时针为正 |

公式（移植项目 A，与项目 C 数学等价，已单测验证互逆）：

```
vehicleToMap:  fx = -veh_y; fy = veh_x;
              map_x = cos(h)*fx + sin(h)*fy + car_x
              map_y = -sin(h)*fx + cos(h)*fy + car_y
              （h = heading_deg * PI/180）

mapToVehicle:  dx = map_x - car_x; dy = map_y - car_y
              θ = -(90 - heading_deg) * PI/180
              veh_x = cosθ*dx - sinθ*dy
              veh_y = sinθ*dx + cosθ*dy

lonLatToMap:   RTKLIB ENU（PointDeg2Enu，锚点=地图锚点）
```

单测 `test_coordinate_transform` 验证：多个航向/位置下 `vehicle→map→vehicle` 往返误差 < 1e-3 m（实测 0.000000），以及航向方向语义（h=0 车头朝北→前方变北；h=90 车头朝东→前方变东等）。

---

## 6. HDMapFilter 如何工作（Cluster 级软约束）

```
Cluster 列表
   ↓ 每个 cluster
cluster.center（车体系，优先 OBB 中心）
   ↓ CoordinateTransformer::vehicleToMap
地图坐标
   ↓ HDMapManager 点面测试（射线法 + 边界外扩 expandDistance）
判断是否位于可行驶（道路）区域
   ↓
设置 in_road / map_valid / map_confidence
   ↓
【软约束】保留 Cluster，不删除
```

- 可行驶区域多边形：车辆所在 Section 的左右边界线 + income/outgo Section 围成（地图系）。
- 点面测试：射线法（`pointInPolygon`）+ 距边界 `<= expandDistance` 视为"在道路"（防误删）。
- 标签规则：
  - `in_road=true` → `map_confidence=1.0`（道路内）
  - `in_road=false` → `map_confidence=0.3`（道路外，仅标记）
  - 定位无效 / 地图未加载 / 查询失败 → `map_valid=false`，**全部保留**
- **第一阶段不执行 `clusters.erase()`**。默认 `filterMode=0`（仅标记）；`filterMode=1`（软约束）仅用于降置信度。

### Cluster 过滤发生在哪里

`include/SuTengDriver.cpp` 的 `ProcessPcapCloud()` 中：

```cpp
ProcessWithObstacleDetection(...)      // → outputClusters
   ↓
if (m_hdmapEnabled) {
    getPose(loc_pose);
    m_hdmapFilter.filterClusters(outputClusters, loc_pose, m_hdmapManager, frame_id);
}
   ↓
ConvertClustersToTrackedObstacles(...)
   ↓
m_tracker.update(...)
   ↓
sendUdpMsg(...)
```

---

## 7. 配置项

`config/debug_config.yaml`（以及 E1/EMX）：

```yaml
# ========== Localization + HDMap（新增）==========
Localization:
  enable: 0        # 0=不订阅定位 1=订阅（需同时 mapFilterModel=1 且非pcd模式）
  timeout_ms: 1000 # 定位新鲜度阈值(ms)，超过判为无效，HDMap 过滤自动关闭
  debugEnable: 0   # 0=off 1=使用固定调试位姿（仅离线pcap回放验证用，勿在实车开启）
  debugX: 0.0      # 调试位姿 X（地图坐标 m）
  debugY: 0.0      # 调试位姿 Y（地图坐标 m）
  debugHeading: 0.0 # 调试航向（度，北=0 顺时针）

HdmapFilter:
  mapFilterModel: 0    # 0=关闭 HDMap 过滤 1=开启
  mapPath: ""          # 为空则使用默认 /etc/echiev/hdmap/hdmap.bin
  filterMode: 0        # 0=仅标记(默认，不删除Cluster) 1=软约束(降低置信度)
  expandDistance: 0.5  # 道路边界外扩距离(m)，防误删
  logEveryN: 50        # 每 N 帧打印一次 cluster 级地图过滤日志
```

开关关系（用户要求，已实现）：

```text
mapFilterModel = 0       → HDMap 不初始化、不过滤、不改 Cluster/Tracker
Localization.enable = 0  → 不订阅定位，HDMap 过滤自动不可用
pcd 模式                 → 不启用 HDMap 过滤（单帧离线调试）
定位订阅失败              → 降级：不过滤，低矮检测照常运行
```

---

## 8. 编译方式

```bash
cd /home/zyl/echiev_low_lidar_detection
cmake -S . -B build          # 首次或 CMakeLists 变更后
cmake --build build -j4      # 编译 low_detection 和 test_coordinate_transform
```

新增源码已纳入 `CMakeLists.txt` 的 `MAPFILTER_SRC`；坐标转换单测为独立目标 `test_coordinate_transform`。

---

## 9. 运行方式

```bash
# 1) HDMap 关闭（保持原有行为）
./build/low_detection -f config/debug_config.yaml

# 2) HDMap 开启 + 真实定位（实车/有 MC 总线环境）
./build/low_detection -f config/debug_config_EMX.yaml

# 3) HDMap 开启 + 离线调试固定位姿（pcap 回放验证过滤链路）
#    需在 yaml 中设 Localization.debugEnable=1, debugX/debugY/debugHeading
./build/low_detection -f /path/to/test.yaml
```

---

## 10. 日志示例

### 场景 1：HDMap 开启 + 定位有效

```text
[Localization] enabled (timeout=1000 ms)
[Localization] initialized
[HDMap] loaded: /home/zyl/echiev_low_lidar_detection/config/hdmap_shenzhen.bin
[HDMap] map objects: lanes=10 sections=10 roads=3 junctions=1
[HDMapFilter] configured: enabled=1 mode=cluster_mark expand=0.50m logEveryN=50
[HDMap] enabled (mapPath=...)
[Localization] valid x=-5.880 y=12.210 yaw=0.00 timestamp=0
[HDMapFilter] frame=1787217411182 cluster=0 center_veh=(3.45,-3.68) center_map=(-2.205,15.660) in_road=1 map_valid=1 confidence=1.00
[HDMapFilter] frame=1787217411182 cluster=4 center_veh=(7.18,-3.32) center_map=(-2.563,19.393) in_road=0 map_valid=1 confidence=0.30
```

（`frame_id / cluster_id / center(vehicle) / center(map) / in_road / map_valid / confidence` 已按要求输出，每 `logEveryN` 帧一次。）

### 场景 2：定位无效（降级）

```text
[Localization] enabled (timeout=1000 ms)
[HDMap] loaded: ...
[HDMap] map objects: ...
[HDMapFilter] configured: enabled=1 ...
[HDMap] enabled ...
[HDMapFilter] disabled: localization invalid
```

### 场景 3：HDMap 关闭

```text
[Localization] disabled
（无 [HDMap]/[HDMapFilter] 日志，低矮检测照常运行）
```

---

## 11. 已验证功能

1. **编译**：`cmake --build build` 成功，`low_detection` 与 `test_coordinate_transform` 均构建通过；新增/修改的 mapfilter 文件无编译警告。
2. **坐标转换单测**：`test_coordinate_transform` **ALL PASS**，vehicle→map→vehicle 往返误差 < 1e-3 m。
3. **HDMap 加载**：`hdmap_shenzhen.bin` 成功加载（lanes=10, sections=10, roads=3, junctions=1），锚点从文件头读取。
4. **HDMap 查询/点面测试**：离线验证车辆所在 Section 定位、可行驶区域多边形、射线法点面测试均正确。
5. **Cluster 级过滤链路**：debug 位姿模式下，Cluster 中心 → vehicleToMap → 点面测试 → `in_road/map_valid/confidence` 标签正确输出，**且所有 Cluster 均被保留（软约束）**。
6. **降级链路**：定位无效（本机无 MC 总线）时 `[HDMapFilter] disabled: localization invalid`，低矮检测（Loop Start / Track id / UDP 逻辑）照常运行，程序不退出。
7. **关闭链路**：`mapFilterModel=0` 时无 HDMap 初始化、无过滤、Cluster/Tracker 行为与修改前一致。

---

## 12. 尚未验证功能

1. **真实定位接收**：本开发机为 x86，`libcomm.so`（aarch64 版）`OpenfusionLocMCClient` 创建线程失败，无法收到真实 MC 0x0a 数据。需在实车/有 MC 总线的环境验证：
   - `Localization callback received`
   - 定位 x/y/yaw 随车辆移动更新
2. **定位与地图对齐精度**：hdmap_shenzhen.bin 的锚点（lon=113.871, lat=22.592）与实际场地是否匹配未验证（设计文档标注 `[待确认]`）。
3. **cluster 级过滤在真实定位下的效果**：仅用离线调试位姿验证了代码链路，未做真实数据下的 A/B 对比（关/开地图的检出率对比）。
4. **性能**：未统计地图查询/多边形构建/过滤每帧耗时（Cluster 级开销极小，理论可忽略）。

---

## 13. 已知问题

1. **`OpenfusionLocMCClient` 在 x86 开发机失败**：`libcomm.so` 为 aarch64 版，`pthread_create` 失败。已实现降级（不退出），实车上应正常。
2. **地图锚点匹配**：`hdmap_shenzhen.bin` 与部署场地地图需现场确认（设计文档 `[待确认]`）。
3. **`GetLaneIdByPoint` 为全图遍历**（移植自项目 A）：`O(车道数 × 采样点数)`，Cluster 级过滤每帧仅调用一次（构建多边形时），开销可接受；后续如需可加空间索引（设计文档第 26 章）。
4. **`HdmapFilter.logEveryN` 日志**：每 N 帧打印一次 cluster 级日志，日志量受 cluster 数量影响，调大 `logEveryN` 可降低。
5. **原有代码的编译警告**（`main.cpp`/`LidarCurbDetection.h`/`SuTengDriver.cpp` 等）为修改前已存在，本次未触碰。

---

## 14. 后续 Grid/PointCloud 过滤计划

第一阶段只做了 **Cluster 级软约束**。后续阶段（设计文档第 18 章）建议：

- **阶段 2（Grid 级）**：在 `ClusterObstacleGrid` 之前，把"地图道路区域外"的 obstacle cell 的 `is_obstacle_candidate` 置 false（复用本项目 Grid 结构，增加 `is_in_drivable_area` 标志；多边形每帧栅格化一次）。
- **阶段 3（PointCloud 级）**：迁移项目 A 的 `HdmapROIFilter`（Bitmap2D 栅格 + 逐点测试），需一并迁移 `bitmap2d`/`polygon_mask` 与 `get_roi.cpp`（当前未纳入编译）。
- 无论哪一级，都必须保持：**地图失败/定位无效 → 不过滤**；默认软约束 + 较大 `expandDistance`；**不以"道路=无障碍"删除道路上目标**。

---

## 15. 代码修改前后数据流

### 修改前

```text
PointCloud
 ↓
Transform (车辆系 x前 y左)
 ↓
Ground Filter
 ↓
Grid
 ↓
Cluster
 ↓
Tracker
 ↓
UDP
```

### 修改后

```text
Localization (MC 0x0a @ 9110)
      │
      ↓
STR_FUSIONLOC
      │
      ↓
LocalizationManager
      │
      │      HDMap ──→ HDMapManager (loadMap)
      │                     │
      │                     ↓
      │            CoordinateTransformer
      │                     │
      ↓                     ↓
PointCloud
 ↓
Transform (车辆系 x前 y左)
 ↓
Ground Filter
 ↓
Grid
 ↓
Cluster
 ↓
HDMapFilter.filterClusters()
 │   ├── 定位+地图有效 → 打标签 in_road / map_valid / confidence
 │   └── 定位无效 / 地图未加载 / 查询失败 → 全部保留（map_valid=false）
 ↓
ConvertClustersToTrackedObstacles
 ↓
Tracker
 ↓
UDP
```

---

## 16. 最终原则落实情况

```text
定位 ≠ 队列            → 项目 B 作为独立进程订阅 MC 0x0a @ 9110（与 A/C 同一数据源）
队列 ≠ 定位源          → 未复用 A/C 的进程内队列，仅用"最新值 + 新鲜度"方式消费
HDMap ≠ 障碍物真值     → 只作"额外空间先验"
道路区域 ≠ 没有障碍物   → 低矮障碍物在道路内仍保留（软约束）
地图查询失败 ≠ 删除障碍物 → 失败时全部保留
定位失败 ≠ 低矮检测退出 → 降级为关闭 HDMap 过滤，检测照常运行
```
