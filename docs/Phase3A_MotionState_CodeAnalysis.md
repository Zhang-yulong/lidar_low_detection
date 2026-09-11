# Phase 3-A 代码分析：Motion State（UNKNOWN / STATIC / MOVING）

> 日期：2026-09-10
> 性质：**实施前代码检查 + 实现方案**（本文件为改动前的分析；实现见文末“修改位置”）
> 目标：只建立 `Map position history → Motion Evidence → UNKNOWN/STATIC/MOVING`，为 Phase 3-B 提供状态基础。
> 明确不做：Historical Geometry / cell fusion / OBB 重算 / static coast / 修改 Phase 2 关联。

---

## 1. 现有结构（已逐行核实）

### 1.1 调用链（`SutengDriver::ProcessPcapCloud()`）

```
stuffed_cloud_queue.popWait()                              // 帧 1 跳过
  ↓
PointCloudTransform(...) → pFilteredPointCloud
  ↓
ElevationMapGroundFilter::ProcessWithObstacleDetection(...) → outputClusters
  ↓
LocalizationManager::instance().getPose(loc_pose); have_pose = got_pose && loc_pose.valid
  ↓
[HDMap] m_hdmapFilter.filterClusters(...)                  // 只打标签
  ↓
m_debugViewer->DrawClusterOverlay(...)
  ↓
{ ConvertClustersToTrackedObstacles(outputClusters, detections);
  m_tracker.update(detections, rec_timestamp_ms);           // ← Tracker
  for (track : m_tracker.vtrackings) LOG_RAW("Track id: ...") }
  ↓
{ ComputeHistoricalFeedback(...)                            // ← Phase 2（不改）
  DrawHistoricalFeedbackOverlay(...)
  UpdateMapAnchors(loc_pose, have_pose); }                  // ← 帧末维护锚点
  ↓
DrawMapAndAllOverlay(...) / DrawTrackOverlay(...)
  ↓
g_previousTimestamp.store(rec_timestamp_ms)                 // ← 帧末更新全局时间戳
  ↓
[onlineModel] ConvertTrackToS2ObstacleBox(vtrackings) → sendUdpMsg
```

**关键**：`g_previousTimestamp` 在**帧末**更新，因此在 `m_tracker.update()` 之后、帧末之前，它仍是**上一帧**时间戳 → 可直接用于计算 `dt`。

### 1.2 `TrackedObstacle`（`include/track.h`）

| 字段 | 语义 | 备注 |
|---|---|---|
| `cluster_id` | **诞生时**的簇 ID | ⚠️ 匹配成功时**不回写** |
| `id` | 稳定跟踪 ID（9999 起） | |
| `pos_x/pos_y/pos_z` | **主雷达系**（x 前, y 左）中心 | |
| `corners[4]` | 主雷达系 4 角点 | 匹配时同步更新 |
| `depth/width/height` | 尺寸 | `depth=obb_length`,`width=obb_width` |
| `age` | **匹配成功帧计数**（匹配 +1；miss 不变） | 语义必须保持不变 |
| `lastSeen` | **连续 miss 计数**（匹配归 0；miss +1） | |
| `vx/vy` | **主雷达系**两点差分速度（m/s） | 见 §1.5 |
| `Translation/Rotation` | 预留 | `ConvertClustersToTrackedObstacles` 里被 memset 为 0 |
| `kf` | `unique_ptr<KalmanFilter2D>` | 未使用（已注释） |
| **无** `map_x/map_y/yaw/motion_state` | — | Phase 3-A 需新增 |

⚠️ **`TrackedObstacle` 的拷贝构造 / 拷贝赋值是手写的**（逐字段拷贝 + 深拷贝 `kf`）。新增字段**必须同步到这两处**，否则在 `vtrackings.push_back(...)` / `vector` 扩容拷贝时丢失。

### 1.3 `SimpleTracker::update()` 流程（`include/track.cpp`）

```
dt = (time - g_previousTimestamp)/1000
if (detections.empty()):
    for each track: pos += vx*dt; corners += vx*dt; lastSeen++;
    removeLostTargets(); return;
else:
    predicted = track.pos + track.v*dt          // 仅用于代价
    cost[t][d] = |predicted - det|              // 雷达系最近邻
    贪心匹配 (match_threshold = 0.5m)
    matched:
        pos = det.pos                            // 直接赋值（不滤波）
        vx = (pos - old)/dt ; vy = (pos - old)/dt // ⚠️ 雷达系两点差分
        depth/width/height/corners = det.*
        age++; lastSeen = 0;
    unmatched track: lastSeen++
    unmatched detection: 新建 track（age=0,lastSeen=0,id=nextTrackID++）
    removeLostTargets()
```

- **不修改**匹配阈值 / 代价 / 预测 / 赋值 / 出生 / 删除。
- `removeLostTargets()`：实测 `t.lastSeen > 5`（连续 6 帧 miss 删除）。
- **本阶段不在 tracker 内做 motion**，而是在 `update()` 之后新增独立步骤（不改变 tracker 行为与签名）。

### 1.4 `m_tracker.vtrackings`

- 类型：`std::vector<TrackedObstacle>`（`SimpleTracker` 公有成员）。
- 每帧被 `update()` 就地改写；跨帧累积；`removeLostTargets()` 删除超期目标。
- 现有消费方：`DrawMapAndAllOverlay` / `DrawTrackOverlay` / `ConvertTrackToS2ObstacleBox` / `m_visBuffer.Publish` / `UpdateMapAnchors` / `ComputeHistoricalFeedback`。

### 1.5 `vx/vy` 的真实坐标系语义（重要）

- 来源：`update()` 匹配时 `vx = (新pos_x - 旧pos_x)/dt` → **主雷达系（vehicle/radar frame）** 速度（m/s）。
- 消费方：
  1. `SimpleTracker::update()` 的 `detections.empty()` 分支：`pos_x += vx*dt`（雷达系外推）；
  2. `DebugViewer::DrawMapAndAllOverlay()`：青色速度箭头 `(pos_x + vx, pos_y + vy)`（雷达系）。
- 结论：**`vx/vy` 是雷达系速度，不能改写为地图系速度**。Phase 3-A 新增独立字段 `map_vx/map_vy`（地图系，m/s），原有 `vx/vy` 语义与消费者全部保持不变。

### 1.6 Localization / 坐标转换调用方式（复用，不重实现）

```cpp
// 每帧已读取（ProcessPcapCloud 内）
bool got_pose = LocalizationManager::instance().getPose(loc_pose);
have_pose = got_pose && loc_pose.valid;              // loc_pose: {x,y,heading_deg,timestamp_ms,valid}

// 地图系转换（纯计算，内部已含 s_grid_heading_offset_deg 补偿）
VehiclePose vpose; vpose.x = loc_pose.x; vpose.y = loc_pose.y; vpose.heading_deg = loc_pose.heading_deg;
CoordinateTransformer::vehicleToMap(veh_x, veh_y, vpose, map_x, map_y);
```

**不得**重复添加 `s_grid_heading_offset_deg`，**不得**重新实现 Localization。

### 1.7 DebugViewer 当前读取的字段

- `DrawMapAndAllOverlay()`（`DrawTrackOverlay()` 直接转发到它）读取：`corners[4]`（白色多边形）、`pos_x/pos_y`（黄色中心）、`id`（标签）、**`vx/vy`（青色箭头）**。
- 因此 Motion State 只要写进 `vtrackings`，即可被该函数读取；本阶段只增加**很小的**文字标签（不改结构、不重构）。

---

## 2. 准备修改的位置

| 文件 | 改动 | 目的 |
|---|---|---|
| **新增** `include/track_motion_state.h` | `enum class MotionState`、阈值常量、`MotionStateName()` | 状态与参数集中，header-only，零依赖 |
| `include/track.h` | include 新头；`TrackedObstacle` 新增字段；**手写拷贝构造/赋值同步** | 让 `m_tracker.vtrackings` 携带 motion 结果 |
| `include/SuTengDriver.h` | 声明 `UpdateTrackMotionStates(pose, pose_valid, rec_timestamp_ms)` | 独立后置步骤 |
| `include/SuTengDriver.cpp` | 实现 + 在 `m_tracker.update()` 之后调用 | 维护 map history / 证据 / 状态 / 日志 |
| `include/DebugViewer.cpp` | `DrawMapAndAllOverlay()` 增加 1 行小标签（state + map 速度） | 可视化观察 |

**不改**：`track.cpp`（`SimpleTracker::update` / `removeLostTargets`）、`ElevationMapGroundFilter.*`、`BuildGrid/Cluster/OBB`、`PointCloudTransform`、`HDMap`、`LocalizationManager`、`CoordinateTransformer`、UDP protocol、Phase 2 `ComputeHistoricalFeedback` / 3×3 Search。

> 说明：不改 `SimpleTracker::update()` 的原因——保持 tracker 匹配/删除行为完全不变（保护 Phase 2），且避免修改其函数签名（需要额外传入 pose）。motion 作为 `update()` 之后的一步，输入来自 `vtrackings` + 已有 `loc_pose`，等价且更安全。

---

## 3. 新增字段（`TrackedObstacle`）

```cpp
// ---- Phase 3-A: Motion State (map-frame) ----
MotionState motion_state   = MotionState::UNKNOWN;
bool  has_map_pos          = false;   // 是否已有有效 matched map position
float map_x = 0.0f, map_y = 0.0f;     // 最近一次有效 map position
float map_vx = 0.0f, map_vy = 0.0f;   // 地图系速度 (m/s, EMA)
float motion_step = 0.0f;             // 最近一次地图系帧间位移 (m)
float motion_net  = 0.0f;             // 窗口内净位移 (m)
float motion_dir_deg = 0.0f;          // 最近一次位移方向 (deg)
int   motion_static_run = 0;          // 连续静止证据计数
int   motion_moving_run = 0;          // 连续运动证据计数
std::vector<Point2D> recent_map_positions;  // 最近 8 个有效 matched map position
```

- `Point2D` 已存在（`Type.h`，`float x,y`）。
- `age / lastSeen / vx / vy` **语义不变**，不新增 `coast_frames` / `track_lifetime`。

---

## 4. Motion Evidence 计算

仅在 **`lastSeen==0`（匹配成功）且 `pose.valid==true`** 时产生新观测：

```
P_map(k) = vehicleToMap(track.pos_x, track.pos_y, pose(k))
step_vec = P_map(k) - P_map(k-1)
step     = |step_vec|
dir      = atan2(step_vec.y, step_vec.x)
net      = |P_map(k) - P_map(k-win)|,  win = min(MOVE_WIN, len(history)-1)
map_v    = EMA(step_vec / dt, alpha=V_EMA_ALPHA)
```

- `dt = (rec_timestamp_ms - g_previousTimestamp)/1000`（与 tracker 同源，帧末才更新）。
- 漏检帧：**不追加** history、**不伪造**位置、**不更新** step/velocity/计数器；`motion_state` 保持不变。
- `miss ≠ static evidence`。

---

## 5. 状态机（迟滞）

```
每匹配帧：
  moving_ok = (step >= MOVE_STEP_MIN) && dir_consist && (net >= MOVE_NET_MIN)
  static_ok = (step <= STATIC_STEP_MAX)

  motion_moving_run = moving_ok ? motion_moving_run+1 : 0
  motion_static_run = static_ok ? motion_static_run+1 : 0

  UNKNOWN: moving_run >= M_MOVE            → MOVING
           else static_run >= K_UNKNOWN_TO_STATIC → STATIC
  STATIC : moving_run >= M_MOVE            → MOVING
  MOVING : static_run >= K_MOVING_TO_STATIC → STATIC
```

- `dir_consist`：最近 `M_MOVE` 步的每一步方向与“窗口净位移方向”夹角均 ≤ `DIR_TOL`（需要 `len(history) >= M_MOVE+1`）。
- **死区**：`STATIC_STEP_MAX < step < MOVE_STEP_MIN` 时两个计数都清零、状态保持 → 天然迟滞。
- `+0.1/-0.1` 来回：`net≈0`、方向翻转 → 不会判 MOVING。
- 单帧大步长：不足 `M_MOVE` 连续 → 不会判 MOVING。

---

## 6. 阈值（可配置常量，语义对应 §十一）

| 常量 | 值 | 控制 |
|---|---|---|
| `kMotionStaticStepMax` | 0.08 m | 明显静止抖动上限 |
| `kMotionMoveStepMin` | 0.12 m | 运动 dead-band 上限（> 0.1 m Grid） |
| `kMotionMoveNetMin` | 0.30 m | 累计运动距离 |
| `kMotionMoveWin` | 3 | 观察窗口 |
| `kMotionMMove` | 3 | 连续运动证据 |
| `kMotionDirTolDeg` | 60° | 方向一致性 |
| `kMotionUnknownToStatic` | 4 | 进入 STATIC |
| `kMotionMovingToStatic` | 6 | 退出 MOVING（迟滞） |
| `kMotionVEmaAlpha` | 0.3 | 速度平滑 |
| `kMotionHistoryCap` | 8 | history 容量 |

---

## 7. Map position history

- `recent_map_positions`：由 `TrackedObstacle` 自带（随 track 生命周期，无需额外侧表 / 无需手工清理）。
- 容量 8；超出时删除最旧。
- 仅 matched + pose valid 时追加。

---

## 8. 为什么不会影响 Phase 2

1. `ComputeHistoricalFeedback()` / 3×3 Search / `SAME_TRACK_ID` / `GEOMETRIC` **零改动**。
2. `UpdateMapAnchors()` 零改动：其只读 `t.id / t.pos_* / t.corners / t.lastSeen / t.age`，新增字段不影响。
3. `SimpleTracker::update()` / `removeLostTargets()` 零改动：motion 是 `update()` **之后**的独立步骤。
4. 新增的 `map_x/map_y` 是**新字段**，不复用/覆盖 `pos_x/pos_y`；`vx/vy` 语义不变。
5. Phase 2 的 `anchor` 侧表与 motion 字段互不干扰（motion 只读 pose + track pos 并写新字段）。

## 9. 为什么不会影响 Detection / Cluster / OBB

- 未触碰 `BuildGrid / ClusterObstacleGrid / BuildClusterFromCells / ComputeClusterOBB`；
- 未重算 `yaw / length / width / corners`；
- 未修改 `PointCloudTransform` / Ground Filter / HDMap / Localization / UDP。

---

## 10. 日志方案

每帧每 track（matched 与 miss 都输出，便于判断“为什么”判成该状态）：

```
[MotionState] Track=10011 age=5 lastSeen=0 map=(10.02,2.01) step=0.041 net=0.072 dir=31.2 state=STATIC v=(0.02,0.01) obs=1 hist=5 srun=7 mrun=0
[MotionState] Track=10012 age=8 lastSeen=0 map=(12.31,1.95) step=0.170 net=0.480 dir=18.6 state=MOVING v=(1.42,-0.31) obs=1 hist=4 srun=0 mrun=3
```

字段：`step / net / dir / state / v / obs(本帧是否新观测) / hist(history长度) / srun / mrun`。

## 11. 可视化方案（最小）

`DrawMapAndAllOverlay()` 在 ID 标签下方加一行小字：

```
STATIC  v=(0.02,0.01)m/s      // 绿色
MOVING  v=(1.42,-0.31)m/s      // 红色
UNKNOWN v=(0.00,0.00)m/s      // 灰色
```

原有青色速度箭头（雷达系 `vx/vy`）保持不变。

---

## 12. 验收对照（§二十六）

| # | 验收项 | 本方案 |
|---|---|---|
| 1 | 基于 Map position | ✅ `vehicleToMap` |
| 2 | age 语义不变 | ✅ 未改 tracker |
| 3 | 新 track 从 UNKNOWN | ✅ `motion_state` 默认 UNKNOWN |
| 4 | 静止目标稳定（容忍 0.1m jitter） | ✅ STATIC_STEP_MAX=0.08 + net/dir 双门限 |
| 5 | MOVING 需持续证据 | ✅ M_MOVE=3 + net≥0.30 + dir 一致 |
| 6 | 状态迟滞 | ✅ 0.08/0.12 死区 + 4/6 非对称计数 |
| 7 | vtrackings 含结果 | ✅ 新增字段 |
| 8 | Phase 2 不变 | ✅ 零改动 |
| 9 | 无 Historical Geometry | ✅ 未实现 |
| 10 | 无核心检测改动 | ✅ 未触碰 |

---

## 13. 实现完成情况（代码已实施）

### 13.1 实际修改文件

| 文件 | 改动 |
|---|---|
| **新增** `include/track_motion_state.h` | `enum class MotionState`、阈值常量、`MotionStateName()` |
| `include/track.h` | include 新头；`TrackedObstacle` 新增 12 个字段（`motion_state/has_map_pos/map_x/map_y/map_vx/map_vy/motion_step/motion_net/motion_dir_deg/motion_static_run/motion_moving_run/recent_map_positions`）；**拷贝构造 + 拷贝赋值同步** |
| `include/SuTengDriver.h` | 声明 `UpdateTrackMotionStates(pose, pose_valid, rec_timestamp_ms)` |
| `include/SuTengDriver.cpp` | 实现 `UpdateTrackMotionStates()`；在 `m_tracker.update()` 之后调用；新增 `[MotionState]` 日志 |
| `include/DebugViewer.cpp` | `DrawMapAndAllOverlay()` 增加 1 行状态小标签（state + map 速度） |

**未修改**：`track.cpp`（`SimpleTracker::update` / `removeLostTargets`）、`ElevationMapGroundFilter.*`、`PointCloudTransform`、`HDMap`、`LocalizationManager`、`CoordinateTransformer`、UDP、Phase 2 `ComputeHistoricalFeedback` / 3×3 Search。

### 13.2 编译结果

`cmake --build build -j4` → **`[100%] Built target low_detection`**（成功，无 error）。

### 13.3 运行验证状态

- 已**编译验证**通过。
- **运行验证未执行**：Motion State 依赖 `loc_pose.valid`（地图系判断）。离线 pcap 回放需要**与 pcap 时间同步的有效定位**；`config/debug_config_EMX.yaml` 的 `Localization.enable=1 / mapFilterModel=1 / debugEnable=0` 依赖在线组播定位。若使用 `debugEnable=1` 固定位姿，则自车运动不被补偿，**不适用于运动状态验证**（会把自车运动误算成目标运动）。
- 真实移动目标数据：**未确认**（仓库有多个 pcap，但未确认包含“真实移动低矮目标”且具备同步定位）。
- 详见 `docs/Phase3A_MotionState_CodeAnalysis.md` §12 与最终回复的验证说明。

### 13.4 已知调参风险

- 若静止目标在 **Map 系**的帧间抖动常落在死区 `(0.08, 0.12)`，`motion_static_run` 会频繁清零，导致长时间停在 UNKNOWN。此时应把 `kMotionStaticStepMax` 调至 0.10（并相应提高 `kMotionMoveStepMin`），或确认 map 抖动来源（定位噪声 / 关联抖动）。
- 验证时应先看 `[MotionState] ... obs=1 hist=...` 是否有值（`obs=0` 表示该帧无有效定位或无匹配 → 不会产生运动证据）。

