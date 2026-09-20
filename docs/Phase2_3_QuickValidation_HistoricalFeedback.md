# Phase 2/3 Quick Validation —— Historical Feedback 实验报告

> 日期：2026-09-08
> 性质：**Quick Validation / Experimental Validation**（非正式 Phase 3）
> 前置设计文档：`docs/MapFrameHistoricalFeedbackRawPointImage_FinalArchitecture.md`（只读，未修改）

本报告验证一个更基础的问题：

> **Historical Track 能否补偿当前帧 Current GridCluster 因 LiDAR 观察角度变化造成的缺失区域？**

即验证 `Previous Track → Map Frame → Current Radar Frame → Current Grid → Historical Feedback Region` 这一最基本闭环，而不是 Raw Point Image / 5cm Image / Historical Contour 等正式 Phase 1/3 内容。

---

## 0. 结论速览

1. **当前代码的 Phase 2（Map-frame Stable Track）实际尚未实现。** `TrackedObstacle` 没有 `map_x/map_y`、没有 `stable_yaw`、没有 `motion_state`。当前 `SimpleTracker` 是纯雷达系贪心匹配，位置直接每帧被检测值覆盖。
2. 因此本次验证通过一个**外部侧表 `MapAnchoredTrack`**（不修改 `TrackedObstacle`、不重写 Tracker）等价实现了最小版"地图系锚点"：每帧 Track 匹配成功时用当前位姿把雷达系位置/角点锚定到地图系，miss 时冻结锚点，删除时同步移除。
3. 已实现完整闭环并通过编译（`[100%] Built target low_detection`）。
4. 正式 Phase 3 仍依赖 Raw Point Image 二次验证；本次只产出**空间先验 + Fused Debug Region**，不修改 `detections` / `m_tracker.update` 输入 / 最终 UDP 输出。

---

## 1. 当前真实数据流（已从代码逐行确认）

生产路径唯一入口 `SutengDriver::ProcessPcapCloud()`（`include/SuTengDriver.cpp`）：

```
stuffed_cloud_queue.popWait()
   │  Frame 1 (GetFrameCountValue()==1) → 存 g_previousTimestamp 后 continue 跳过
   ▼
PointCloudTransform(pInputCloud, R_CombinedTransMatrix, pFilteredPointCloud)
   │  R_Combined = R_X_Exchange_Y * (toCarInfo * toMainLidarInfo)   → 雷达系→车体系(x前,y左,z上)
   ▼
m_pElevationMapGroundFilter->ProcessWithObstacleDetection(pFilteredPointCloud, ...)
   │  BuildGrid → ComputeGroundHeight → ComputeSlope → RegionGrowing
   │  → GenerateGroundMask → ComputeGroundReference → AnalyzeVerticalOccupancy
   │  → ClusterObstacleGrid → std::vector<GridCluster> outputClusters
   ▼
【m_hdmapEnabled】LocalizationManager::getPose(loc_pose) → buildDrivablePolygons
   → m_hdmapFilter.filterClusters(outputClusters, ...)   (只打 in_road/map_valid 标签)
   ▼
ConvertClustersToTrackedObstacles(outputClusters, detections)
   │  ⚠️ 硬过滤 if(cluster.in_road || !m_hdmapEnabled) 才入 detections
   ▼
m_tracker.update(detections, rec_timestamp_ms)      (V1 贪心匹配, ID 从 9999 起)
   ▼
【新增】ComputeHistoricalFeedback + DrawHistoricalFeedbackOverlay + UpdateMapAnchors
   ▼
m_debugViewer->DrawTrackOverlay / DrawMapAndAllOverlay
   ▼
【onlineModel】ConvertTrackToS2ObstacleBox → sendUdpMsg
```

关键事实：
- 第一帧点云被显式跳过。
- `ProcessWithObstacleTracking()` 内部的 `tracker_` 不是生产路径；生产路径用 `SutengDriver` 自己的 `m_tracker`（`SimpleTracker`）。
- 定位原本只在 `if (m_hdmapEnabled)` 内读取。**本次已把定位读取提升到每帧统一读取一次**（独立于 HDMap 开关），为 Historical Feedback 服务（对应架构 §2 要求）。

### 1.1 EMX 配置下 Grid 真实几何

| 参数 | 值 |
|---|---|
| roi_x_min / roi_x_max | 0.0 / 20.0 |
| roi_y_min / roi_y_max | -4.0 / 4.0 |
| grid_resolution | 0.1 m |
| car_half_x | 0.8 |
| body_filter_x_threshold | 2.5 |
| **m_effective_roi_x_min** | **max(0.0, 0.8+2.5) = 3.3 m** |
| cols / rows / cells | 167 / 80 / 13,360 |
| near_range_boundary | 3.0（< 3.3 → 近场阈值实际永不生效，全部走 far 分支）|

---

## 2. 当前 Track 数据结构（TrackedObstacle）盘点

`include/track.h`：

| 字段 | 是否存在 | 语义 | 是否用于本次 Historical Feedback |
|---|---|---|---|
| `id` | ✅ | 跟踪 ID（9999 起） | ✅（track_id 标签/日志） |
| `cluster_id` | ✅ | 本帧对应簇 ID | ❌ |
| `pos_x / pos_y / pos_z` | ✅ | 雷达系中心 | ✅（锚点建立时读取） |
| `depth`（=length, x 方向）/ `width`（y 方向） | ✅ | 尺寸 | ✅（OBB 长宽） |
| `height` | ✅ | 高度 | ❌ |
| `corners[4]` | ✅ | 雷达系 4 角点 | ✅（转地图系重建 OBB 区域） |
| `vx / vy` | ✅ | 雷达系速度（两点差分） | ❌ 本次不用（避免引入运动模型） |
| `age` | ✅ | 存活帧数（匹配帧计数） | ✅（`kMinTrackAgeForFeedback=3` 门控） |
| `lastSeen` | ✅ | 丢失帧计数 | ✅（锚点 miss 冻结依据） |
| `map_x / map_y` | ❌ **缺失** | — | 由外部 `MapAnchoredTrack` 侧表补齐 |
| `stable_yaw` | ❌ **缺失** | — | 由 corners 隐含，未单独维护 |
| `motion_state` | ❌ **缺失** | — | 本次未实现，见风险 §7 |

结论：**Phase 2 未实现。** 本次用 `MapAnchoredTrack` 侧表做最小等价补丁，不修改 `TrackedObstacle`。

---

## 3. 坐标系分析

| 坐标系 | 定义 | 来源 |
|---|---|---|
| 雷达系 / 车体系（Grid / Cluster / Track 所在系） | x=前向, y=左向, z=上 | `PointCloudTransform` 输出 |
| Grid 索引 | `row = floor((y - roi_y_min)/res)`，`col = floor((x - effective_roi_x_min)/res)`；`idx = row*cols + col` | `WorldToGrid` |
| Grid cell 中心 | `cx = effective_roi_x_min + (col+0.5)*res`，`cy = roi_y_min + (row+0.5)*res` | `GridIndexToWorld` |
| 地图系 | ENU（x=东, y=北, 米），heading 度（北=0 顺时针） | `LocalizationManager::Pose` |
| `vehicleToMap` | `P_map = R(heading)·[-y_veh; x_veh] + P_car`（含 `gridHeadingOffsetDeg` 补偿） | `coordinate_transformer.cpp` |
| `mapToVehicle` | `vehicleToMap` 的严格逆变换 | 同上 |

**确认结论**：
1. Grid x/y 原点：Grid 从 `x = effective_roi_x_min`（EMX=3.3m）开始，`y = roi_y_min`（-4.0m）开始；车体原点 (0,0) 在 Grid 盲区内（车前 3.3m 之外）。
2. Grid resolution = **0.1m**（EMX）。
3. Grid cell center 见上表（`+0.5` 偏移）。
4. Grid / `GridCluster` / `TrackedObstacle.pos_*` 都是**雷达系（车体系）**，不是地图系。
5. Track 的 `pos_x/pos_y` = 雷达系中心（OBB 中心或 AABB 中心）。
6. Track **没有** yaw 字段；朝向由 `corners[4]` 隐含（OBB 角点）。
7. `map_x/map_y` **当前不存在**，由本次 `MapAnchoredTrack` 侧表补齐。
8. `mapToVehicle(map_x, map_y, pose, veh_x, veh_y)`：输入地图系坐标 + 车辆地图位姿，输出主雷达系坐标。
9. 当前 LiDAR 点云与 vehicle frame：`PointCloudTransform` 已统一到车体系，Grid 与 Track 同系，无需额外外参。
10. 安装外参旋转：`CoordinateTransformer` 内部统一应用 `s_grid_heading_offset_deg`（=90°+fLidar2Vehicle_Heading），本次无需额外处理。

---

## 4. 实现方案

### 4.1 数据流（本次新增，Debug/Experimental Path）

```
Current LiDAR
   ▼
ElevationMapGroundFilter (BuildGrid 未改) → outputClusters (雷达系)
   ▼
ConvertClustersToTrackedObstacles → m_tracker.update()  (原有行为不变)
   │
   ├── ComputeHistoricalFeedback()
   │     m_mapTracks(上一帧锚点) + 当前 loc_pose.valid
   │        → mapToVehicle(anchor.map_x/y) → 当前雷达系中心/角点
   │        → RasterizeQuadToGrid() → HistoricalFeedbackRegion.projected_cell_indices
   │        → overlap / fused 统计 + LOG_RAW
   │
   ├── DrawHistoricalFeedbackOverlay()   (第十层 Debug 图)
   │
   └── UpdateMapAnchors()                (每帧结束维护地图锚点侧表)
```

### 4.2 关键实现点

- **地图锚点侧表** `MapAnchoredTrack`（`include/historical_feedback.h`）：
  - Track 匹配成功（`lastSeen==0`）→ 用当前位姿 `vehicleToMap` 锚定中心 + 4 角点。
  - Track miss → 冻结锚点（保留最后一次地图位置）。
  - Track 被 `removeLongLostTargets` 删除 → 同步删除锚点。
- **投影**：`mapToVehicle(anchor.map_x/y, current_pose)` → 当前雷达系；4 角点同法投影。
- **栅格化**：`RasterizeQuadToGrid` 对投影后的凸四边形做 cell 中心采样点-in-多边形判定。
- **年龄门控**：`anchor.age < kMinTrackAgeForFeedback(=3)` 的 Track 不产生反馈。
- **定位硬门控**：`pose.valid==false` → 直接禁用（不打 mapToVehicle），锚点冻结。

### 4.3 与当前正式检测的隔离（强制项）

- `HistoricalFeedbackRegion` 只进 DebugViewer + 日志，**不进入** `detections` / `m_tracker.update` / `newS2AviodObject` / UDP。
- `BuildGrid` / `ClusterObstacleGrid` / `ConvertClustersToTrackedObstacles` / `SimpleTracker` 均未改。

---

## 5. DebugViewer 修改

新增 `DebugViewer::DrawHistoricalFeedbackOverlay(clusters, feedback, gridCfg)`，复用已有 `ComputeGridImageGeometry / AddBackGround / WorldToPixel / ShowOrSave`，窗口名 `HistoricalFeedback`，配置键 `HistoricalFeedback`。

三层显示（BGR 颜色）：

| 层 | 内容 | 颜色 | 标签 |
|---|---|---|---|
| Layer 1 | Current-only cell | 琥珀 `(0,200,255)` | `C{cluster.id}` |
| Layer 2 | Historical-only cell | 品红 `(200,0,200)` | `T{track_id}` + 白色 OBB 轮廓 |
| Layer 3 | Overlap cell（Fused） | 黄 `(0,255,255)` | — |

标题：`HistoricalFeedback Cur=%zu Hist=%zu Overlap=%zu Fused=%zu`。

---

## 6. 测试方案（如何回答验收问题）

配置：`config/debug_config_EMX.yaml` 已加 `HistoricalFeedback: {enable:1, show:1, save:1}`（`debug_config.yaml` / `debug_config_E1.yaml` 为 `enable:1, show:0, save:1`）。定位无效时（如 `mapFilterModel=0` 且无定位）会被硬门控自动关闭。

| 场景 | 预期日志/图 |
|---|---|
| Case A 正常检测 | `overlap_tracks>0`，`overlap_cells` 大；图上层 Current 与 Historical 高度重叠（黄色 overlap 多） |
| Case B 部分漏检 | 同一 Track 的 `CurrentCluster` 存在但 `current_cells < historical_cells`，`fused_cells` 明显补齐缺失部分（品红区补在 Current 轮廓缺失侧） |
| Case C 完全漏检 | `CurrentCluster=-1`，计入 `recovery_candidate_tracks`；图上只有 `T{track_id}` 品红区域，落在目标实际位置附近 |
| 移动目标 | 观察 `T{track_id}` 是否停在旧位置（历史拖影）——预期：地图系锚点按上一帧观测投影，移动目标会滞后 |
| 定位无效 | 日志 `[HistoricalFeedback] localization invalid -> disabled`，第十层无输出 |

日志每帧给出：

```
[HistoricalFeedback] Track=10011 CurrentCluster=7 current_cells=31 historical_cells=47 overlap_cells=24 fused_cells=54
[HistoricalFeedback] tracks=N historical_valid_tracks=M overlap_tracks=K recovery_candidate_tracks=R historical_only_cells=X current_total_cells=P fused_total_cells=Q
```

可直接回答 §二十四 的 6 个问题。

---

## 7. 风险

| # | 风险 | 现状 / 缓解 |
|---|---|---|
| 1 | **历史拖影（移动目标）** | 本次**不做**运动预测；锚点=上一帧观测的地图位置，移动目标会滞后。这正是本实验要观察的负面证据。后续正式 Phase 2 需 `motion_state + vx/vy(地图系)`。 |
| 2 | 错误坐标变换（x/y swap、镜像、旋转 90°/180°、degree/radian） | 已核对 `vehicleToMap/mapToVehicle` 互逆且含 heading 补偿；若图上 Historical 出现在镜像/对侧，优先查 `s_grid_heading_offset_deg` 与 lidar.cfg，勿改 Grid。 |
| 3 | Track stale（miss 时锚点冻结但仍投影） | miss 时锚点冻结是**设计行为**（静态先验）；但 `lastSeen` 增大后仍会投影，需结合 `removeLongLostTargets(lastSeen>10)` 与 `kMinTrackAgeForFeedback` 观察。 |
| 4 | Track 生命周期 | 沿用 `removeLongLostTargets: lastSeen>10`（约 1s），锚点随 tracker 删除同步移除。 |
| 5 | 静态/动态目标混用 | 当前**无 motion_state**，统一按静态先验投影；移动目标会拖影（见 #1）。 |
| 6 | Localization invalid | 硬门控 `pose.valid`；无效时禁反馈、锚点冻结，恢复后按最后地图锚点继续（静态合理，动态会漂）。 |
| 7 | HDMap 硬过滤 `in_road` 与软约束矛盾 | 与本次无关，但会导致 off-road 目标永不进 tracker → 无锚点 → 无法反馈（已知风险，见 `tracker-update-analysis.md`）。 |
| 8 | 多 Track 重叠同一 Cluster 时全局 `fused_total` 可能重复计数 | 全局统计已用 union 去重；逐 Track 统计不受影响。 |

---

## 8. 改动文件清单（全部为增量，未改 BuildGrid / Tracker / UDP）

| 文件 | 改动 |
|---|---|
| `include/historical_feedback.h` | 新增：`MapAnchoredTrack` / `HistoricalFeedbackRegion` / `PointInQuad` / `RasterizeQuadToGrid` / `kMinTrackAgeForFeedback` |
| `include/ElevationMapGroundFilter.h` | 新增 getter `GetEffectiveRoiXMin()`（不改 BuildGrid） |
| `include/DebugViewer.h` | 新增 `DrawHistoricalFeedbackOverlay` 声明 + include |
| `include/DebugViewer.cpp` | 新增第十层绘制实现 |
| `include/SuTengDriver.h` | 新增 `m_mapTracks` / `m_historicalFeedback` + 两个方法声明 |
| `include/SuTengDriver.cpp` | 定位读取提升到每帧；新增 `ComputeHistoricalFeedback` / `UpdateMapAnchors` + 调用点 |
| `common/ReadYamlFile.h/.cpp` | 新增 `DEBUG_VIEWER_CONFIG HistoricalFeedback` 及解析 |
| `config/debug_config_EMX.yaml`、`debug_config.yaml`、`debug_config_E1.yaml` | 新增 `HistoricalFeedback` 段 |

---

## 9. 本次明确未做（对应任务禁止项）

- ❌ 修改 `BuildGrid`
- ❌ Raw Point Image / 5cm Image / CSR / Raw Point ID / Raw Point minAreaRect
- ❌ Historical Image / 5-frame Raw Point History / Historical Contour / Morphological Fusion
- ❌ 重写 `SimpleTracker` / 修改最终 UDP 输出 / 重新设计 Kalman/UKF

## 10. 后续建议（不在此次范围）

1. 先跑通 Case A/B/C 与移动目标，确认投影/坐标系无误。
2. 若移动目标拖影不可接受 → 正式 Phase 2：给 `TrackedObstacle` 增加 `map_x/map_y/stable_yaw/motion_state`，速度改到地图系。
3. 若静态漏检补偿验证通过 → 再叠加 Raw Point 二次验证（防止历史把地面/新物体误当障碍物），即正式 Phase 3。
4. 修正 `ConvertClustersToTrackedObstacles` 的 `in_road` 硬过滤与软约束矛盾，否则 off-road 目标永远无锚点。
