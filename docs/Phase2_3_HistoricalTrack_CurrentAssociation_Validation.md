# Phase 2/3 Quick Validation —— Stable Track → Map Anchor → A → 8-Neighborhood Association

> 日期：2026-09-09
> 性质：**Quick Validation / Debug-only**（非正式 Phase 3；不产生 UDP / detections / tracker 修改）
> 目标文件：`Phase2_3_HistoricalTrack_CurrentAssociation_Validation.md`
> 前置：`docs/Phase2_3_QuickValidation_HistoricalFeedback.md`（上一轮 OBB-rasterize / overlap / fused 验证）

本报告只回答一个问题：

> **"历史稳定 Track → Map Anchor → 当前 Radar 预测位置 A → A 所在 Cell ± 8 邻居(3×3) → 当前 Cluster / Track 关联" 这条链在当前代码上是否可靠？**

本轮**没有**实现 Historical Contour Fusion，也没有把 `historical_cells / overlap_cells / fused_cells` 当作主指标（它们保留为 secondary）。

---

## 0. 结论速览（代码级事实 + 待回放确认项分开标注）

1. ✅ 代码已按本阶段目标重写并通过编译（`[100%] Built target low_detection`，result code 0，无新增 warning）。
2. ✅ 时序未变：`m_tracker.update()` → `ComputeHistoricalFeedback()`(上一帧锚点) → `DrawHistoricalFeedbackOverlay()` → `UpdateMapAnchors()`(帧末存本帧锚点)。未把 `UpdateMapAnchors()` 提前。
3. ✅ 关联逻辑从“OBB 栅格化 + overlap/fused”改为“A → Base Cell → 3×3 → 候选 Cluster → 关联（SAME_TRACK_ID / GEOMETRIC 门控）”。
4. ✅ DebugViewer 在既有 `HistoricalFeedback` 图层上叠加：Base Cell(橙)、8 邻居(青)、A 点(橙实心圆)、A→关联 Cluster 连线(绿)。
5. ⚠️ **关键发现（Q5/Q7/Q8 的硬约束）**：当前 `SimpleTracker::removeLostTargets()` 实测删除条件是 `t.lastSeen > 5`（连续 6 帧 miss 即删），且 `UpdateMapAnchors()` 在 tracker 删除 Track 时**同步删除**其 Map Anchor。
   因此如果 10011 在 **320～326 连续 7 帧** 全 miss，它在第 325 帧末就会被 tracker 删除 → 326 起**连 Map Anchor 也没有**，327 不可能再以“历史 Track=10011”产生 A。
   若你之前观察到 327 仍是 10011，说明实际 miss 间隔 ≤5 帧（或阈值在观察时不同）。这是进入 Phase 3 前必须先解决的生命周期问题。
6. ⚠️ **317～327 逐帧数值未在本环境回放**（无匹配 pcap/定位数据组合），报告给出“应出现内容 + 提取命令”，请用你的 pcap 回放后回填（见 §12、§19 模板）。

---

## 1. 当前实际代码路径（全部改动点）

| 文件 | 改动 | 说明 |
|---|---|---|
| `include/historical_feedback.h` | 扩展 | `HistoricalMatchReason` 枚举、`kHistAssociationMaxDistM=0.5`；`HistoricalFeedbackRegion` 新增 A/base cell/3×3 窗口/候选/关联/live-track 字段；`valid` 语义改为“A 投影存在” |
| `include/SuTengDriver.cpp` | 重写 | `ComputeHistoricalFeedback()` 全函数改为 A→3×3 关联 + `[HistoricalAssociation]` 日志（原 overlap/fused 统计降级 secondary） |
| `include/SuTengDriver.h` | 注释 | 方法说明更新 |
| `include/DebugViewer.cpp` | 扩展 | `DrawHistoricalFeedbackOverlay()` 增加 Base Cell/3×3/A/连线 绘制 + 标题含关联计数 |
| 未改动 | — | `BuildGrid / ClusterObstacleGrid / ConvertClustersToTrackedObstacles / SimpleTracker::update / ConvertTrackToS2ObstacleBox / UDP` |

调用点（`ProcessPcapCloud()`，顺序与 §三 要求一致）：

```cpp
m_tracker.update(detections, rec_timestamp_ms);          // 1. 帧 N 先用本帧检测更新
{
    m_historicalFeedback.clear();
    ComputeHistoricalFeedback(outputClusters, loc_pose, have_pose, m_historicalFeedback); // 2. 用帧 N-1 存的锚点
    if (m_debugViewer) m_debugViewer->DrawHistoricalFeedbackOverlay(outputClusters,
        m_historicalFeedback, m_pElevationMapGroundFilter->GetConfig(), hdmap_polygons, loc_pose);
    UpdateMapAnchors(loc_pose, have_pose);               // 3. 帧 N 结束才存帧 N 锚点
}
```

---

## 2. Historical Track 来源（已核实）

- 生产 Track：`SimpleTracker m_tracker`，`vtrackings` = `std::vector<TrackedObstacle>`（`track.h`）。
- 字段实测：
  - `id`：跨帧稳定 ID，从 9999 递增（`nextTrackID`）。→ 本轮历史 Track ID 就用它。
  - `cluster_id`：⚠️ **只在 Track 诞生时写入一次**，`update()` 匹配成功时**不回写** `cluster_id`（代码核实）→ 不能把 `track.cluster_id` 当“本帧簇 ID”。
  - `pos_x/pos_y/pos_z`：雷达系中心（主雷达系 x 前 y 左）。
  - `corners[4] / depth / width / height`：OBB/AABB 几何。
  - `age`：**匹配成功帧累计数**（匹配时 `age++`），miss 不增。
  - `lastSeen`：**连续 miss 帧计数**（匹配成功归 0，miss 每帧 `++`）。
  - `vx/vy`：两点差分速度（本轮只打印，不做运动状态）。
- 历史稳定目标 = `m_mapTracks`（外部侧表 `MapAnchoredTrack`，不修改 `TrackedObstacle`）。
- 门槛：`anchor.has_map && anchor.age >= kMinTrackAgeForFeedback(=3)`。
  - ⚠️ `anchor.age` 是“最后一次匹配成功时的 track.age”（miss 期冻结），所以**不能解释成连续观测帧数**；日志同时打印 `age / lastSeen / live_age / live_lastSeen / live_matched`（见 §12 日志样例）。

---

## 3. Map Anchor 来源

`UpdateMapAnchors(pose, pose_valid)`（未改，仍在帧末）：
- `pose_valid==false` → 全部冻结（不更新）。
- 对每个 `vtrackings`：`lastSeen==0`（本帧匹配成功）→ `vehicleToMap(center)` + 4 角点 → 更新锚点 `map_x/map_y/map_corners/depth/width/age/lastSeen`。
- miss → 冻结中心，仅同步 `lastSeen`。
- tracker 删除的 Track（`lastSeen>5`）→ 同步删除锚点（**本轮关键限制，见 §0.5**）。

---

## 4. Map → Radar 坐标转换路径

`LocalizationManager::Pose`（地图系 ENU：x=东，y=北，heading 度 北=0 顺时针）→ 构造 `VehiclePose{vpose.x=pose.x, vpose.y=pose.y, vpose.heading_deg=pose.heading_deg}` →
`CoordinateTransformer::mapToVehicle(map_x, map_y, vpose, veh_x, veh_y)`。
（`coordinate_transformer.cpp` 内部已统一应用 `s_grid_heading_offset_deg`（=90°+fLidar2Vehicle_Heading），调用方无需再补偿。）

---

## 5. A 的计算方式（本轮最重要结果）

```cpp
double ax=0.0, ay=0.0;
CoordinateTransformer::mapToVehicle(anchor.map_x, anchor.map_y, vpose, ax, ay);
r.projected_x = ax; r.projected_y = ay;   // = A（当前主雷达系）
```

A 与旧 `center_x/center_y` 对齐（= A）。日志中打印 `ProjectedCurrent=(ax,ay)`。

---

## 6. A → Grid Cell 转换方式（复用现有映射，未发明新公式）

与 `BuildGrid/WorldToGrid` 完全一致：

```
row = floor((A.y - roi_y_min) * inv_res)
col = floor((A.x - effective_roi_x_min) * inv_res)
idx = row * cols + col
```

- `effective_roi_x_min = GetEffectiveRoiXMin() = max(roi_x_min, car_half_x+body_filter_x_threshold)`。
- 严格 ROI 内 → `a_inside_grid=true`；距边界 ≤1 cell → 夹取到最近边界 cell 仍做 3×3；>1 cell 完全出 ROI → `a_searchable=false`（视作无当前检测，不硬造 cell）。

---

## 7. 3×3 Neighbor Search 实现方式

```cpp
for (dr = -1..1) for (dc = -1..1) {
    rr = br+dr; cc = bc+dc;
    if (rr<0||rr>=rows||cc<0||cc>=cols) continue;   // 边界合法性
    idx = rr*cols+cc;
    search_window_indices.push_back(idx);
    if (cell_to_cluster.contains(idx)) candidate_set.insert(...);
}
```

即严格按“Base Cell + 8 邻居 = 3×3”枚举（非只查 A 所在 cell）。窗口 cell 全部记录在 `search_window_indices`，日志用 `Search3x3` 三行打印（见 §12）。

---

## 8. Cluster Candidate 获取方式

- 预建 `cell_to_cluster`：遍历当前 `outputClusters` 每个 `cell_indices`，线性索引 → 所属 cluster id（first-wins）。
- 候选 = 3×3 窗口内出现的**去重** cluster id（升序），存 `candidate_cluster_ids`。
- 一个 Cluster 占据多个窗口 cell 只计一次（如 C5 占 {1002,1003,1004} → 只记录一次 `5`）。
- **没有重新聚类**。

---

## 9. Track/Cluster Association 方式

对每个历史 Track（anchor）：

```
if (live_track 存在 && live.lastSeen==0)          // 当前 tracker 同 ID 且本帧匹配
    → 反查该 track 本帧被匹配的 cluster（参考中心 ≈ live.pos，<0.05m）
    → matched = 该 cluster；reason = SAME_TRACK_ID；inWindow = 是否在 3×3 内
else if (live_track == nullptr && 有候选)          // 同 ID 已被删（可能新 ID 重建 / 不同目标）
    → 3×3 窗口内最近候选，且距离 ≤ 0.5m → matched；reason = GEOMETRIC_DISTANCE；inWindow = true
else                                              // live 存在但 miss / 无候选 / A 完全出 ROI
    → matched = -1；reason = NO_CURRENT_DETECTION
```

说明：
- **优先 Track ID 确认**（`SAME_TRACK_ID`）只在“同 ID track 本帧匹配成功”时可用；因为 tracker 匹配时**不回写 cluster_id**，我们用“cluster 参考中心 ≈ live.pos（tracker 把检测中心拷给 track.pos）”反查其 cluster，属现有代码结构下的可靠反查。
- **几何 fallback 只在同 ID 已不存在时启用**，并且：
  - 候选必须占据 3×3 窗口内 cell（grid 级门控）；
  - A→候选中心距离 ≤ `kHistAssociationMaxDistM = 0.5 m`；
  - 同 ID 正在 miss 时**禁止**几何关联（避免把“附近其他障碍物”误认成本目标，见 §13、指标 4）。

---

## 10. Gating / Distance Criterion

| 规则 | 值/条件 | 目的 |
|---|---|---|
| Grid 级 | 候选 Cluster 必须至少一个 cell 落在 3×3 窗口 | 先验空间近邻 |
| SAME_TRACK_ID | 无距离上限（tracker 已用 0.5m 阈值确认过） | 复用 tracker 可靠 ID |
| GEOMETRIC | A→候选中心 ≤ 0.5 m；取最近 | 防止“无限制取最近” |
| 反查容差 | cluster 参考中心 ≈ live.pos < 0.05 m | 定位 tracker 匹配到的 cluster |
| 生命周期 | live 存在但 miss → 不几何关联 | 防 miss 期误关联 |

参考距离量级：317/319/327 中心 ≈ (9.28,1.42)/(9.30,1.40)/(9.22,1.42)，若 A≈(9.30,1.40)，则距离 ≈ 0.02~0.08 m，远小于 0.5 m 门控；3×3 允许 A 与 cluster 中心差 ≤ ~1.5 cell（≈0.21m 对角）仍能命中邻居 cell。

---

## 11. DebugViewer 可视化结果

在既有 `HistoricalFeedback` 图层（琥珀=当前 cluster cell，品红=历史 OBB cell，黄=重叠）之上，每帧新增（全部为既有坐标系，无新可视化系统）：

| 元素 | 画法/颜色 |
|---|---|
| Base Cell（A 所在） | 橙色矩形边框（thickness 2） |
| 8 邻居 Cell | 青色矩形边框（thickness 1），越界 cell 不画 |
| A 点 | 橙色实心圆 + 白描边 + “A” 标签 |
| A → 关联 Cluster Center | 绿色连线 + 绿色中心圆（仅 `association_cluster_id>=0`） |
| 标题 | `HistoricalFeedback Cur/Hist/Overlap/Fused | A-assoc same= n geom= n none= n` |

关键视觉验证点：**即使 A 所在 cell(1001) ≠ 当前 cluster 所在 cell(1002)，只要 C5 占据 3×3 邻居 cell，其 amber cell 会落在青色邻居框内，且绿色连线从 A 指到 C5 中心** → 一眼可判“邻居 cell 关联成功”。

---

## 12. 日志（每历史 Track 一簇，便于 317~327 回放核对）

样例（对应 §十二 要求字段；数值为占位，回放后回填）：

```
[HistoricalAssociation] Track=10011 age=34 lastSeen=6 live_age=40 live_lastSeen=0 live_matched=1 mapAnchor=(xxx.xx,xxx.xx)
[HistoricalAssociation]   ProjectedCurrent=(9.30,1.40) insideGrid=1 baseCell=(row,col) windowCells=9
[HistoricalAssociation]   Search3x3 row-1: |  -  |  -  |  -  |
[HistoricalAssociation]   Search3x3 row  : |  -  | C5  |  -  |
[HistoricalAssociation]   Search3x3 row+1: |  -  |  -  |  -  |
[HistoricalAssociation]   Candidates: Cluster=5 Center=(9.22,1.42) Dist=0.083 |
[HistoricalAssociation]   MatchedCluster=5 Center=(9.22,1.42) Distance=0.083 inWindow=1 reason=SAME_TRACK_ID
```

无当前检测时：

```
[HistoricalAssociation] Track=10011 age=34 lastSeen=6 ...
[HistoricalAssociation]   ProjectedCurrent=(9.30,1.40) ...
[HistoricalAssociation]   Candidates: none
[HistoricalAssociation]   MatchedCluster=-1 reason=NO_CURRENT_DETECTION (Historical A exists, no current detection)
```

帧摘要：`[HistoricalAssociation] frame summary: hist_tracks=N sameTrackIdMatches=M geomMatches=K noCurrentDetection=L`

提取命令（ulog 输出在 `config/ulog.cfg` 指定，默认 `log/ulog/ulog.log`）：

```bash
grep -n "Track id: 10011\|HistoricalAssociation" log/ulog/ulog.log | sed -n '1,400p'
```

---

## 13. 是否出现误关联（待回放，防误设计已就位）

- 候选被限制在 3×3 窗口 → 远处 cluster 不可能被选；
- 同 ID 正在 miss 时不几何关联 → 历史目标“失踪期”不会抓到旁边其他障碍物；
- GEOMETRIC 只在同 ID 不存在时启用，且 ≤0.5m；
- 观测方法：若某帧 `Candidates` 出现 ≥2 个 cluster，且 `MatchedCluster` 与 A 的几何关系与其他候选接近，需人工核对 `C{id}` 中心是否为同一目标（对应指标 4）。

---

## 14. 317~327 期望 vs 代码行为

| 帧 | 期望 | 代码预期行为 | 前提 |
|---|---|---|---|
| 317 | C2→10011 | `SAME_TRACK_ID` matched=C2, inWindow=1 | 10011 有历史锚点且本帧匹配 |
| 319 | C3→10011 | `SAME_TRACK_ID` matched=C3 | 同上（cluster id 变化而 track id 稳定正是本验证想证明的） |
| 320~326 | 无 cluster | A 存在 + `NO_CURRENT_DETECTION` | **锚点存活**（见下） |
| 327 | C5→10011 | 若 10011 存活：`SAME_TRACK_ID`；若已被删：新 ID 建立 → `GEOMETRIC`（若 A 距 C5 ≤0.5m） | 受 §0.5 生命周期约束 |

⚠️ **320～326 = 7 帧 miss > 当前 `lastSeen>5` 删除阈值（6 帧）**。按当前代码，10011 会在第 325 帧末被删除 → 锚点同步删除 → 326 起不再有“10011 的 A”。因此：
- 若你的回放里 327 想看到“历史 Track=10011 的 A → C5”，必须保证 10011 在 320-326 内的实际 miss 间隔 ≤5 帧，或临时把 `SimpleTracker::removeLostTargets()` 阈值调大（仅实验；**本轮未改 tracker**）。
- 这正是 §17 / Q8 需要补的“锚点独立生命周期”。

---

## 15. Projected A 的稳定性（设计分析，待回放数值确认）

- A = 冻结的 Map Anchor 用**当前位姿**反投影。因此：
  - 车辆静止（或定位给出的自车运动与实际一致）且目标静态 → A 在 miss 期几乎不动（帧间仅差定位噪声）。
  - 车辆移动而目标静态 → A 会**沿车运动反向平移**（这是正确行为：地图系目标 → 当前雷达系）。
  - 目标移动 → 因 anchor 冻结，A 会滞后（拖影）——**没有 motion_state 前这是已知局限**，日志可观察。
- 判定方法：连续帧看 `ProjectedCurrent` 是否 ~不变/按车速平滑移动，而不是看 fused_cells。

---

## 16. 当前 age / lastSeen 的实际语义（重要，勿误读）

- `Track.age`：**累计匹配成功帧数**（每次匹配 +1，miss 不 +1）。不是“连续观测帧数”。
- `Track.lastSeen`：**当前连续 miss 计数**（匹配成功清零，miss 每帧 +1）。
- `SimpleTracker::removeLostTargets()` 实际执行 `t.lastSeen > 5`（代码核实；`track.cpp` 第 484 行），即**连续 6 帧 miss 删除**。
- `MapAnchoredTrack.age/lastSeen`：最后一次更新时的快照；miss 期 age 冻结、lastSeen 同步 → 只要曾 ≥3 次匹配，miss 中依然过年龄门槛。
- `anchor.age` 门槛（`kMinTrackAgeForFeedback=3`）保留为临时门槛，但**不作为“连续观测”证据**，真实状态以 `live_age/live_lastSeen/live_matched` 为准。
- `Track.cluster_id`：诞生簇，**匹配时不更新**（代码核实）→ 不用它做簇对应。

---

## 17. 当前实现与未来 Historical Contour Fusion 的接口关系

本阶段产出的 `HistoricalFeedbackRegion`（每历史 Track 一个）已包含 Phase 3 需要的**空间先验输入**：

```
track_id / A(projected_x/y) / base cell / 3×3 window / candidate clusters /
association_cluster_id / association_reason / association_in_window / association_distance /
matched_center / live_track_* 状态
```

Phase 3 可直接消费：
- `association_reason == SAME_TRACK_ID` → 当前有匹配，无需补充（可顺带验证几何误差 `association_distance`）。
- `association_reason == NO_CURRENT_DETECTION` 且 `A` 可信（静态目标）→ **这正是 Historical Contour Supplementation 的触发点**：用 A 回到当前帧 Raw Point Image / 5cm Image 找点云证据。
- `association_reason == GEOMETRIC` → 提醒“同 ID 已断，需谨慎/可重新建档”。

但 Phase 3 需要 A **在长 miss 期仍存在**，当前 `m_mapTracks` 生命周期绑定 tracker 删除，无法跨 >6 帧。→ 接口上需要新增“锚点独立存活表”。

---

## 18. Phase 3 仍然需要完成的工作

1. **锚点独立生命周期**：Map Anchor 需在 tracker 删除后继续保留（带独立 age/lastSeen/motion 判定），否则 320-326 这类 >6 帧漏检期无先验可用。
2. **Raw Point Image / 5cm image + CSR + Raw Point ID history**（见 `docs/MapFrameHistoricalFeedbackRawPointImage_FinalArchitecture.md`）。
3. **Current raw points → `minAreaRect`** 得到当前精确 contour（不要再回退到 10cm cell）。
4. **Historical contour 存储**（按 track id + 地图系 yaw / stable geometry），实现 Current contour + Historical contour 融合。
5. **motion_state（UNKNOWN/STATIC/MOVING）**：静态才允许 contour 补充，移动目标避免拖影（本轮只打印 vx/vy）。
6. 逐帧回放 317~327 并回填 §14 表格，验证 Q1~Q6 数值。

---

## 19. Q1~Q8 明确回答

**Q1. 历史 Track → Map → 当前 Radar 的位置预测 A 是否稳定？**
代码链已闭环并可在日志打印 A（`ProjectedCurrent`）。稳定性**数值待回放确认**；机制上：车辆静止 + 目标静态 + 定位准确时 A 帧间应 ~不变；车辆移动时 A 应平滑移动（锚点在地图系）。当前无 motion_state，移动目标会有拖影（已知）。

**Q2. A 与实际当前 Cluster 中心之间通常有多大距离？**
按你给的 317/319/327 中心 (9.28,1.42)/(9.30,1.40)/(9.22,1.42) 与 A≈(9.30,1.40)，预期 **≈0.02~0.08 m**（即亚 cell 级）。逐帧数值以 `Distance=` 为准回填；`inWindow=1` 说明 3×3 能容忍约 1 cell（0.1m）以上的误差。

**Q3. 若 A 与 Current Cluster 不在同一个 10cm Grid Cell，3×3 能否正确找到？**
**能**——只要该 cluster 任一 cell 落在 A 的 3×3 窗口内即成为候选并关联（`inWindow=1`）。代码就是为这个场景写的；需用 317→C2 / 327→C5 回放证实。

**Q4. 317、319、327 是否分别关联到 C2、C3、C5？**
设计上：317(C2)/319(C3) 通过 **SAME_TRACK_ID**（同 10011 本帧匹配，位置反查簇）；327 是否也是 10011 取决于其在 320-326 是否存活——**当前代码 7 帧 miss 会让 10011 在第 325 帧被删**，若如此 327 只能靠新 ID + GEOMETRIC 关联（或无锚点）。需要你回放核实 10011 真实存活帧数后回填。

**Q5. 320～326 miss 是否保持“Historical A 存在，但 Current Cluster 不存在”？**
只要锚点仍存活：会打印 `A 存在 + MatchedCluster=-1 reason=NO_CURRENT_DETECTION`（不硬造检测）。但**当前锚点在连续 6 帧 miss 后随 tracker 删除而消失** → 320-326（7 帧）尾部可能已无 A。这正是 Phase 3 要补的（§18.1）。

**Q6. 是否出现误关联到其他障碍物？**
防误设计已就位：候选限 3×3、同 ID miss 不几何关联、几何门控 ≤0.5m、GEOMETRIC 仅在同 ID 不存在时启用。是否实际出现**待回放确认**；观察 `Candidates` 多条 + 对比各 `C{id}` 中心即可发现。

**Q7. 当前代码是否已具备进入 Phase 3 "Historical Contour Supplementation" 的基本位置关联条件？**
**部分具备**：A→Base Cell→3×3→Cluster/Track 关联链已实现并在“同 ID 匹配帧”给出可靠 SAME_TRACK_ID 确认；miss 期能输出 A 先验（锚点存活期内）。**尚不完整**（见 Q8）。

**Q8. 如果还不具备，具体缺少什么？**
1. **锚点独立生命周期**（当前与 tracker 删除绑定，>6 帧 miss 后无 A）——最优先；
2. **motion_state / 静态判定**：否则无法决定“A 可信、应触发补充”，移动目标会拖影误补；
3. **317-327 逐帧实测数值**尚未回填（需在你的 pcap + 定位配置下回放，本环境无匹配数据组合）；
4. 当前 tracker 匹配时**不回写 cluster_id**，SAME_TRACK_ID 用“位置反查”间接实现；若未来要求显式 Cluster→Track 对应，建议在 tracker 匹配处回写（属 Phase 3 范围，本轮按约束未改 tracker）。

---

## 20. 本阶段原则确认

- ✅ 只做：历史 Track→Map Anchor→A→A±1 Grid→当前 Track/Cluster 关联验证（Debug only）。
- ❌ 未做：Historical Contour Fusion、5cm Raw Point Image/CSR、Raw Point→minAreaRect、修改 `m_tracker.update()` 核心行为、修改 UDP 输出、引入 UNKNOWN/STATIC/MOVING 完整分类。
- ✅ `historical_cells / overlap_cells / fused_cells` 已降级为 secondary（标题仍显示，但不作为成功标准）。

---

## 21. 为什么新障碍物前几帧 `hist_tracks=0`？—— `m_mapTracks` 生效条件与 271~278 逐帧复算

> 现象：障碍物刚出现的多帧 `hist_tracks=0`，但 `m_tracker.update()` 明确已经跟踪到（如 `Track id: 10011`）。
> 结论：**这不是 tracker 没检测到，也不是关联失败，而是 Historical Association 的“年龄门槛”尚未满足（`kMinTrackAgeForFeedback=3`），叠加“帧末才写锚点”的时序。**

### 21.1 核心原因

`ComputeHistoricalFeedback()` 只处理同时满足以下条件的锚点（`m_mapTracks` 条目）：

```cpp
anchor.has_map == true
anchor.age    >= kMinTrackAgeForFeedback   // = 3
```

而 `anchor.age` 的赋值是**“最后一次匹配成功那一帧的 `track.age`”**（见 `UpdateMapAnchors()` 的 `if (t.lastSeen == 0)` 分支）。
`track.age` 从 0 起步，且**只在匹配成功的帧 +1**（`SimpleTracker::update()` 内 `vtrackings[i].age++`）。

因此一个新 Track 在它生命早期的若干“匹配成功帧”里 `age = 0, 1, 2, 3 …`；**只要 `age < 3`，该锚点就被 `continue` 跳过**，于是这些帧的 `hist_tracks=0`。

### 21.2 `m_mapTracks` 有值（且能被输出）的完整条件链

| # | 条件 | 代码位置 | 说明 |
|---|---|---|---|
| 1 | `UpdateMapAnchors()` 执行时 `pose_valid==true` | `UpdateMapAnchors()` 开头 `if (!pose_valid) return;` | **定位无效时整个函数直接返回，连锚点都不会创建** → `m_mapTracks` 为空。这是“很多帧 hist_tracks=0”的第一嫌疑项，日志会出现 `[HistoricalAssociation] localization invalid -> disabled`。 |
| 2 | Track 存在于 `m_tracker.vtrackings` | `for (const auto& t : m_tracker.vtrackings)` | 即该目标必须被检测并匹配成功过一次（`update()` 才会建档）。 |
| 3 | 锚点条目被创建，且 `has_map=true` | 遍历中 `push_back(MapAnchoredTrack())`；`has_map=true` 仅在 `t.lastSeen==0` 分支 | 匹配帧才会写入地图锚点；纯 miss 不写中心。 |
| 4 | **输出门槛** `anchor.age >= 3` | `ComputeHistoricalFeedback()` 中 `if (anchor.age < kMinTrackAgeForFeedback) continue;` | 未达标的锚点完全不参与统计，故 `hist_tracks` 不含它。 |

补充两个“时序/生命周期”事实（容易误判）：
- **帧内顺序**：`ComputeHistoricalFeedback()` 在 `UpdateMapAnchors()` **之前** 执行 → 帧 N 用的是**帧 N-1 帧末**写入的 `anchor.age`（差一帧），这正是 §三 要求“用上一帧已有历史信息”。
- **锚点随 tracker 删除而删除**：`removeLostTargets()` 当前为 `lastSeen > 5`，Track 被删则锚点同步删除；目标若以**新 ID** 重现，`age` 从 0 重新累计 → 又会经历一段 `hist_tracks=0`。这就是“不止一个障碍物会连续多帧 `hist_tracks=0`”的另一来源。

### 21.3 你的 271~278 逐帧复算（与日志逐项吻合）

| 帧 | 检测 / Tracker | `track.age`(帧末) | `lastSeen` | 帧末 `anchor.age` | 本帧 `ComputeHistoricalFeedback` | 说明 |
|---|---|---|---|---|---|---|
| 271 | C9 → 新建 10011 | 0 | 0 | 0 | `hist_tracks=0` | 锚点刚建立，age=0 < 3 |
| 272 | C10 匹配 | 1 | 0 | 1 | `hist_tracks=0` | age=1 < 3 |
| 273 | 漏检 | 1 | 1 | 1（冻结） | `hist_tracks=0` | **miss 不会让 age 增长** |
| 274 | 漏检 | 1 | 2 | 1（冻结） | `hist_tracks=0` | 同上 |
| 275 | C5 匹配 | 2 | 0 | 2 | `hist_tracks=0` | age=2 < 3 |
| 276 | 漏检 | 2 | 1 | 2（冻结） | `hist_tracks=0` | 同上 |
| 277 | C6 匹配 | 3 | 0 | 3 | `hist_tracks=0` | **帧末**才把 anchor.age 写成 3；本帧计算仍用上一帧的 2 |
| 278 | C8 匹配 | 4 | 0 | 4 | **`hist_tracks=1`**，`Track=10011 age=3 lastSeen=0 live_age=4 …` | 本帧用帧末 277 的 `anchor.age=3` → **首次通过门槛** ✅ |

关键自洽点：
- 你 278 行的 `age=3 live_age=4` **完全正确**：
  - `age=3` = 锚点快照（来自 277 帧末，`track.age` 首次到 3）；
  - `live_age=4` = 本帧 `m_tracker.update()` 之后当前 track.age（278 又匹配了一次）。
- 273/274/276 的漏检**只增大 `lastSeen`、不增大 `age`**，所以从“日历帧”看要等到 278（第 8 帧），而从“匹配帧”看只需要累计 3 次成功匹配（271→272→275→277 四次匹配把 age 推到 3）。

### 21.4 为什么“多个障碍物都会连续多帧 `hist_tracks=0`”

1. 每个新 Track **各自**从 `age=0` 起步，都要独立跨过 `age>=3` 门槛；
2. 目标若间歇性漏检，`age` 只在匹配帧 +1 → 达标所需**日历帧数更多**；
3. `frame summary` 里的 `hist_tracks = out.size()` = **通过门槛的锚点数**，所以任一帧只要所有存活 Track 的 `anchor.age` 都 <3，就打印 `hist_tracks=0`（即使 tracker 里有一堆 `Track id`）；
4. 还有两种会**直接**导致 `hist_tracks=0`：
   - `pose_valid==false`（定位无效）→ 函数提前 return，日志为 `localization invalid -> disabled`；
   - 目标被 `removeLostTargets()`（`lastSeen>5`）删除、随后以新 ID 重现 → 锚点被删后再从 age=0 重新累计。

### 21.5 如何快速区分“年龄门槛”与“定位无效”

查看同一帧附近的日志：

```
[LocDiag] cloud_ts_ms=... loc_valid=1 loc_age_ms=... loc_n=...
```
- 若 `loc_valid=1` 且**没有** `[HistoricalAssociation] localization invalid -> disabled` → `hist_tracks=0` 属于**正常的年龄未达标**；
- 若 `loc_valid=0` 或出现 `localization invalid` → 是定位门控，需先修定位/配置（`Localization.enable`、`mapFilterModel`、离线固定位姿等）。

### 21.6 对本次验证的影响（重要）

- **新障碍物刚出现的前几帧 `hist_tracks=0` 属于预期行为，不是关联失败。** 评估“A→3×3→Cluster”关联能力时，应只看 `anchor.age>=3` 之后的帧（例如本例从 278 起）。
- 若希望新目标更早获得历史先验，可临时下调 `kMinTrackAgeForFeedback`（例如 1 或 2）做实验；但这会增加“未稳定目标”被纳入历史先验的风险，**本轮按约束不改生产 Tracker，也不改默认门槛**。

---

## 22. 顺带修复：`Search3x3` 日志重复打印

**现象**（你的日志）：

```
Search3x3 row  : | C8   C8   C8  | C8   C8  | C8  |
```

同一行内容被打印了 3 遍。

**原因**：原实现把 3 段 5 字符写入 `tagbuf[row] + (dc+1)*5`，再用 `%s` 打印 `tagbuf[row] / +5 / +10`。但 `snprintf(seg, 6, ...)` 写满 5 字符后其 `\0`（位于 offset 5）会被**下一段的首字符覆盖**，导致 `tagbuf[row]` 实际是一条 15 字符的无终止字符串 → 输出变成“整行 15 字符 + 后 10 字符 + 后 5 字符”。

**修复**：改用 `char celltag[3][3][8]`，**每格独立缓冲**，按 `[%s][%s][%s]` 输出。修复后单行样例：

```
[HistoricalAssociation]   Search3x3 row  : [C8 ][C8 ][C8 ]
[HistoricalAssociation]   Search3x3 row+1: [C8 ][  -][  -]
```

（`C{id}` 表示该 cell 属于当前 Cluster；`  - ` 表示窗口内该 cell 无 cluster；`  X ` 表示越界未搜索。）
