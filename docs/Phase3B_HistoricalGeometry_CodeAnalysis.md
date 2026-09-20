# Phase 3-B 代码分析：STATIC Historical Geometry

> 日期：2026-09-11
> 性质：**实施前代码检查 + 实现方案**（本文件基于当前真实代码逐项核实，不依赖文档猜测）
> 目标：`STATIC + 当前检测存在 + Phase 2 关联有效` 时，用 Historical Geometry **安全补充**当前 Cluster 的 cell footprint。
> 明确不做：STATIC+MISS/Coast、MOVING/UNKNOWN 融合、Raw Point/5cm/CSR、Track lifetime 重设计。

---

## 1. 当前历史 geometry 从哪里来（逐项核实）

```
m_tracker.update()                    // 本帧 Track（原始检测几何）
      ↓
UpdateTrackMotionStates()             // Phase 3-A：只写 motion_* / map_* 新字段
      ↓
ComputeHistoricalFeedback()           // Phase 2：产 HistoricalFeedbackRegion
      ↓
UpdateMapAnchors()                    // 帧末：写 MapAnchoredTrack（地图锚点）
```

- `MapAnchoredTrack` 由 `UpdateMapAnchors()` 在**每帧末**、且仅当 `t.lastSeen==0`（匹配成功）时写入：`vehicleToMap(t.pos_x, t.pos_y)` 与 `vehicleToMap(t.corners[0..3])`。
- `HistoricalFeedbackRegion` 由 `ComputeHistoricalFeedback()` 在本帧产：`mapToVehicle(anchor.map_x/y)` → A；`mapToVehicle(anchor.map_corners[0..3])` → `corners[4]`（**当前雷达系**）；`RasterizeQuadToGrid(corners)` → `projected_cell_indices`（**当前 Grid**）。

## 2. 当前历史 geometry 保存什么（字段级）

| 结构 | 字段 | 坐标系 |
|---|---|---|
| `MapAnchoredTrack`（`historical_feedback.h`） | `id` | — |
| | `map_x, map_y` | **地图系 ENU** |
| | `has_map` | — |
| | `depth(=obb_length), width(=obb_width)` | 雷达系尺寸 |
| | `map_corners[4]` | **地图系** 4 角点 |
| | `age, lastSeen` | — |
| `HistoricalFeedbackRegion` | `projected_x, projected_y`（=A） | 当前雷达系 |
| | `corners[4]` | 当前雷达系（历史 OBB 投影） |
| | `length, width` | 雷达系（来自 anchor.depth/width） |
| | `projected_cell_indices` | **当前 Grid 线性索引**（历史 OBB 栅格化） |
| | `association_cluster_id / _reason / _in_window / _distance` | 本帧关联结果 |

**关键核实**：
- 历史侧 **没有独立 `yaw` 字段**；历史 yaw 可由 `corners[0]→corners[1]`（primary/length 轴）推出：`atan2(c1.y-c0.y, c1.x-c0.x)`。
- `projected_cell_indices` **已存在且可直接复用**为 “historical predicted cells”，无需新增投影 / 新表 / 新 3×3 search。
- 历史 geometry 的**时效性**：锚点来自**最后一次匹配成功那一帧的原始检测几何**（miss 期冻结），**不是多帧稳定 footprint**。这是第一版的已知限制（§46/§47）。

## 3. 如何投影到 current grid

**复用 Phase 2 结果**：直接使用 `HistoricalFeedbackRegion.projected_cell_indices`。
若需要 cell 中心坐标（距离/连通性判断），复用与 `BuildGrid` 完全一致的换算：

```
row = idx / cols ; col = idx % cols
cx  = eff_x_min + (col + 0.5) * res
cy  = roi_y_min + (row + 0.5) * res
idx = row * cols + col
```

`res = cfg.grid_resolution (0.1m)`，`eff_x_min = GetEffectiveRoiXMin()`。**不修改** Grid resolution / origin / ROI / BuildGrid。

## 4. 当前 cluster cells 从哪里取得

`outputClusters`（`std::vector<GridCluster>`）→ `GridCluster.cell_indices`（当前帧已生成）。
**不重新** BuildGrid / ClusterObstacleGrid / 不从 raw point 生成。

## 5. 如何判断 cell 是否属于其他 cluster

`ComputeHistoricalFeedback()` 内部已建 `cell_to_cluster`（first-wins）但**未持久化**。
Phase 3-B 在融合时**本地重建**同款 `cell_owner`（遍历 `clusters[].cell_indices`）。成本 O(总 cell 数)，无新增状态。

## 6. Missing cell 定义

```
missing = { c ∈ projected_cell_indices | c ∉ current_cluster_cells }
```

## 7. 安全过滤（见 §9 规则表）

## 8. Weighted OBB

新增 header-only `include/historical_geometry.h`：
- 2×2 **加权**协方差 + 闭式特征分解（等价于 `ComputeClusterOBB` 的 PCA，但支持权重）；
- 权重：Current cell = `1.0`，Historical supplement cell = `0.5`；
- `orientation_confidence = (λ1-λ2)/(λ1+λ2)`（与现有 `ComputeClusterOBB` 的公式一致——**该值当前只被计算、未保存**，Phase 3-B 在 helper 内直接产出，不改 `GridCluster`）；
- `n < 3` → 不做 PCA（保留当前 OBB）；
- 保证 `length ≥ width`；yaw fold 到 `[-90°,90°)`；
- 低置信度（`conf < 0.30`）→ **保留历史 yaw**（STATIC prior），只采用 fused 的 center/L/W。

**不修改** `ComputeClusterOBB()`（Detection pipeline 核心）。

## 9. 如何写回 vtrackings

仅当 `STATIC + association 有效 + 有 accepted supplement` 时：

```
m_tracker.vtrackings 中 id == r.track_id 的 Track:
    pos_x, pos_y   = fused OBB center        （保留 pos_z）
    depth, width   = fused L / W
    corners[4]     = fused corners（顺序 左下→右下→右上→左上）
```

**不修改**：`map_x / map_y / motion_* / age / lastSeen / vx / vy / id / cluster_id`。

## 10. 是否会影响 Phase 2（含 §三十 anchor 污染分析）

**把 Phase 3-B 放在 `UpdateMapAnchors()` 之后**，并保持现有调用顺序不变（§三十一）：

```
m_tracker.update()
  → UpdateTrackMotionStates()          (3-A)
  → ComputeHistoricalFeedback()        (Phase 2 association)
  → DrawHistoricalFeedbackOverlay()
  → UpdateMapAnchors()                 (Phase 2 anchor, 读原始几何)
  → 【新增】ApplyHistoricalGeometry()  (3-B, 之后才改 vtrackings)
```

- `UpdateMapAnchors()` 只在 `t.lastSeen==0` 时读 `t.pos_*/t.corners`；3-B 在其**之后**执行 → 锚点始终基于**原始检测几何**，**不会被 fused 几何污染**。
- miss 分支只同步 `anchor->lastSeen`，**不读** pos/corners → 即使 vtrackings 保留上一帧 fused 值也不影响锚点。
- 唯一副作用：fused 几何会保留在 vtrackings 到**下一帧 `tracker.update()`**；对已匹配 Track，`update()` 会用原始检测**覆盖** pos/corners（并据此重算 vx），因此 **fusion 不累积**；仅有“下一帧匹配代价使用 fused 位置”的轻微影响（supplement 限距 ≤0.2m、面积比 ≤1.5 → 中心偏移很小）。**不改 tracker**。

## 11. 是否会影响 Phase 3-A

只**读** `track.motion_state`（作为策略开关），不修改 Motion State / map history / 阈值 / 状态机。

## 12. 是否会影响 Detection

不影响。3-B 位于 `Cluster 已生成 + Tracker 已出 Track` 之后；未触碰 `BuildGrid / ProcessWithObstacleDetection / ClusterObstacleGrid / BuildClusterFromCells / ComputeClusterOBB / PointCloudTransform`。

## 13. 是否会影响 HDMap

不影响。未读写 `in_road`，未改 `HDMapFilter / HDMapManager`。

## 14. 是否会影响 UDP

**协议不变**。`ConvertTrackToS2ObstacleBox(m_tracker.vtrackings, ...)` 继续使用同一接口与 `S2obstacleBox{depth,width,height,pos_x/y/z,corners[4]}`；因 3-B 写回了 fused 几何，UDP 内容会体现融合结果（这是 Phase 3-B 的目的，非协议改动）。

---

## 附A. 安全规则 → 实现位置

| # | 规则 | 位置 |
|---|---|---|
| 1 | Grid 有效性（cell 在 grid 内） | `FuseHistoricalGeometry()`：`idx` 解出的 row/col 范围检查 |
| 2 | 不得属于其他 cluster（owner 排他） | `cell_owner` 映射；`owner!=cid` → `OTHER_CLUSTER` |
| 3 | 距当前 cluster 最近 cell ≤ 0.20m | `kHistoricalSupplementMaxDistM`；`TOO_FAR` |
| 4 | 8-connected 连通性 | 候选按距离升序，仅当与已接受集合 8-邻接才接受；否则 `DISCONNECTED` |
| 5 | 面积比限制 | `max_supp = floor(cur*(ratio-1))`，`cur<=2` 时例外允许 +1；否则 `AREA_LIMIT` |
| 6 | STATIC 才补 | `motion_state == STATIC`，否则跳过 |
| 7 | association 有效 + ≤0.5m（沿用 Phase 2 阈值） | `association_cluster_id>=0 && reason∈{SAME_TRACK_ID,GEOMETRIC}`；`kHistAssociationMaxDistM` |
| 8 | 一个 current cluster 只能被一个 historical track 补 | `cluster → set<track_id>`，size>1 → `AMBIGUOUS` 跳过 |

## 附B. 开关（§35/§51）

```cpp
kEnableHistoricalCellSupplement = true;   // 3-B-1：cell 补充 + 日志
kEnableHistoricalFusedOBB       = false;  // 3-B-2：加权 OBB 写回
```

第一轮：cell 补充 ON、fused OBB OFF（只验证/记录 cell，不改几何）。

## 附C. 调用顺序结论（§三十一）

**不调整现有调用顺序**；新增的 3-B 步骤放在 `UpdateMapAnchors()` 之后、Debug/UDP 之前。分析证明该位置不会污染 Phase 2 锚点（附 §10）。

---

## 附D. 实现完成情况（代码已实施）

### D.1 实际修改文件

| 文件 | 改动 |
|---|---|
| **新增** `include/historical_geometry.h` | 开关 + 参数 + `WeightedOBB` / `HistoricalGeometryResult` + `HistCellCenter` / `HistFoldYawDeg` / `ComputeWeightedOBB` / `FuseHistoricalGeometry()`（全部 header-only inline） |
| `include/SuTengDriver.h` | include 新头；新增 `m_historicalGeometry` 与 `ApplyHistoricalGeometry()` 声明 |
| `include/SuTengDriver.cpp` | 在 `UpdateMapAnchors()` 之后调用 `ApplyHistoricalGeometry(outputClusters)`；实现该函数并输出 `[HistoricalGeometry]` 日志 |

**未修改**：`track.cpp`（`SimpleTracker::update` / `removeLongLostTargets`）、`track.h`、`ElevationMapGroundFilter.*`（含 `ComputeClusterOBB`）、`PointCloudTransform`、`ComputeHistoricalFeedback` / 3×3 Search、`UpdateMapAnchors`、`HDMap`、`LocalizationManager`、`CoordinateTransformer`、UDP。

### D.2 开关与参数（`historical_geometry.h`）

```cpp
kEnableHistoricalCellSupplement = true;    // 3-B-1：cell 补充 + 日志（默认 ON）
kEnableHistoricalFusedOBB       = false;   // 3-B-2：加权 OBB 写回（默认 OFF）
kHistoricalGeometryVerboseCells = false;   // 逐 cell reject 原因日志（默认 OFF）
kHistoricalSupplementMaxDistM   = 0.20f;
kHistoricalMaxAreaRatio         = 1.5f;
kHistCurrentWeight              = 1.0f;
kHistSupplementWeight           = 0.5f;
kHistYawMinConfidence           = 0.30f;
kHistMinCellsForPCA             = 3;
```

第一轮即当前默认：**cell 补充 ON、fused OBB OFF**（只记录/验证 cell，不改几何）。

### D.3 日志

```
[HistoricalGeometry] Track=10011 state=STATIC cluster=5 assoc=SAME_TRACK_ID cur=4 hist=6 missing=3 accepted=2 rej(oog=0 other=0 dist=1 conn=0 area=0) area_ratio=1.50 max_supp=2 fusion=ACCEPT obb=0
[HistoricalGeometry] Track=10011 state=MOVING cluster=5 assoc=SAME_TRACK_ID cur=4 hist=6 missing=3 accepted=0 skip=NOT_STATIC
[HistoricalGeometry]   fusedOBB center=(...) L=.. W=.. yaw=.. conf=..      // 仅 obb=1 时
```

`skip_reason` ∈ `NO_TRACK / NOT_STATIC / AMBIGUOUS / ASSOC_FAR / PCA_TOO_FEW_CELLS / NO_SUPPLEMENT`。

### D.4 编译结果

`cmake --build build -j4` → **`[100%] Built target low_detection`**（成功；期间修复了一处注释内 `*/` 提前闭合的编译错误）。

### D.5 运行验证状态

**未执行运行验证**（原因同 Phase 3-A：需要与 pcap 时间同步的有效定位；`debugEnable` 固定位姿不适用于含自车运动的验证）。因此 Case A~F 均**未验证**，报告中不给出伪造数据。

### D.6 已知问题

1. 历史 geometry = **最后一次匹配成功那一帧的原始检测几何**（非多帧稳定 footprint）→ 若最后一次观测本身也是 partial，补充能力有限。属第一版已知限制。
2. `kEnableHistoricalFusedOBB=false` 时**不写回几何**，因此当前对 UDP 内容**无影响**；只有打开 3-B-2 才会影响。
3. 未处理 `association_cluster_id < 0`（完全漏检）→ 属 Phase 3-C。
4. `cluster_tracks` 排他判定基于“本帧发起补充的 track 数>1”，若两 track 关联到同一 cluster 但其中一个是 MOVING，仍会判 AMBIGUOUS（保守，符合“宁可少补”）。

