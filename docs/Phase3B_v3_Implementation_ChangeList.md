# Phase 3-B v3 实现改动清单（7 点）

> 日期：2026-09-16
> 状态：**已实现并编译通过**（`[100%] Built target low_detection`，无新增告警）；运行验证待执行
> 设计说明见 `docs/Phase3B_TrackGeometryRefinement_Design_v3.md`
> 本文件只回答："这次到底改了什么、为什么必须改"。

---

## 0. 一句话概括

> 在 `SimpleTracker::update()` 与最终 Debug/UDP 输出之间插入 `RefineStaticObbGeometry()`：
> **只对 STATIC + 本帧匹配成功 + pose 有效 的 Track，用「当前 cells 的 PCA 主方向是否可观测」决定 yaw 来源
> （PCA / 历史 yaw / RAW），L/W 始终用最终 yaw 在当前 cells 上重新投影，center 始终为 RAW**；
> 结果只写入输出副本 `outTracks`，**Tracker 内部几何永远保持 RAW**。

编译结果：

```
[  8%] Built target test_coordinate_transform
[100%] Built target low_detection        (result code: 0)
```

---

## 1. 新增 `include/static_obb_refinement.h`（header-only）

**改了什么**
新增 Phase 3-B 的完整策略内核（纯函数、零副作用、可单测）：

| 内容 | 说明 |
|---|---|
| `kObbRefineEnable` / `kObbRefineVerbose` | A/B 总开关 + 日志开关 |
| `kObbEigenRatioThreshold = 4.0f` | **主阈值**：PCA 主方向可观测性（λ1/λ2 下限） |
| `kObbYawJumpMaxDeg = 30.0f` | 连续性阈值：PCA yaw 与历史 yaw 的最大允许无向轴夹角 |
| `kObbLambdaDegenerate = 1e-9f` | 数值退化保护（非调参项） |
| `kObbMinCellsForPCA = 3` | 与上游 `ComputeClusterOBB` 的 `n<3` 一致（必要非充分） |
| `NormalizeYaw180()` | 无向轴角度归一化到 `[-π/2, π/2)`（弧度，NaN/Inf 保护） |
| `AngularDistance180()` | 无向轴夹角，返回 `[0, π/2]`；显式覆盖 `θ ≡ θ±180°` 与 eigenvector ±v 符号歧义 |
| `IsOrientationObservable(λmax, λmin, n_cells)` | 只判断"cell 几何能否提供有区分度的主方向" |
| `DecideStaticObbYaw(...)` | 决策表：PCA / HISTORY / RAW + 是否需要更新历史 |
| `ComputeYawAlignedExtents(centers, yaw, L, W)` | 用给定 yaw 在当前 cells 上重投影求 L/W |
| `BuildObbCorners(center, yaw, L, W, out[4])` | 重建 4 角点（顺序与 `ComputeClusterOBB` 一致） |
| `ObbYawSource` / `ObbRefineSkip` 枚举 | 封闭决策码，便于脚本聚合 |

**为什么必须**
1. 把"可观测性 + 无向角工具 + 决策 + 几何重建"收敛到一个可独立验证的单元，避免散落在 `SuTengDriver.cpp` 里；
2. yaw 的 180° 等价与 `fabs(yaw1-yaw2)` 陷阱是**最容易写错**的地方，必须只有一处实现；
3. 项目已有 `track_motion_state.h` / `historical_feedback.h` / `historical_geometry.h` 三个 header-only 模块，延续同一风格。

---

## 2. `include/ElevationMapGroundFilter.h`：`GridCluster` 新增 3 个字段

```cpp
float obb_lambda_max = 0.0f;              // 主轴特征值 λ1
float obb_lambda_min = 0.0f;              // 次轴特征值 λ2
float obb_orientation_confidence = 0.0f;  // (λ1-λ2)/(λ1+λ2)，仅诊断用
```

**为什么必须**
可观测性判断需要 λ1/λ2，而当前 `GridCluster` 里没有任何特征值信息（只有 center/length/width/angle/corners/has_obb）。
**只新增字段，不改变任何既有字段语义与初始化行为。**

---

## 3. `include/ElevationMapGroundFilter.cpp`：`ComputeClusterOBB()` 保存 eigenvalues（3 行）

```cpp
// 仅新增“保存”，不改变本函数任何 OBB 计算与输出
cluster.obb_lambda_max = lambda_max;
cluster.obb_lambda_min = lambda_min;
cluster.obb_orientation_confidence = orientation_confidence;
```

**为什么必须**
`lambda_min / lambda_max / orientation_confidence` **上游已经算好了**（只是作为局部变量被丢弃）。
按"最小改动、复用现有 PCA、不重复计算"的原则，这里只做保存，**没有重写 OBB 算法**（协方差、特征分解、投影 min/max、角点构造全部原样）。

---

## 4. `include/historical_feedback.h`：`MapAnchoredTrack` 新增历史 yaw 记忆（2 字段）

```cpp
// Phase 3-B: STATIC OBB 的“已验证 yaw”记忆（地图系，无向长轴，弧度）
//   - 观测到可靠 PCA 主方向时写入；不可观测或与历史明显冲突时不写入（保留上一次可靠值）
//   - UpdateMapAnchors() 不修改这两个字段（保证锚点始终基于 RAW 几何）
bool  has_static_yaw     = false;
float static_yaw_map_rad = 0.0f;
```

**为什么必须**
1. 历史 yaw **必须来自"上一帧之前被接受的稳定方向"**，不能从 `map_corners` 反推 —— 那等于用"本帧 RAW 的 yaw"当历史，fallback 立刻失效；
2. 项目已有"地图系 Track 几何侧表"（`MapAnchoredTrack` / `m_mapTracks`），**只做最小扩展**，不创建第二套历史结构（§23 要求）；
3. 存**地图系**（而非雷达系）：自车 heading 变化时雷达系 yaw 会跟着变，只有地图系 yaw 才是静止目标的稳定属性。

---

## 5. `include/track.cpp`：V1 matched 分支 +1 行（暴露本帧匹配的 Cluster id）

```cpp
// Phase 3-B: 暴露“本帧匹配到的 Cluster id”（与 V2 update 中已有写法一致）。
vtrackings[i].cluster_id = detections[best_det_idx].cluster_id;
```

**为什么必须**
1. `TrackedObstacle` **没有 cells**（`ConvertClustersToTrackedObstacles` 只传 center/size/corners），
   而 "L/W 必须用当前 cells 重算" 和 "λ1/λ2 判断" 都需要**本帧匹配的那个 `GridCluster`**；
2. V1 的 matched 分支原本**不回写** `cluster_id`（出生时写一次后就陈旧了），miss 帧更不能用 ⇒ 必须每帧匹配时同步；
3. 写法与 V2（`track.cpp:680`）完全一致，**只暴露匹配结果，不改匹配/门限/代价函数/删除策略**。

---

## 6. `include/SuTengDriver.h`：声明精修接口

```cpp
void RefineStaticObbGeometry(const std::vector<GridCluster>& clusters,
                             const LocalizationManager::Pose& pose,
                             bool pose_valid,
                             std::vector<TrackedObstacle>& outTracks);
```

**为什么必须**
接口需要在 `ProcessPcapCloud()` 中调用，并按 §20/§25 的约定把结果交给 `outTracks`（而不是写回 tracker）。

---

## 7. `include/SuTengDriver.cpp`：接线 + 实现 + 输出换源

| 子项 | 内容 |
|---|---|
| ① include | `#include "static_obb_refinement.h"` |
| ② 启用 `UpdateMapAnchors(loc_pose, have_pose)` | 用 **RAW 几何**刷新 map 锚点；**必须放在 refinement 之前**（§21 时序），杜绝 `Refined → Anchor → Association → Refined` 反馈回路 |
| ③ 新增输出副本 | `std::vector<TrackedObstacle> outTracks; RefineStaticObbGeometry(outputClusters, loc_pose, have_pose, outTracks);` |
| ④ 消费对象换源（3 处） | `DrawMapAndAllOverlay(..., outTracks, ...)`、`m_visBuffer.Publish(..., outTracks)`、`ConvertTrackToS2ObstacleBox(outTracks, ...)` |
| ⑤ 实现 `RefineStaticObbGeometry()` | 逐 Track：STATIC 门 → `lastSeen==0` 门 → Cluster 查找 → `has_obb` 门 → `pose_valid` 门 → 读历史 yaw → 决策 → 用 final yaw 重投影 L/W → 重建 corners → 写 `outTracks[i]` → 更新历史 yaw → 日志 |
| ⑥ 两个 file-local 工具 | `RadarYawRadToMapYawRad()` / `MapYawRadToRadarYawRad()`：**两点法**经 `CoordinateTransformer` 求方向差，自动包含 `heading + gridHeadingOffsetDeg`，避免手推符号出错 |

**为什么必须**

1. **RAW/Refined 隔离**：原先只有一份几何（`m_tracker.vtrackings`）直接喂 Debug/visBuffer/UDP。
   引入副本后，Phase 3-B **在结构上不可能**污染 association / 速度 / age / lastSeen / motion_state / map anchor / 删除策略，A/B 也能同帧对比；
2. **时序**：map 锚点必须基于 RAW 几何（§21），否则历史关联会被 refined 几何影响；
3. **pose 的唯一用途**是 `map↔radar` 的 yaw 投影（复用每帧已读取的 `loc_pose/have_pose`），**不引入任何新的 Localization 逻辑**；pose 无效则该帧输出 RAW。

---

## 附 A：数据流 before / after

**Before**

```
Cluster(cells, AABB, OBB, [λ1,λ2 被丢弃])
   → ConvertClustersToTrackedObstacles          (cells 丢失)
   → SimpleTracker::update                      → m_tracker.vtrackings（唯一几何）
   → DrawMapAndAllOverlay / visBuffer / UDP     全部直接读 m_tracker.vtrackings
```

**After**

```
Cluster(cells, AABB, OBB, λ1/λ2 已保存)
   → ConvertClustersToTrackedObstacles
   → SimpleTracker::update                       → m_tracker.vtrackings = RAW（永不写回 refined）
   → UpdateTrackMotionStates   (Phase 3-A → motion_state)
   → UpdateMapAnchors          (map 锚点/角点, 只用 RAW 几何)
   → RefineStaticObbGeometry:
        outTracks = vtrackings 副本
        STATIC + matched + pose valid:
            yaw     = PCA | HISTORY | RAW
            L/W     = 用 final yaw 在当前 cells 上重新投影
            center  = RAW center
            corners = 重建
        → 只写 outTracks[i].depth / width / corners
   → Debug / visBuffer / UDP      读 outTracks
```

---

## 附 B：RAW geometry 与 Refined geometry 的边界

| 项 | RAW（`m_tracker.vtrackings`） | Refined（`outTracks`） |
|---|---|---|
| center `pos_x/pos_y/pos_z` | 始终 RAW | **恒等于 RAW**（本阶段不做位置修正） |
| `depth/width` | 本帧 PCA OBB 尺寸 | 用 final yaw 对当前 cells 投影得到的尺寸 |
| `corners` | 本帧 PCA OBB 角点 | `BuildObbCorners(RAW center, final yaw, refined L/W)` |
| `height` | RAW | RAW（不改） |
| 谁能改 | **只有 `SimpleTracker::update()`** | 只有 Phase 3-B |
| 谁消费 | Tracker 关联/速度/age/lastSeen、Phase 3-A、`UpdateMapAnchors` | Debug、`m_visBuffer`、UDP |

---

## 附 C：验证日志示例（格式 + 判读）

```text
# 3-cell L 型、无历史 yaw → 不凭空创造方向 → RAW
[GeomRefine] Track=10010 state=STATIC cells=3 lambdaMax=0.003333 lambdaMin=0.001111 eigenRatio=3.000 observable=0 skip=NO_USEFUL_YAW (无历史 yaw，不凭空创造方向)

# 变成长条（可观测）+ 首次初始化历史 → yawSource=PCA
[GeomRefine] Track=10010 state=STATIC cells=4 lambdaMax=0.012500 lambdaMin=0.000000 eigenRatio=inf pcaYaw=88.9 histYaw=-999.0 yawDelta180=0.0 observable=1 yawSource=PCA finalYaw=88.9 rawL=0.30 rawW=0.00 refinedL=0.30 refinedW=0.00 axisMismatch=0 center=(3.42,0.88)

# 变成 2×2（不可观测）→ 回退历史 yaw；L/W 仍来自当前 cells
[GeomRefine] Track=10010 state=STATIC cells=4 lambdaMax=0.002500 lambdaMin=0.002500 eigenRatio=1.000 pcaYaw=41.2 histYaw=88.9 yawDelta180=47.7 observable=0 yawSource=HISTORY finalYaw=88.9 rawL=0.14 rawW=0.14 refinedL=0.20 refinedW=0.10 axisMismatch=0 center=(3.44,0.87)

# 可观测但与历史冲突 82° → 保守使用历史 yaw
[GeomRefine] Track=10010 state=STATIC cells=3 lambdaMax=0.006667 lambdaMin=0.000000 eigenRatio=inf pcaYaw=-70.3 histYaw=11.9 yawDelta180=82.2 observable=1 yawSource=HISTORY finalYaw=11.9 rawL=0.20 rawW=0.08 refinedL=0.17 refinedW=0.09 axisMismatch=1 center=(3.41,0.88)

# 每帧汇总（与 verbose 开关无关）
[GeomRefineSummary] tracks=12 static=7 refined=6 yawPca=4 yawHistory=2 raw=1 notStatic=3 noMatch=1 noCluster=0 noObb=0 noPose=0 noUsefulYaw=1 axisMismatch=0
```

判读要点：

- `yawSource=PCA / HISTORY / RAW` 直接回答"为什么这一帧角度（没）变化"；
- `refinedL/refinedW` 若随 cells 变化而 `finalYaw` 不变 ⇒ 说明"yaw 稳定、尺寸仍跟随当前帧"（设计目标）；
- `center=` 应恒等于同帧 tracker 日志的 `Track id: … Center(...)`。

---

## 附 D：本次**未**改动（刻意保持）

```
Ground Filter 算法 / Grid resolution / ROI / BFS / Region Growing
HDMap / Localization / CoordinateTransformer 数学
UDP 协议 / newS2AviodObject / S2obstacleBox
SimpleTracker 的匹配、门限、代价函数、删除策略
Phase 3-A 的 UpdateTrackMotionStates()
ApplyHistoricalGeometry()（保持注释状态，旧 cell union 方案不启用）
ElevationMapGroundFilter 的 OBB 计算逻辑（仅新增保存 eigenvalues）
```

---

## 附 E：待确认的 3 点

1. **`axisMismatch`（`refinedL < refinedW`）**：采用 HISTORY yaw 且该 yaw 与当前 cells 长轴不一致时，按公式 `length = max_u - min_u` 会出现 `length < width`。
   当前**不做交换**（交换等于变相改变 yaw，违背"保持历史 yaw"），只记日志与计数。若下游假定 `depth ≥ width`，请告知后再定策略。
2. **MOVING 时清除历史 yaw 记忆**：避免 `MOVING → STATIC` 后误用运动前的陈旧方向；`UNKNOWN` 不清除。
   若希望 MOVING 也保留记忆，可改回。
3. **两个阈值需要标定**：`kObbEigenRatioThreshold = 4.0`（实测双峰间隙 `[3.57, 6.00]`）、`kObbYawJumpMaxDeg = 30`（尚无实测分布）。
   建议先用日志里 `eigenRatio` / `yawDelta180` 的分布标定。
