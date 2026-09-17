# Phase 3-B：Static Track Geometry Stabilization（设计文档 v2 · 收敛版）

> 状态：**纯设计文档，未修改任何源码**
> 适用工程：`/home/zyl/echiev_low_lidar_detection`
> 版本：**v2（2026-09-15 架构收敛）**，替代 v1 的 "Center + L/W + Yaw + confidence + historical geometry" 方案。
> v1 中被删除的机制见 §0.3；`Historical Cell Fusion` 的废弃声明见 §27。

---

## 0. 结论先行

### 0.1 一句话定义

> **Phase 3-B = Static Track Geometry Stabilization**
> 当 Phase 3-A 已确认某 Track 为 `STATIC` 后，利用该 Track 的历史几何先验，抑制稀疏 LiDAR 检测造成的 `Length / Width / Yaw` 抖动，
> 并用 **RAW Center + refined L/W + refined Yaw** 重新计算 OBB 四角点，使最终输出的 corners 更稳定。

### 0.2 职责边界（三个 Phase 不重叠）

```
Phase 3-A：判断目标是不是 STATIC / MOVING / UNKNOWN（地图系位置证据 + 迟滞状态机）
Phase 3-B：只对已经确认 STATIC 的 Track 做 L/W/Yaw 几何稳定（本文件）
Phase 3-C：以后再考虑"检测丢失"时的位置/几何预测（coasting）—— 本 Phase 明确不做
```

Phase 3-B **不再承担任何运动状态判断**，也**不再做任何位置/中心修正**。

### 0.3 v1 → v2 的删除清单（本次收敛的核心）

| v1 机制 | v2 处置 |
|---|---|
| Center EMA / Center prior smoothing | **删除**。Center 完全使用 Tracker 的 RAW 值 |
| Center reject / clamp / max correction（`kGeomCenterRejectM`、`kGeomMaxOffsetM`） | **删除**（标记废弃） |
| `kGeomAlphaCenterBase` 等 Center 参数 | **删除** |
| `meas_conf` / `prior_conf` / `final_conf` / `size_conf` 参与控制 | **删除**。confidence 不参与任何分支判定 |
| EMA 稳态方差理论（$\sigma_{out}$、$\sqrt{\alpha/(2-\alpha)}$ 等） | **删除**。只保留工程语义：α 大 = 更信当前，α 小 = 更信历史 |
| 三维联合一致性打分矩阵 | **简化为**两条独立规则：尺寸 gate + yaw gate |
| refined 结果写回 `m_tracker.vtrackings`（v1 方案 A） | **禁止**。改为独立 `TrackGeometryResult` + 输出副本，Tracker State 永远 RAW |
| AABB 作为几何测量 / 门控输入 | **删除**。本 Phase 不融合 AABB，也不把 AABB 传入 Track |
| Historical Cell Union / Footprint Fusion | **废弃方向**（§27），仅保留旧代码作 rollback 参考 |

### 0.4 最终运行逻辑（唯一版本）

```
Current Cluster
      ↓
SimpleTracker::update()
      ↓
UpdateTrackMotionStates()          ← Phase 3-A，已存在，不修改
      ↓
┌──────────────┬──────────────┬──────────────┐
│  UNKNOWN     │   MOVING     │   STATIC     │
├──────────────┼──────────────┼──────────────┤
│ 初始化/更新  │ 不使用 history│ L/W refinement│
│ geometry     │ 几何(不建立、 │ Yaw refinement│
│ prior        │ 不更新)       │ Center = RAW  │
│ 输出 RAW     │ 输出 RAW      │               │
└──────────────┴──────────────┴──────────────┘
                                     ↓
                        Recompute OBB corners
                        (RAW Center + refined L/W + refined Yaw)
                                     ↓
                          TrackGeometryResult
                                     ↓
                              Debug / UDP
```

三条硬规则：

```
UNKNOWN ：可以 Initialize / Update geometry prior，但【绝对不能】用它修改输出 → 输出 RAW
MOVING  ：完全不使用历史几何（不建立、不更新、不施加）→ 输出 RAW
STATIC  ：Center 不动；只稳定 L/W 与 Yaw；然后重建 corners
```

### 0.5 必须先具备的前置条件

1. **Phase 3-A 生效**：`ProcessPcapCloud` 中 `UpdateTrackMotionStates(loc_pose, have_pose, rec_timestamp_ms)`（在 `m_tracker.update()` 之后）必须处于启用状态 —— 实施前先确认。
2. **本帧匹配到的 Cluster（仅用于 `n_cells` / 几何有效性，可选）**：见 §17。
   **注意：RAW L/W/Yaw 本身不需要它** —— 匹配帧的 `track.depth/width/corners` 就是本帧检测几何（§4.1）。

### 0.6 Tracker 与 Output 的隔离（硬性架构原则）

```
SimpleTracker  →  Raw Track State（center / L / W / corners / vx / vy / age / lastSeen / motion_state）永远 RAW
Refinement     →  TrackGeometryResult（独立结构）
Debug / UDP    →  TrackGeometryResult（refinement 有效时）否则 raw geometry
```

**禁止把 refined geometry 写回 `SimpleTracker` 的内部状态**（理由与可诊断性要求见 §12）。

---

## 1. 当前 Geometry Pipeline（代码级，已核实到行）

```
ProcessPcapCloud()                                     SuTengDriver.cpp:888
  ├─ PointCloudTransform()
  ├─ ProcessWithObstacleDetection()                    (调用 :1013 → 定义 ElevationMapGroundFilter.cpp:964)
  │     … AnalyzeVerticalOccupancy
  │     → ClusterObstacleGrid()                        (:1977, 8邻域 BFS, ≥ min_cluster_cells)
  │          └─ BuildClusterFromCells()                (:2145)  AABB(±0.05 padding) + center + length/width
  │                └─ ComputeClusterOBB()              (:2377)  2D PCA on cell centers
  │                     → obb_center / obb_length / obb_width / obb_angle / obb_corners[4]
  │                     → orientation_confidence  ← 算了但没存 (:2447)
  ├─ HDMapFilter.filterClusters()                      (只打 in_road 标签)
  ├─ DrawClusterOverlay(outputClusters)                (:1050)   ← Cluster 层 Debug（另一图层，不受本 Phase 影响）
  ├─ ConvertClustersToTrackedObstacles()               (:1846)   OBB 优先 / AABB 回退；Rotation 被 memset(0) (:1899)
  ├─ m_tracker.update()                                (:1058) → track.cpp:258 (V1 贪心)
  ├─ UpdateTrackMotionStates()                         (:1078)   ← Phase 3-A
  ├─ [已注释] Phase 2 feedback / UpdateMapAnchors       (:1095)
  ├─ [已注释] ApplyHistoricalGeometry()（旧 3-B cell 补充）(:1101)
  ├─ Debug: DrawMapAndAllOverlay / DrawTrackOverlay(…, m_tracker.vtrackings, …)   (:1122 / :1130)
  ├─ m_visBuffer.Publish(…, m_tracker.vtrackings)      (:1159) → main.cpp:603 可视化
  └─ [onlineModel] ConvertTrackToS2ObstacleBox(m_tracker.vtrackings) → UDP   (:1168)
```

### 1.1 数据流一句话

```
GridCluster(AABB+OBB) ──OBB优先──▶ detection ──tracker──▶ Track.几何(=本帧检测几何 / miss 时保持) ──▶ UDP / Debug
```

---

## 2. 当前 OBB 生成方式与抖动来源

`ComputeClusterOBB`（`ElevationMapGroundFilter.cpp:2377`）：

1. 取簇内每个 cell 的**中心点** $(x_i,y_i)$（**不含半格 padding**）；
2. 均值 $\mu$、协方差 $C$、`SelfAdjointEigenSolver`（$\lambda_0\le\lambda_1$）；
3. 全部点投影到主/次轴取 min/max ⇒ `obb_length`（长轴）、`obb_width`（短轴）、`obb_angle`；
4. 4 角点顺序固定：`(min_pri,min_sec) → (max_pri,min_sec) → (max_pri,max_sec) → (min_pri,max_sec)`
   = **左下 → 右下 → 右上 → 左上**，且 `corner0→corner1` 沿长轴方向（`:2543`）。

### 2.1 抖动来源（Phase 3-B 的靶子）

| 来源 | 机制 | 量级 |
|---|---|---|
| G1 cell 集合增减 | 3 cells ↔ 4 cells，投影 min/max 端点跳 1 格 | L/W 跳 **0.1m**（对 0.1~0.2m 的目标即 50%~100%） |
| G2 cell 中心量化 | 所有测量都是 0.1m 网格上的离散值 | L/W/yaw 全部离散化 |
| G3 PCA 对少量点敏感 | 3~4 个点，删掉 1 点等于换掉 25% 的分布 | yaw 可跳几十度 |
| G4 近正方形 | $\lambda_1\approx\lambda_2$，主轴方向由数值噪声决定 | yaw 随机翻转 |
| G5 对角/斜格 | 2~3 个对角 cell：PCA "各向异性极高"，但斜向可能只是采样假象 | yaw 出现 45°/135° 类值 |
| G6 视角/距离变化 | 自车前进，同一目标采样格子换一批 | 上述全部叠加 |

> 关键认识：对低矮目标（3~4 cells、0.1m 网格），**L/W 与 yaw 的抖动量级与目标尺寸同阶**，
> 因此本 Phase 的价值集中在 L/W 与 Yaw —— 它们直接决定 corners 的位置。

---

## 3. OBB 与 AABB 的定义差异（仅作说明，本 Phase 不融合 AABB）

| 量 | 定义 | 例（3 个共线 cell） |
|---|---|---|
| AABB length | `max_x-min_x`，**含 ±0.05 半格 padding** | 0.30 |
| OBB length | 主轴投影跨度，**只覆盖 cell 中心** | 0.20 |

**处置**：本 Phase 的 L/W 先验与输出**统一采用 OBB / cell-center 定义**（与现有 `track.depth/width` 语义一致），因此不会引入系统性尺寸变化。
AABB **不参与**本 Phase 的 refinement、门控与输出（理由：AABB 不传到 Track；两种定义混用会引入 0.1m 级系统偏差）。

---

## 4. Tracker / Track 数据结构（与本设计相关的部分）

```cpp
struct TrackedObstacle {
  int   cluster_id;              // ⚠ V1 只在出生时写入；matched 分支【不回写】
  int   id;                      // 10000 起
  float pos_x, pos_y, pos_z;     // 雷达系 RAW center
  Point2D corners[4];            // RAW corners（matched 时 = 本帧检测 corner；miss 时 = 上一帧值）
  float depth, width, height;    // RAW：depth = OBB 长轴, width = OBB 短轴
  int   age, lastSeen;
  float Translation[3], Rotation[9];   // ⚠ 真实 pipeline 里 Rotation 被 memset(0) → Track 层没有 yaw 字段
  float vx, vy;
  MotionState motion_state;      // Phase 3-A
  bool has_map_pos; float map_x, map_y, ...;   // Phase 3-A
  int   motion_static_run, motion_moving_run;  // Phase 3-A
  /* 手写拷贝构造 + 拷贝赋值（新字段必须同步，历史踩坑） */
};
```

`SimpleTracker`（`track.cpp:258`，V1）：贪心匹配，`match_threshold = 0.5m`，代价为 `|pos+v·dt − detection_pos|`；
`matched` 分支用检测几何覆盖 `pos_z/depth/width/height/corners`（并 `age++`、`lastSeen=0`）；
`miss` 分支只 `lastSeen++`；空 detections 分支 `pos/corners += v·dt`；`removeLostTargets()` 删除 `lastSeen>10`。

### 4.1 可直接读取的 RAW 几何（重要结论）

> 对 `lastSeen == 0` 的 Track：
> `depth / width` = 本帧匹配检测的 OBB 尺寸；`corners` = 本帧检测的 OBB 角点。
> ⇒ **RAW L/W/Yaw 可以直接从 Track 自身读取，不需要访问 Cluster，也不需要修改 `track.cpp`。**

推荐做法（自洽，且避开上游分支不一致）：

```
yaw_raw = atan2(c1.y - c0.y, c1.x - c0.x)     // corner0→corner1 沿长轴
L_raw   = |c1 - c0|
W_raw   = |c2 - c1|
若 W_raw > L_raw：交换 L/W 并与 yaw_raw 一起 +90°     // 归一化为 L ≥ W
```

该做法对上游 `ComputeClusterOBB` 中极罕见的 `len_pri < len_sec` 分支（角点仍按主轴构造，而 `obb_angle` 取次轴）天然免疫；
若改读 `track.depth + obb_angle`，在该分支下会差 90°。

---

## 5. 问题定义：为什么目标是 corners

```
OBB corners = f(center, L, W, yaw)
```

低矮目标通常只有 3~4 个 grid cell，grid = 0.1m ⇒ L、W、yaw 都是"小样本 + 强离散"的量。
本 Phase **不修改 center**，所以：

```
稳定 L/W  +  稳定 Yaw   →   稳定 corners
```

> **Phase 3-B 的最终效果评价重点不是 "Center 是否更平滑"，而是 "OBB 四角点是否更稳定、角度翻转与尺寸跳变是否减少"。**
> 用直接平滑 center 去换取"看起来更稳"（v1 的做法）会把重心放错，并掩盖 L/W/yaw 的真实问题。

**目标**：对 `STATIC` + 本帧匹配成功的 Track，用历史几何先验稳定 `L / W / Yaw`，并重建 corners。

**非目标**：不做 cell union / footprint 融合；不做 center 修正；不做 miss 帧预测（Phase 3-C）；不做运动状态判断（Phase 3-A）；
不引入 Kalman / Hungarian / 新 tracker / 第二套 ID。

---

## 6. 为什么删除 Center Geometry Refinement

> Phase 3-A 的 `UpdateTrackMotionStates()` 已经在 **Map Frame** 中利用历史位置证据判定 `STATIC / MOVING / UNKNOWN`。
> 对已判定为 `STATIC` 的 Track，本 Phase 不再重复对位置做 EMA/平滑：
> 1. 会让 `MotionState` 与 `Geometry Refinement` 双重承担"位置稳定"职责，责任不清；
> 2. 会引入额外的位置滞后（对仍可能有少量真实运动的静止目标不利）；
> 3. 位置是否稳定，本质上是 Phase 3-A 的判定输出，不是几何精修的职责。

因此：

```
STATIC Track Center  →  直接使用 Tracker 的 RAW center（pos_x / pos_y），不做任何修改
```

由此也**不再需要** center prior / center gate / center clamp / center confidence，
以及 v1 中用于防止"锁死"的偏移上限（这些参数在 v2 中全部删除，见 §15.1）。

---

## 7. 运行逻辑：UNKNOWN / MOVING / STATIC 三分支

### 7.1 UNKNOWN 的逻辑（彻底修正 v1 的矛盾）

v1 文档存在自相矛盾的伪代码（"非 STATIC 就 continue" 却又要求 UNKNOWN 初始化 prior）。最终规则只有一条：

```
UNKNOWN
   ↓  current observation valid ?
   ↓ YES
可以 Initialize / Update Geometry Prior
   ↓
但 output 必须保持 RAW
```

逐帧示意：

```
Frame 1: UNKNOWN    raw geometry → Initialize prior          output = RAW
Frame 2: UNKNOWN    Update prior                             output = RAW
Frame 3: UNKNOWN    Update prior                             output = RAW
Frame 4: UNKNOWN → STATIC（Phase 3-A 判定）
Frame 5: STATIC     prior 已存在 → 启用 L/W/Yaw refinement    output = REFINED(corners)
```

伪代码必须显式拆成三分支（**不能**写成 `if (motion_state != STATIC) continue;`）：

```
if (motion_state == UNKNOWN)
{
    if (valid_raw_geometry)
        UpdateOrInitializeGeometryPrior(track, raw);   // 只养 prior
    OutputRawGeometry(track);                          // 输出绝不使用 prior
    continue;
}

if (motion_state == MOVING)
{
    OutputRawGeometry(track);                          // 不建立、不更新、不施加 prior
    continue;
}

// STATIC
RefineStaticTrackGeometry(track, raw, prior, result);
OutputGeometryResult(result);
```

### 7.2 MOVING 为什么连 prior 都不建立

运动目标的 L/W/Yaw 是**时变量**，其历史值没有"稳定先验"的语义；把它喂进先验只会污染数据。
Phase 3-A 已提供 `MOVING → STATIC` 的迟滞（连续 6 帧静止证据），目标真正停稳后仍会进入 STATIC 并从当时开始养 prior，代价很小。

### 7.3 STATIC + miss / pose 无效

```
STATIC 且 lastSeen != 0  → 无本帧匹配检测：输出 RAW（不做预测，Phase 3-C 再做）
STATIC 且 pose 无效      → 无法把 map 系 yaw prior 投影到当前雷达系：输出 RAW（§18）
```

---

## 8. Geometry Prior 设计（简化版）

### 8.1 结构（概念稿）

```cpp
// include/track_geometry_prior.h（header-only）
struct TrackGeometryPrior
{
    bool  valid       = false;   // 是否已初始化
    bool  established = false;   // 是否已建立（valid_frames >= kGeomMinValidFrames）
    int   track_id    = -1;

    // ---- 尺寸：目标自身量（OBB / cell-center 定义，与 track.depth/width 同定义）----
    float length = 0.0f;
    float width  = 0.0f;

    // ---- 朝向：唯一需要坐标系的量 → 存地图系 ----
    bool  has_yaw     = false;
    float yaw_map_deg = 0.0f;    // 折到 [-90, 90)，长轴无向

    // ---- 生命周期（只有计数，不做 confidence 加权）----
    int   valid_frames = 0;      // 已接受的有效观测帧数
    int   miss_frames  = 0;      // 连续未更新帧数
    int   grow_run     = 0;      // L/W 超门(变大)连续帧数
    int   shrink_run   = 0;      // L/W 超门(变小)连续帧数
    int   yaw_flip_run = 0;      // yaw 大角差连续帧数
    float yaw_flip_cand_deg = 0.0f;   // 上一帧的翻转候选角（雷达系）
    int   last_age     = -1;     // 与 track.age 对齐（防 nextTrackID 回绕导致的陈旧先验）
};
```

### 8.2 字段取舍（明确"存什么、不存什么"）

| 字段 | 存? | 理由 |
|---|---|---|
| `length` / `width` | ✅ | 目标自身量，与坐标系无关；refinement 的主角之一 |
| `yaw_map_deg` | ✅ | yaw 是唯一需要坐标系的姿态量，必须单独存、单独更新 |
| `valid_frames` / `miss_frames` | ✅ | 生命周期与 "established / stale" 判定（简单计数，不是权重） |
| `grow_run / shrink_run / yaw_flip_run` | ✅ | 超门后的连续帧确认计数（简单计数） |
| center（map 或 radar） | ❌ **不存** | Center 不 refinement，本 Phase 完全不需要 |
| AABB geometry | ❌ | 不融合（§3），也不参与门控 |
| corners / L/W 的历史序列 | ❌ | 与 (L,W,yaw) 冗余；存了会产生"两套几何不同步"的 bug 源 |
| cell 数历史（median/max/frequency） | ❌ | 会被误用成"补格子"（旧方向错误）；只允许作为**当帧** yaw 可靠性输入 |
| `meas_conf / prior_conf / final_conf / size_conf` | ❌ | 本 Phase 不使用 confidence 控制（§13） |
| `height / pos_z` | ❌ | Z 方向不动，沿用 RAW |

### 8.3 存哪里

| 方案 | 评价 |
|---|---|
| A. 塞进 `TrackedObstacle` | ❌ 污染 tracker 状态语义；且必须手改拷贝构造/赋值（历史踩坑） |
| **B. `SutengDriver` 侧表 `std::vector<TrackGeometryPrior> m_geomPriors`（按 `track_id`）** | ✅ **推荐**。与既有 `m_mapTracks` / `m_historicalGeometry` 同构；可一键停用；零侵入 `track.h` |
| C. 全局单例 | ❌ |

清理策略：每帧用 `vtrackings` 的 id 集合做 `erase-remove`；再加 `prior.last_age <= track.age` 单调性校验，
防 `nextTrackID > 50000 → 9999` 回绕（`track.cpp:467`）。

---

## 9. L/W Refinement 规则（简单、可解释、尺度相关）

### 9.1 统一约定

- L/W 定义与 `track.depth/width` 一致（OBB / cell-center，§3），并强制 `L ≥ W`；
- 只使用：**有效性 + gate + 历史帧计数 + 简单更新规则**；
- **不使用任何 confidence 控制**。

### 9.2 尺度相关的 gate（对低矮目标必须如此）

v1 的固定绝对门限（0.15m）对本项目的小目标失效：

```
W = 0.07m 时允许 ±0.15m  ⇒  实际允许尺寸变化超过 2 倍
```

因此 gate 必须**以相对量为主 + 一个小的绝对下限/上限**：

$$
\text{gate}_X=\text{clamp}\big(k_{rel}\cdot X^{prior},\; g_{min},\; g_{max}\big),
\qquad
\text{gate}_L=\text{gate}(L^{prior}),\quad
\text{gate}_W=\text{gate}(W^{prior})
$$

$k_{rel},g_{min},g_{max}$ 的**初值**见附录 A。**它们不是物理真值**，必须由现有 STATIC Track 日志的
$\lvert\Delta L\rvert,\lvert\Delta W\rvert$ 分布标定（§26.1）。

### 9.3 更新规则

| 情况 | 条件 | 动作 | 决策码 |
|---|---|---|---|
| 正常更新 | $\lvert\Delta L\rvert\le\text{gate}_L$ 且 $\lvert\Delta W\rvert\le\text{gate}_W$ | 简单平滑 $X_t=X_{t-1}+\alpha_s(X^{cur}-X_{t-1})$ | `SIZE_UPDATE` |
| 明显变大 | $\Delta L>\text{gate}_L$ 或 $\Delta W>\text{gate}_W$ | **不立刻跳过去**：`grow_run++`，用 $\alpha_{slow}$ 靠近；`grow_run ≥ K` 后才按 $\alpha_s$ 正常接受 | `SIZE_UPDATE_SLOW` → `SIZE_CONFIRM` |
| 明显变小 | $\Delta L<-\text{gate}_L$ 或 $\Delta W<-\text{gate}_W$ | 同上（`shrink_run`），且不得低于地板 `kGeomSizeMin` | `SIZE_UPDATE_SLOW` → `SIZE_CONFIRM` |
| 合法但无效 | NaN/Inf、$L>kGeomSizeMax$ 或 $<kGeomSizeMin$、$W<0$ | 该轴保持历史值 | `SIZE_HOLD` / `SIZE_REJECT_OUTLIER` |
| 无有效观测 | `lastSeen != 0`、或 RAW 几何不可用 | 保持历史值；输出走 §7.3 的 RAW | （`RAW_*`） |

补充：
- **允许正常误差**：$g_{min}$ 必须与"该轴的测量量化步长"匹配（cell-center 投影在斜向时步长可小至 0.0707m），所以 $g_{min}$ 不能设成 0；
- L 与 W **独立判定**：L 超门不影响 W 的正常更新；
- 两个方向矛盾（L 变大而 W 变小，或反之）时按"慢速更新"处理（视为形状不确定），不引入额外模型；
- α 的工程语义：**α 大 = 更相信当前观测；α 小 = 更相信历史 prior**（不做任何统计推导）。

### 9.4 日志示例（要求：delta/action 优先于 confidence）

```
Track 10010  motion=STATIC
[GeomRaw]     L=0.18  W=0.08
[GeomPrior]   L=0.15  W=0.07
[GeomDelta]   dL=+0.03  dW=+0.01  gate_L=0.075  gate_W=0.035
[GeomAction]  Size=SIZE_UPDATE
[GeomRefined] L=0.165 W=0.075
```

这类日志优先于任何 `size_conf=0.73` 形式的输出。

---

## 10. Yaw Refinement 规则（本 Phase 的另一半）

PCA 对稀疏 cell 极易产生 $\theta$ 与 $\theta\pm180^\circ$ 等价、以及大角度跳变，因此 yaw 需要：

```
yaw 历史 prior
+ 无向角归一化（θ ≡ θ±180°）
+ yaw jump gate
+ HOLD
+ 连续多帧确认
+ 简单平滑
```

### 10.1 可靠性判定（只用于"能不能信当前 yaw"）

| 条件 | 含义 | 初值 |
|---|---|---|
| Y1 `n_cells ≥ kGeomYawMinCells` | 排除 2~3 cell 的假各向异性（G5） | 4 |
| Y2 `L_cur ≥ kGeomYawMinExtent` | 长轴至少要有 2 格以上的空间延展 | 0.15m |
| Y3 各向异性 $a=\dfrac{L-W}{L+W}\ge k_{aniso}$ | "近正方形不提供可靠 yaw"（v1 用 AABB 各向异性表达同一思想；现改用 **OBB 的 L/W**，因为 AABB 未传到 Track，且 OBB 对斜向目标更正确） | 0.3 |

- `orientation_confidence`（上游 PCA 的 $(\lambda_1-\lambda_2)/(\lambda_1+\lambda_2)$）**只作为 diagnostic log**，
  **不作为 gate、不作为分支条件、不作为调参依据**；
- Y1 需要 cell 数 ⇒ 需要访问本帧匹配的 Cluster（§17）；若采用"零 Tracker 改动"变体，则 Y1 缺省通过，仅用 Y2 + Y3。

### 10.2 无向角处理

先折叠到 $[-90^\circ,90^\circ)$：

$$\text{fold}(\theta)=\theta-180^\circ\left\lfloor\frac{\theta+90^\circ}{180^\circ}\right\rfloor$$

**禁止**对角度做线性平均（$0.5\cdot170^\circ+0.5\cdot(-170^\circ)=0^\circ\ne180^\circ$）。
平滑必须用双角向量法（把方向当无向轴处理）：

$$X=\sum_i w_i\cos 2\theta_i,\quad Y=\sum_i w_i\sin 2\theta_i,\quad
\theta_{out}=\tfrac12\operatorname{atan2}(Y,X)\ \text{再 fold}$$

### 10.3 决策规则

```
if (!reliable(Y1 ∧ Y2 ∧ Y3)):
      yaw_out = prior.has_yaw ? θ_prior_radar : θ_cur
      prior.yaw 不更新
      action = YAW_HOLD(reason=UNRELIABLE)      # 若无 prior → 输出 RAW 的 yaw
else:
      Δ = fold(θ_cur − θ_prior_radar)
      if (!prior.has_yaw):
          prior.yaw = toMap(θ_cur)              → action = YAW_INIT
      elif (|Δ| ≤ kGeomYawGateDeg):             # 正常变化 → 简单平滑
          θ_out = UndirectedBlend(θ_prior_radar, θ_cur, kGeomYawAlpha)
          prior.yaw = toMap(θ_out)              → action = YAW_UPDATE
      else:                                     # 明显 jump → 先 HOLD；连续 K 帧一致才接受
          if (|fold(θ_cur − yaw_flip_cand_deg)| ≤ kGeomYawCandTolDeg) yaw_flip_run++
          else                                                       yaw_flip_run = 1
          yaw_flip_cand_deg = θ_cur
          if (yaw_flip_run ≥ kGeomYawFlipConfirm):
              prior.yaw = toMap(θ_cur); yaw_flip_run = 0   → action = YAW_ACCEPT_FLIP
          else:
              θ_out = θ_prior_radar                       → action = YAW_HOLD(reason=JUMP)
```

要点：
- `YAW_HOLD` 只丢弃"这一个方向值"，**不影响 L/W 的更新**（逐量独立）；
- 连续 K 帧支持新方向 ⇒ 真实形状/姿态变化仍会生效（噪声赢不了，真变化不会被永久封锁）；
- 一旦有可靠 yaw，prior 不会因为单帧不可靠而失效（只是没有新证据）。

### 10.4 案例验证（原始诉求）

```
Prior  : L=0.20  W=0.08  yaw_radar ≈ 12°
Current: L=0.14  W=0.07  yaw_cur   = -70°
```
- 若 `n_cells=2`：Y1 否决 ⇒ `YAW_HOLD` ⇒ 输出 yaw ≈ 12°，而 L/W 仍按 §9 正常更新 ✅
- 若 `n_cells=3`：Y1 否决（阈值为 4）⇒ 同上 ✅
- 若 `n_cells≥4` 但近方形（$a<0.3$）：Y3 否决 ⇒ 同上 ✅
- 若 `n_cells≥4` 且各向异性足够：$|\Delta|=82^\circ>25^\circ$ ⇒ 进入翻转确认 ⇒ 前 2 帧仍输出 12°，
  第 3 帧若持续 -70°（且候选角一致）才接受 ⇒ **噪声不会赢，真实变化会生效** ✅

---

## 11. Geometry Prior 生命周期

```
Track 出生 (age=0, lastSeen=0)
   │  prior 不存在 → 输出 RAW
   ▼
首个有效观测 (lastSeen==0 && valid_raw_geometry)
   │  INITIALIZE：L/W = raw；yaw = raw(map)；valid_frames=1；established=false
   │  ⚠ UNKNOWN 也允许执行这一步（只养 prior，输出仍 RAW）
   ▼
后续有效观测
   │  UPDATE：L/W 按 §9；yaw 按 §10；valid_frames++
   │  valid_frames >= kGeomMinValidFrames(=2) → established = true
   ▼
STATIC && established && pose ok && lastSeen==0
   │  → 执行 refinement（产出 TrackGeometryResult）
   ▼
miss 帧 / pose 无效 / MOVING
   │  HOLD：不更新任何几何量；miss_frames++
   │  输出 RAW（不做预测 → Phase 3-C）
   ▼
miss_frames > kGeomPriorMaxMissFrames(=5) → STALE
   │  停止 refinement（输出 RAW）；prior 保留（track 复活可继续）
   ▼
Track 被 removeLostTargets 删除 (lastSeen>10) → 侧表 erase
```

### 11.1 "什么算有效观测"

```
valid_obs = (lastSeen == 0) && finite(L_raw, W_raw) && (L_raw >= kGeomSizeMin) && (W_raw >= 0)
```
- 不引入额外年龄门槛：Phase 3-A 的 `UNKNOWN→STATIC` 已要求 4 帧连续静止证据，天然保证 STATIC 时 prior 已有多次更新；
- **不需要**"PCA 置信度足够"才能初始化：低置信度只影响 yaw 是否更新，不应阻止 L/W 初始化；
- 若可访问 Cluster，额外校验 `has_obb && n_cells ≥ 3`（`min_cluster_cells=3` 使其近乎恒真）。

---

## 12. Tracker State 与 Geometry Result 的彻底隔离

### 12.1 架构

```
SimpleTracker
    → Raw Track State（raw center / raw geometry / raw association / raw velocity / raw age / raw lastSeen）

Geometry Refinement
    Raw Track  +  Geometry Prior  →  TrackGeometryResult

Debug / UDP
    → TrackGeometryResult（refinement 有效时）
    → 否则 raw geometry
```

**硬约束**：

```
Geometry Refinement 只能影响最终 Debug / UDP 输出
```
不能影响：

```
SimpleTracker association / velocity / age / lastSeen / cluster matching / motion state / track deletion
```
也不能：

```
refined center → 下一帧 tracker prediction
refined yaw    → 下一帧 association
refined L/W    → cluster matching
```

### 12.2 为什么必须隔离（可诊断性优先）

A/B 必须能区分"Raw Track 本身就有问题"与"Geometry Refinement 引入了问题"：

```
RAW    : yaw = 5 → 7 → 6 → 8
REFINED: yaw = 6 → 6 → 6 → 7        ⇒ refinement 有效

RAW    : yaw = 5 → 6 → 5 → 6
REFINED: yaw = 5 → 20 → 4 → 18      ⇒ Phase 3-B 有问题（一眼可见）
```
如果 refined 结果写回 Track，上述两类原因会被混在一起，实验设计失效。

### 12.3 实现方式（推荐：输出侧使用"副本"）

```cpp
// ProcessPcapCloud 内，tracker 之后
std::vector<TrackedObstacle> outTracks = m_tracker.vtrackings;   // 拷贝（RAW 快照）
ApplyGeometryResults(outTracks, m_geomResults);                  // 只改副本的 depth/width/corners

// 之后所有"对外"消费都使用副本
m_debugViewer->DrawMapAndAllOverlay(..., outTracks, ...);        // 或 DrawTrackOverlay
m_visBuffer.Publish(pGroundCloud, pObstacleCloud, outTracks);
m_pElevationMapGroundFilter->ConvertTrackToS2ObstacleBox(outTracks, rec_timestamp_ms, outputObject);  // UDP
```

优点：
1. **Tracker State 结构上不可能被污染**（写的是副本）；
2. 不需要改 `track.h`（不加 `refined_*` 字段）、不需要改 `ElevationMapGroundFilter`、不需要改 `DebugViewer` 签名；
3. A/B 天然可行：`outTracks`（refined）与 `m_tracker.vtrackings`（raw）同帧并存，可直接对比；
4. ID 连续性等 Tracker 行为**按构造保持不变**（§26.5 的回归指标不可能因本 Phase 变化）。

代价：每帧一次 `std::vector<TrackedObstacle>` 拷贝（≤20 个元素；`TrackedObstacle` 的手写拷贝构造已正确处理 `unique_ptr`/vector 成员）。

---

## 13. 删除复杂 Confidence 体系

```
meas_conf / prior_conf / final_conf / size_conf   →   全部删除
confidence 不参与任何 refinement 控制
```

保留为 **diagnostic log only**：

- `orientation_confidence`（上游 PCA 已计算）—— 仅在需要时存为 `GridCluster` 字段或按需重算，用于观察，
  **不作为 yaw gate、不作为分支条件、不作为调参依据**；
- `n_cells`（当帧 cell 数）—— 只作为 yaw 可靠性 Y1 的输入，**不进入任何"历史统计/补格"逻辑**。

调试解释链统一为：

```
raw → prior → delta → threshold → action → refined
```

---

## 14. 日志设计（围绕"跳变"而不是 confidence）

```
[GeomRaw]     Track=10010 state=STATIC cells=3 yaw_conf_diag=0.91
              Center=(3.41,0.88) L=0.20 W=0.08 yaw=-70.3
[GeomPrior]   L=0.20 W=0.08 yaw_radar=11.9 (yaw_map=101.9) frames=8 miss=0
[GeomDelta]   dL=0.00 dW=0.00 dYaw=-82.2
[GeomAction]  Size=SIZE_UPDATE  Yaw=YAW_HOLD(reason=JUMP, flip_run=1/3)
[GeomRefined] Center=(3.41,0.88) L=0.20 W=0.08 yaw=11.9
[GeomCorners] c0=(3.31,0.83) c1=(3.51,0.87) c2=(3.51,0.93) c3=(3.31,0.89)
```

另外每帧一行汇总（与 verbose 开关无关）：

```
[GeomSummary][frame] tracks=12 static=7 refined=6 size_update=4 size_slow=1 yaw_hold=3 yaw_update=2 yaw_flip=0 raw_unknown=3 raw_moving=2
```

原则：
1. **同帧同时打印 RAW 与 REFINED**（A/B 与"偏差"统计的前提）；
2. 打印判定依据（`dL/dW/dYaw/gate/flip_run/cells`），而不只是结论；
3. 决策码是封闭枚举（附录 B），可被脚本聚合；
4. DebugViewer 可选叠加：RAW OBB（白）与 REFINED OBB（另一种颜色）+ action 文本；指标仍以日志为准。

---

## 15. 参数体系（大幅精简）

### 15.1 删除的参数（v1 遗留，全部废弃）

```
kGeomAlphaCenterBase     （center EMA）
kGeomCenterRejectM       （center reject）
kGeomMaxOffsetM          （center clamp / max correction）
prior_conf / meas_conf / final_conf / size_conf 相关全部
EMA 稳态方差 / sigma_out 类推导相关
```

### 15.2 保留的参数（全部为 initial tuning values）

| 分组 | 参数 | 初值 | 语义 |
|---|---|---|---|
| 生命周期 | `kGeomEnable` | (实施后 true) | 总开关（A/B 用） |
| | `kGeomEnableSize` / `kGeomEnableYaw` | true / true | 子开关 |
| | `kGeomMinValidFrames` | 2 | prior 何时 `established` |
| | `kGeomPriorMaxMissFrames` | 5 | 连续 miss 超限 → STALE |
| L/W | `kGeomSizeAlpha` | 0.5 | 正常更新（大 = 更信当前） |
| | `kGeomSizeSlowAlpha` | 0.2 | 超门时的慢速靠近 |
| | `kGeomSizeGateRel` | 0.5 | **相对**门（主门限） |
| | `kGeomSizeGateAbsMin` | 0.03 | 绝对下限（需与量化步长匹配） |
| | `kGeomSizeGateAbsMax` | 0.15 | 绝对上限（防大目标门过宽） |
| | `kGeomSizeConfirmFrames` | 3 | 超门后确认帧数 |
| | `kGeomSizeMin` / `kGeomSizeMax` | 0.05 / 3.0 | 合法性边界 |
| Yaw | `kGeomYawMinCells` | 4 | 可靠性 Y1 |
| | `kGeomYawMinExtent` | 0.15 | 可靠性 Y2 |
| | `kGeomYawMinAniso` | 0.3 | 可靠性 Y3（近方形否决） |
| | `kGeomYawAlpha` | 0.3 | 正常平滑权重 |
| | `kGeomYawGateDeg` | 25 | 正常变化带 |
| | `kGeomYawCandTolDeg` | 20 | 翻转候选一致性 |
| | `kGeomYawFlipConfirm` | 3 | 接受翻转所需连续帧数 |

> **所有具体阈值都只是 initial tuning values，不是最终正确值。**
> 特别是本项目低矮目标 `L ≈ 0.1~0.2m`、`W ≈ 0.05~0.1m`，必须先统计现有 STATIC Track 日志的
> $\lvert\Delta L\rvert$、$\lvert\Delta W\rvert$、$L/W$ ratio、$\lvert\Delta\text{yaw}\rvert$ 分布，再取定门限（§26.1）。

---

## 16. Geometry Refinement 插入位置

```
1050  DrawClusterOverlay(outputClusters)                      # Cluster 图层，保持原样
1058  m_tracker.update(detections, rec_timestamp_ms)           # 不变
1078  UpdateTrackMotionStates(loc_pose, have_pose, ts)         # Phase 3-A（必须启用）
      ──────────────────────────────────────────────────────
      RefineTrackGeometry(outputClusters, outTracks, loc_pose, have_pose, m_geomPriors, m_geomResults)
          · 只读 m_tracker.vtrackings
          · 生成 outTracks（RAW 副本 + refined L/W/yaw/corners）
          · 更新 m_geomPriors（UNKNOWN / STATIC 都更新）
      ──────────────────────────────────────────────────────
1101  // ApplyHistoricalGeometry(...)  保持注释（旧方向已废弃，§27）
1122  DrawMapAndAllOverlay(..., outTracks, ...) / DrawTrackOverlay(..., outTracks, ...)
1159  m_visBuffer.Publish(..., outTracks)
1168  ConvertTrackToS2ObstacleBox(outTracks, ...) → UDP
```

硬约束：
1. 必须在 `m_tracker.update()` 之后；
2. 必须在 `UpdateTrackMotionStates()` 之后（要读 `motion_state`）；
3. 必须在 Debug / visBuffer / UDP 之前；
4. **不得**插入在 `ConvertClustersToTrackedObstacles` 之前（那是改 detection）。

`pcdRunningModel`（单帧、无 tracker）路径：不接 refinement（`SuTengDriver.cpp:2026` 保持不变）。

---

## 17. 前置条件：本帧匹配到的 Cluster（仅用于 `n_cells` / 几何有效性）

| 方案 | 改动 | 评价 |
|---|---|---|
| **A（推荐）** | `track.cpp` matched 分支加 **1 行**：`vtrackings[i].cluster_id = detections[best_det_idx].cluster_id;`（与 V2 已有写法 `track.cpp:680` 完全一致） | 精确；只暴露匹配结果，不改匹配/门限/删除；提供 `n_cells`、`has_obb` |
| B | 复用 Phase 2 的 `m_historicalFeedback.association_cluster_id`（`reason == SAME_TRACK_ID`） | 需重开被注释的 Phase 2 整块；且其 SAME_TRACK_ID 是"cluster 参考中心 ≈ live->pos 且 < 0.05m"的启发式反查（`SuTengDriver.cpp:1606`）→ 建议仅作交叉校验 |
| C（零改动变体） | **完全不访问 Cluster**：yaw 可靠性只用 Y2（最小延展）+ Y3（L/W 各向异性）；L/W 只用 RAW 反解值 | 完全不需要改 `track.cpp`；代价是失去 Y1（对 2-cell 对角假象的防御变弱，Y3 通常仍能拦住小 W 情形） |

> 重要：**RAW 几何本身不需要 Cluster 访问**（§4.1）。方案 A 的 1 行只是为了拿到 `n_cells / has_obb` 这类"测量质量"信息。
> 若希望本 Phase 对 Tracker 完全零改动，可直接选方案 C，并把 Y1 标记为"可选增强"。

---

## 18. Radar frame / Map frame：本 Phase 只有 yaw 需要坐标系

### 18.1 结论

| 量 | 存放坐标系 | 理由 |
|---|---|---|
| `length` / `width` | 目标自身（标量，无坐标系） | 与坐标系无关 |
| `yaw` | **地图系**（`yaw_map_deg`） | 自车 heading 变化时雷达系 yaw 会跟着变；地图系 yaw 才是目标的稳定属性 |
| center | 不涉及 | 本 Phase 不 refinement center |

### 18.2 变换公式（从代码确认，勿自行推符号）

`vehicleToMap`（`mapfilter/coordinate_transformer.cpp:47`）：

```
fx = -veh_y ; fy = veh_x ;  A = heading_deg + gridHeadingOffsetDeg
map_x =  cosA*fx + sinA*fy + pose.x
map_y = -sinA*fx + cosA*fy + pose.y
```

等价线性映射：

$$M_{v\to m}=\begin{bmatrix}\sin A & -\cos A\\ \cos A & \sin A\end{bmatrix}=R(90^\circ-A)$$

⇒ 方向角变换：

$$\varphi_{map}=\varphi_{veh}+(90^\circ-A)
\quad\Longleftrightarrow\quad
\varphi_{veh}=\varphi_{map}+A-90^\circ$$

### 18.3 实现建议：两点法（避免符号错误）

```
# map yaw → 当前雷达系 yaw（只用到方向，与 center 无关）
d_m = (cos(yaw_map), sin(yaw_map))          # 地图系方向
P   = (map_x, map_y)                        # 地图系任一点（可用 track.map_x/map_y 或历史锚点）
v0  = mapToVehicle(P.x,      P.y)           # 注意：mapToVehicle 内部已含 heading + offset
v1  = mapToVehicle(P.x+d_m.x, P.y+d_m.y)
yaw_prior_radar = fold( atan2(v1.y-v0.y, v1.x-v0.x) )
```

反向（雷达系观测 yaw → 地图系）同理，用 `vehicleToMap` 两点法。

> **pose 的使用范围仅限此处的 yaw 投影**：复用 `ProcessPcapCloud` 已读取的 `loc_pose / have_pose`，
> **不引入任何新的 Localization 逻辑**。`pose` 无效 ⇒ 该帧输出 RAW（不做 yaw refinement）——这是简单降级，不是新的定位机制。

---

## 19. 最终 UDP / Debug 的 geometry 来源

```
Current Geometry Source
  GridCluster.obb_*            ElevationMapGroundFilter.cpp:2145 / :2377
        ↓
  detections                   SuTengDriver.cpp:1846  ConvertClustersToTrackedObstacles
        ↓
  SimpleTracker::update()      track.cpp:258   matched 覆盖 / miss 保持 / 空帧外推
        ↓
  m_tracker.vtrackings         ← Tracker State（RAW，禁止修改）
        ↓ （只读）
  RefineTrackGeometry()        Phase 3-B：STATIC 时产出 refined L/W/Yaw + corners
        ↓
  outTracks（副本）            ← Debug / UDP 的最终来源
        ├─ DrawMapAndAllOverlay(outTracks) / DrawTrackOverlay(outTracks)   ✅
        ├─ m_visBuffer.Publish(outTracks)                                  ✅
        └─ ConvertTrackToS2ObstacleBox(outTracks) → UDP                    ✅
```

结论：**输出从一开始就是 Track 几何**，所以本 Phase 只要把 refined L/W/Yaw 落到 `outTracks.depth/width/corners`，
就必然影响最终输出；不存在"算了 refined 但输出仍是 raw cluster"的风险。
注意：UDP 报文**没有 yaw 字段**（`Type.h` 的 `S2obstacleBox` 只有 `depth/width/height/pos_x/pos_y/pos_z/corners[4]`），
因此 **yaw 的稳定性只能通过 corners 体现** ⇒ 重建 corners 是本 Phase 的必需输出步骤（§20）。

---

## 20. OBB corners 重建（Phase 3-B 的明确输出步骤）

```
RAW center
  + refined length
  + refined width
  + refined yaw
        ↓
Recompute OBB Corners
        ↓
TrackGeometryResult
```

公式（与 `ComputeClusterOBB` 的 `worldCorner` 约定逐项一致）：

```
u  = (cos θ, sin θ)         # 长轴
v  = (-sin θ, cos θ)        # 短轴
hl = L/2,  hw = W/2
c0 = center - hl·u - hw·v   # 左下
c1 = center + hl·u - hw·v   # 右下
c2 = center + hl·u + hw·v   # 右上
c3 = center - hl·u + hw·v   # 左上
```

**不改变**：

- corner 顺序（左下 → 右下 → 右上 → 左上）；
- 坐标系约定（主雷达系：x 前向、y 左向）；
- UDP 协议与 `S2AvoidObject` / `newS2AviodObject` 接口（只改传入几何数值的来源，不加字段）；
- `height / pos_z`（沿用 RAW）。

`TrackGeometryResult`（概念稿，独立于 `TrackedObstacle`）：

```cpp
struct TrackGeometryResult
{
    int   track_id  = -1;
    bool  refined   = false;     // false ⇒ 使用 RAW（UNKNOWN/MOVING/miss/无 prior/pose 无效…）
    float center_x  = 0.0f;      // 恒等于 RAW center（本 Phase 不修改）
    float center_y  = 0.0f;
    float length    = 0.0f;      // refined 或 RAW
    float width     = 0.0f;
    float yaw_deg   = 0.0f;      // refined 或 RAW（雷达系）
    Point2D corners[4] = {};
    const char* size_action = "";   // 决策码（附录 B）
    const char* yaw_action  = "";
    const char* skip_reason = "";   // 未 refinement 的原因
};
```

---

## 21. MVP 实施方案（三小步，每步可独立验证）

**Step 1：先出 RAW 日志（零行为变化）**
只读 `m_tracker.vtrackings`，逐帧输出 `[GeomRaw]`（L/W/yaw/cells/state），并计算 `dL/dW/dYaw` 与 gate 结果，
**但不修改任何几何**。目的：在真实 pcap 上拿到 L/W/yaw 抖动分布，用于标定 §15 的阈值。

**Step 2：UNKNOWN 养 prior（零输出变化）**
实现 prior 侧表 + UNKNOWN 分支的 Initialize/Update，输出仍为 RAW。
目的：验证 prior 生命周期与 STATIC 出现时机（与 Phase 3-A 日志对齐）。

**Step 3：STATIC refinement + corners 重建（进入 A/B）**
打开 L/W 与 Yaw refinement，`outTracks` 供 Debug/UDP 使用；同帧打印 RAW 与 REFINED。
目的：按 §26 做 A/B 与指标评估。

开关：

```
kGeomEnable = false  → Debug / UDP = RAW
kGeomEnable = true   → STATIC: Center=RAW, L/W=REFINED, Yaw=REFINED, Corners=rebuilt
                       UNKNOWN / MOVING / miss / pose 无效 : RAW
```
建议同时保留 `kGeomEnableSize` / `kGeomEnableYaw` 子开关，便于"只开 yaw 保持"这类最小验证。

---

## 22. 风险与边界情况

| # | 风险 | 影响 | 缓解 |
|---|---|---|---|
| R1 | HOLD_YAW 时 refined 框**不再紧包**当前 cells | 下游若用该框做"必须包含"逻辑会漏 | 明确声明：refined 框仅用于输出/显示；**禁止**用于过滤/关联/漏检判定；日志记录"当前 cells 落在 refined 框内的比例"（cover ratio）以监控 |
| R2 | 先验陈旧（长时间 miss 后复活，目标已变化） | 输出被旧几何拖住 | `kGeomPriorMaxMissFrames` → STALE → 输出 RAW；复活后可选择重新 INITIALIZE（更稳） |
| R3 | `nextTrackID > 50000 → 9999` 回绕（`track.cpp:467`） | 陈旧 prior 挂到新 track | `prior.last_age <= track.age` 单调性校验 + id 集合差集清理 |
| R4 | `W` 退化为 0（对角 cell，cell-center 定义下可能） | 宽度稳定在 0 或 0/0.1 跳变 | 本 Phase **不改输出约定**（不人为加地板）；日志标记 `W_DEGENERATE`；"输出宽度加 0.1m 地板"作为可选议题登记，**需下游确认后再做** |
| R5 | pose heading 噪声 / `gridHeadingOffsetDeg` 配错 | `yaw_map ↔ yaw_radar` 投影带系统偏差，refined yaw 被缓慢带偏 | 只用两点法；`pose.valid` 硬门；日志同时打印 `yaw_radar` 与 `yaw_map`，观察二者差是否与 heading 变化一致 |
| R6 | Phase 3-A 把慢速运动误判为 STATIC（< ~1.2m/s 盲区） | L/W/yaw 被"稳定"到略滞后于真实姿态 | 量级小；一旦转 MOVING 立即输出 RAW；本 Phase 不新增补偿 |
| R7 | 门限对小目标不合适（过大 ⇒ 无效；过小 ⇒ 冻结） | 无明显改善或尺寸被锁死 | 必须用 Step 1 日志分布标定；`kGeomSizeGateAbsMax` 防大目标门过宽；`kGeomSizeConfirmFrames` 防止永久冻结 |
| R8 | Y1 依赖 Cluster 访问（方案 A 的 1 行改动） | 若不做该改动，2-cell 对角假象更难拦 | 采用方案 C 时把 Y1 标记为可选增强；Y3 仍能拦"W 极小/近方形"情形 |
| R9 | 上游 `len_pri < len_sec` 分支角点与 `obb_angle` 不一致 | 若用 `depth + obb_angle` 会差 90° | **从 corners 反解 RAW（L,W,yaw）**（§4.1），天然免疫 |
| R10 | `has_obb == false`（$n<3$，当前参数下不可达） | AABB 回退路径语义突变 | refinement 侧显式处理：RAW 几何不合法 ⇒ 输出 RAW |
| R11 | 自车运动中的静止目标（本 Phase 最有价值场景） | 采样格变化大 ⇒ RAW L/W/yaw 抖动大 | 正是要抑制的对象；但要求定位精度足够（Phase 3-A 已依赖） |
| R12 | 一个 Cluster 被多个 Track 关联（歧义） | 两个 track 抢同一几何 | 可访问 Cluster 时做唯一性检查（`cluster_id → track_id` 唯一），否则跳过并记日志 |

---

## 23. 不应该做的事情（硬性禁止清单）

**算法层**

- ❌ Center refinement（EMA / 平滑 / reject / clamp / 历史位置修正）
- ❌ 任何 confidence 参与控制（`meas_conf / prior_conf / final_conf / size_conf`）
- ❌ Historical Cell Union / footprint fusion / 用历史 cell 补当前 cluster（§27 废弃方向）
- ❌ Raw point history / 5cm image / contour fusion / morphology / 最小面积矩形搜索
- ❌ 把 cell count 作为时间状态（median/max/frequency）：只允许作为当帧 Y1 输入
- ❌ 普通线性 yaw 平均（180° 等价问题）
- ❌ Kalman / Hungarian / JPDA / MHT / 新 tracker / 第二套 Track ID
- ❌ EMA 方差类统计理论推导（只保留"α 大信当前、α 小信历史"的工程语义）

**架构层**

- ❌ 把 refined geometry 写回 `SimpleTracker` 内部状态（`m_tracker.vtrackings`）
- ❌ 让 refined center/yaw/L/W 影响下一帧预测、关联、匹配、motion state、删除
- ❌ 修改 Ground Filter 算法 / Grid resolution / ROI / 阈值
- ❌ 修改 HDMap / Localization 实现与数学 / `CoordinateTransformer`
- ❌ 修改 UDP 协议 / `newS2AviodObject` / `S2obstacleBox` 接口
- ❌ 修改 `SimpleTracker` 的匹配、门限、代价函数、删除策略
- ❌ 修改已由 Phase 3-A 验证的 `UpdateTrackMotionStates()`
- ❌ 依赖 `TrackedObstacle.Rotation`（真实 pipeline 中被 `memset(0)`）
- ❌ 在 Phase 3-B 内实现 miss 帧预测 / coasting（属 Phase 3-C）

---

## 24. 推荐的代码修改文件与函数

| 文件 | 改动 | 性质 |
|---|---|---|
| **`include/track_geometry_prior.h`**（新增） | `TrackGeometryPrior` + 常量 + 纯函数：`FoldYawDeg`、`UndirectedBlendDeg`（双角法）、`MapToRadarYaw / RadarToMapYaw`（两点法）、`DeriveRawGeometryFromCorners`、`UpdateSize`、`UpdateYaw`、`RecomputeOBBCorners` | 新增，零侵入 |
| **`include/track_geometry_result.h`**（新增，或并入上者） | `TrackGeometryResult` + 决策码枚举 | 新增，零侵入 |
| `include/SuTengDriver.h` | 新增成员 `std::vector<TrackGeometryPrior> m_geomPriors; std::vector<TrackGeometryResult> m_geomResults;`；声明 `void RefineTrackGeometry(const std::vector<GridCluster>&, std::vector<TrackedObstacle>& outTracks, const LocalizationManager::Pose&, bool, unsigned long long);` | 侵入（与既有 `m_mapTracks/m_historicalGeometry` 同构） |
| `include/SuTengDriver.cpp` | ① 确认 `:1078` `UpdateTrackMotionStates(...)` 已启用；② 在其后新建 `outTracks` 副本 + 调用 `RefineTrackGeometry(...)`；③ 把 `:1122 / :1159 / :1168` 的消费对象由 `m_tracker.vtrackings` 改为 `outTracks`；④ 日志 | 侵入（核心接线 + 输出层换源） |
| `include/track.cpp`（**可选**，§17） | matched 分支加 1 行 `vtrackings[i].cluster_id = detections[best_det_idx].cluster_id;`（对齐 V2 `:680`），用于 `n_cells / has_obb` | ⚠️ 侵入（仅暴露匹配结果） |
| `include/ElevationMapGroundFilter.h/.cpp`（**可选**） | 若要把 `orientation_confidence` 做成 diagnostic log：`GridCluster` 增 `float obb_confidence`，`ComputeClusterOBB` 中赋值（2 行，纯附加） | ⚠️ 附加式，非算法改动 |
| `include/DebugViewer.cpp`（**可选**） | 叠加绘制 REFINED OBB（另一种颜色）+ action 文本（消费 `outTracks`，无需改签名） | 可选，验证用 |
| `include/historical_geometry.h` | 头部加注"已废弃（§27），被 Static Track Geometry Stabilization 取代"；开关保持关闭 | 只改注释 |

> ⚠️ 本设计**不需要**给 `TrackedObstacle` 加字段。若将来仍需添加：`track.h` 的拷贝构造/赋值是**手写**的，必须同步（历史踩坑）。

---

## 25. A/B 实验设计

```
kGeomEnable = false  →  Debug / UDP = RAW（outTracks 与 vtrackings 相同）
kGeomEnable = true   →  STATIC : Center=RAW, L/W=REFINED, Yaw=REFINED, Corners=rebuilt
                       UNKNOWN / MOVING / miss / pose 无效 : RAW
```

要求：

1. 同一条 pcap，两次运行，日志分别落盘；
2. 每帧同时打印 `geom_enable`、`motion_state`、RAW geometry、REFINED geometry（§14）；
3. 脚本按 `track_id` 对齐 RAW / REFINED 两条序列（用 `TrackGeometryResult.track_id` 关联）；
4. 当前配置 `pcapRunningModel=1 / onlineModel=0`（`config/debug_config.yaml:17-20`）⇒ 没有 UDP 发送，
   离线验证以**日志 + DebugViewer** 为主；UDP 验收需切 `onlineModel=1`。

---

## 26. Validation Plan

> Phase 3-B 的主要优化指标**不再包含 Center 平滑度**（Center 明确不 refinement）。

### 26.1 阶段 0：阈值标定（Step 1 日志，必做）

统计（逐 STATIC track）：

- $\lvert\Delta L\rvert$、$\lvert\Delta W\rvert$ 的 p50/p90/p99，以及 $L/W$ ratio 分布；
- $\lvert\Delta\text{yaw}\rvert$ 分布、180° 等价翻转次数；
- `n_cells` 分布（3 / 4 / ≥5 各占多少）。

用途：确定 `kGeomSizeGateRel / AbsMin / AbsMax / ConfirmFrames / Alpha`、`kGeomYawGateDeg / FlipConfirm / MinCells / MinExtent / MinAniso`。
**禁止**把初值当最终值直接上线。

### 26.2 L/W 稳定性（主要指标 1）

```
L step std / p95            （帧间 ΔL）
W step std / p95
L jump rate                 #{|ΔL| > gate_L} / #frames
W jump rate
```
同时监控"偏差"以防冻结：
```
mean |L_refined − L_raw| , p95
mean |W_refined − W_raw| , p95
```

### 26.3 Yaw 稳定性（主要指标 2）

```
yaw flip rate               #{|Δyaw| > 45°} / #frames
yaw step p95
180° flip count             （fold 后仍发生的大角差次数）
yaw hold / update / accept_flip 次数（决策码计数）
unreliable 比例             （Y1/Y2/Y3 否决占比，判断门是否过严）
```

### 26.4 Corner 稳定性（最重要指标 3）

```
corner displacement         Σ_j |c_j(t) − c_j(t−1)|
corner step std / p95
RAW vs REFINED corners 逐帧对比（同 track、同帧）
```
说明：即使 yaw 的统计均值没变，HOLD_YAW 也可能改变 corners；因此 corners 必须**单独度量**，不能只看 yaw 统计。

### 26.5 回归守卫（指标 4）

```
ID switch count / new ID rate / track continuity
```
**期望结果：与 baseline 完全相同。**
因为本设计不修改 Tracker 状态（§12.3 输出副本），任何变化都说明实现越界（bug），而不是效果。
此外还需说明：

> **Phase 3-B 不允许为了几何稳定而改变 Tracker 行为。**

### 26.6 场景矩阵

| Case | 场景 | 关注点 |
|---|---|---|
| A | 自车静止 + STATIC 低矮目标 | 最干净：L/W/yaw 收敛效果、corners 抖动 |
| B | 自车直线前进 + 路侧 STATIC 目标 | 采样格变化引起的 L/W 跳变是否被抑制 |
| C | 自车转弯 + STATIC 目标 | `yaw_map ↔ yaw_radar` 投影是否正确（若 yaw 随自车旋转被"保持"到错误方向 ⇒ 投影有 bug） |
| D | 3 cells ↔ 4 cells 反复跳 | L/W jump rate 是否下降；是否因 confirm 机制被"冻结" |
| E | 近方形目标 | yaw 应 HOLD；`unreliable` 占比应显著 |
| F | 目标真实移动 | 应尽快转 MOVING 并输出 RAW（不得被 prior 拉回） |

### 26.7 回滚

`kGeomEnable=false` 一键回到今天行为；`kGeomEnableSize / kGeomEnableYaw` 支持子开关最小验证。

---

## 27. Historical Cell Fusion：废弃声明（保留作参考）

保留为 rollback / reference：

```
include/historical_geometry.h（cell 补充 + 可选 weighted OBB）
SutengDriver::ApplyHistoricalGeometry() / m_historicalGeometry
ComputeHistoricalFeedback() / UpdateMapAnchors() / m_mapTracks / HistoricalFeedbackRegion
```

但必须明确：

```
Historical Cell Supplement
Historical Cell Union
Historical Candidate Cluster Fusion
```
**不是当前 Phase 3-B 的实现方向，已废弃。**

原因：

> 历史投影 Cell 不等于目标真实 footprint。尤其对 0.1m grid、3~4 cell 的低矮目标，历史 Cell 很可能只是"投影采样位置"，
> 不能直接作为当前目标的轮廓去做 union（实测亦如此：同一 Track 当前 cells = 3/4，历史 projected cells 可能只有 1）。

处置：

- `SuTengDriver.cpp:1101` 的 `ApplyHistoricalGeometry(...)` 继续保持注释；
- `kEnableHistoricalCellSupplement` / `kEnableHistoricalFusedOBB` 保持现状（不扩展、不启用）；
- 可复用工具函数：`HistFoldYawDeg`（折角）、`HistCellCenter`（cell 中心）—— 保留复用，避免重复实现；
- 旧实现说明见 `docs/Phase3B_HistoricalGeometry_CodeAnalysis.md`。

---

## 附录 A：参数表（全部为 initial tuning values）

| 参数 | 初值 | 语义 | 是否需标定 |
|---|---|---|---|
| `kGeomEnable` | (实施后 true) | 总开关 | — |
| `kGeomEnableSize` / `kGeomEnableYaw` | true / true | 子开关 | — |
| `kGeomMinValidFrames` | 2 | prior 何时 established | ⚠️ |
| `kGeomPriorMaxMissFrames` | 5 | 连续 miss 超限 → STALE | ⚠️ |
| `kGeomSizeAlpha` | 0.5 | 正常更新（大 = 更信当前） | ⚠️ |
| `kGeomSizeSlowAlpha` | 0.2 | 超门慢速靠近 | ⚠️ |
| `kGeomSizeGateRel` | 0.5 | 相对门（主门限） | ⚠️ **必须标定** |
| `kGeomSizeGateAbsMin` | 0.03 | 绝对下限（≥ 量化步长） | ⚠️ **必须标定** |
| `kGeomSizeGateAbsMax` | 0.15 | 绝对上限 | ⚠️ |
| `kGeomSizeConfirmFrames` | 3 | 超门确认帧数 | ⚠️ |
| `kGeomSizeMin` / `kGeomSizeMax` | 0.05 / 3.0 | 合法性边界 | ⚠️ |
| `kGeomYawMinCells` | 4 | Y1 | ⚠️ |
| `kGeomYawMinExtent` | 0.15 | Y2 | ⚠️ |
| `kGeomYawMinAniso` | 0.3 | Y3（近方形否决） | ⚠️ |
| `kGeomYawAlpha` | 0.3 | 正常平滑 | ⚠️ |
| `kGeomYawGateDeg` | 25 | 正常变化带 | ⚠️ |
| `kGeomYawCandTolDeg` | 20 | 翻转候选一致性 | ⚠️ |
| `kGeomYawFlipConfirm` | 3 | 接受翻转帧数 | ⚠️ |

## 附录 B：决策码（封闭枚举）

```
skip_reason : RAW_UNKNOWN | RAW_MOVING | RAW_NO_MATCH | RAW_NO_POSE | RAW_NO_PRIOR
              | RAW_PRIOR_STALE | RAW_BAD_GEOM | RAW_AMBIGUOUS | RAW_NOT_ENABLED
size_action : SIZE_INIT | SIZE_UPDATE | SIZE_UPDATE_SLOW | SIZE_CONFIRM
              | SIZE_HOLD | SIZE_REJECT_OUTLIER | SIZE_DISABLED
yaw_action  : YAW_INIT | YAW_UPDATE | YAW_HOLD(reason=JUMP|UNRELIABLE)
              | YAW_ACCEPT_FLIP | YAW_DISABLED
```

## 附录 C：单帧完整伪代码

```
RefineTrackGeometry(clusters, outTracks, pose, pose_valid, priors, results)
{
    // 0. 输出默认 = RAW
    outTracks = m_tracker.vtrackings;              // 拷贝（RAW 快照，之后只改副本）
    results.clear();

    // 1. 索引（可选：用于 n_cells / has_obb / 唯一性检查）
    cluster_by_id = buildMap(clusters, key = GridCluster::id)

    // 2. 侧表清理
    erase priors whose id ∉ outTracks 或 prior.last_age > track.age

    for each track t in outTracks:

        raw   = DeriveRawGeometryFromCorners(t)    // yaw/L/W 从 corners 反解；失败 → RAW_BAD_GEOM
        prior = GetOrCreatePrior(t.id)

        // ---------- UNKNOWN：只养 prior，输出 RAW ----------
        if (t.motion_state == UNKNOWN)
        {
            if (valid(raw))
                UpdateOrInitializeGeometryPrior(prior, raw, pose, pose_valid);
            results.push(RawOnly(t, "RAW_UNKNOWN"));
            continue;
        }

        // ---------- MOVING：不建立、不更新、不施加 prior ----------
        if (t.motion_state == MOVING)
        {
            results.push(RawOnly(t, "RAW_MOVING"));
            continue;
        }

        // ---------- STATIC ----------
        if (t.lastSeen != 0)  { results.push(RawOnly(t, "RAW_NO_MATCH")); continue; }  // 预测属 Phase 3-C
        if (!pose_valid)      { results.push(RawOnly(t, "RAW_NO_POSE"));  continue; }
        if (!valid(raw))      { results.push(RawOnly(t, "RAW_BAD_GEOM")); continue; }
        // 可选：cluster 唯一性 / has_obb / n_cells 检查（方案 A）

        if (!prior.valid)
        {
            InitializePrior(prior, raw, pose);
            results.push(RawOnly(t, "RAW_NO_PRIOR"));
            continue;
        }

        if (!prior.established)
        {
            UpdatePriorHistory(prior, raw, pose);
            if (!prior.established) { results.push(RawOnly(t, "RAW_NO_PRIOR")); continue; }
        }

        // ---------- STATIC geometry refinement ----------
        refined_center  = raw.center;                                   // Center 不做任何修改
        refined_LW      = RefineStaticTrackSize(prior, raw);            // §9
        refined_yaw     = RefineStaticTrackYaw(prior, raw, pose);       // §10
        refined_corners = RecomputeOBBCorners(refined_center, refined_LW, refined_yaw);  // §20

        result.track_id = t.id;
        result.refined  = true;
        result.center_x = refined_center.x;
        result.center_y = refined_center.y;
        result.length   = refined_LW.L;
        result.width    = refined_LW.W;
        result.yaw_deg  = refined_yaw;
        result.corners  = refined_corners;
        results.push(result);

        ApplyToCopy(outTracks[i], result);        // 只改副本：depth/width/corners（center 不变）
        LogRawAndRefined(t, raw, prior, result);  // [GeomRaw][GeomPrior][GeomDelta][GeomAction][GeomRefined][GeomCorners]
}
```

## 附录 D：全文档自检清单（对应本次收敛要求）

| # | 检查项 | 结果 |
|---|---|---|
| 1 | 是否还有 "Center EMA" | ✅ 无（§0.3 删除清单、§6 删除理由） |
| 2 | 是否还有 Center reject / clamp | ✅ 无（§15.1 标记废弃） |
| 3 | 是否还有 `prior_conf / meas_conf / final_conf` 参与控制 | ✅ 无（§13：不参与控制） |
| 4 | UNKNOWN 初始化 prior 的逻辑矛盾 | ✅ 已修正（§7.1 三分支 + 附录 C） |
| 5 | Historical Cell Fusion 是否仍被当作当前方案 | ✅ 已声明废弃（§27） |
| 6 | 是否存在 "refined geometry 写回 Tracker" | ✅ 禁止（§12、§19、§23） |
| 7 | 是否存在 "refined 影响下一帧 Tracker" | ✅ 禁止（§12.1 硬约束；实现上用输出副本） |
| 8 | 是否仍以 center stability 为主要目标 | ✅ 否（§5、§26：主要目标为 L/W、Yaw、Corners） |
| 9 | 是否明确 STATIC 才做 L/W/Yaw refinement | ✅ 是（§0.4、§7、§19、附录 C） |
| 10 | 是否明确 UNKNOWN / MOVING 输出 RAW | ✅ 是（§0.4、§7） |
| 11 | 是否明确 refinement 后重算 corners | ✅ 是（§20、附录 C） |
| 12 | 是否明确 Debug/UDP 使用 `TrackGeometryResult` | ✅ 是（§12.3、§19、§20） |
| 13 | 是否明确 Tracker State 保持 RAW | ✅ 是（§0.6、§12） |
| 14 | 是否明确 miss 后预测属 Phase 3-C | ✅ 是（§0.2、§7.3、§26.6 Case F） |
| 15 | 是否明确 Validation 重点为 L/W、Yaw、Corners | ✅ 是（§26.2 / 26.3 / 26.4） |

---

## 附：本 Phase 的一句话可执行结论

> 确认 Phase 3-A 的 `UpdateTrackMotionStates()` 已生效；在 `SimpleTracker::update()` 与最终 UDP/Debug 输出之间插入 `RefineTrackGeometry()`。
> **UNKNOWN 只建立/更新几何先验但输出 RAW，MOVING 始终输出 RAW，只有 STATIC 启用 Geometry Refinement。**
> STATIC 下**不修改 Center**，只对 **L/W 与 Yaw** 做简单、可解释的历史稳定
> （相对门 + 慢速/保持/确认；yaw 无向折叠 + jump gate + K 帧确认），
> 并基于 **RAW Center + refined L/W + refined Yaw 重新计算 OBB 四角点**。
> **Tracker State 永远保持 RAW**，refinement 结果通过独立 `TrackGeometryResult`（以输出副本的形式）送到 Debug/UDP，
> 并同帧记录 RAW 与 REFINED 结果，用于 L/W、Yaw、Corner 稳定性与回归指标的 A/B 分析。
