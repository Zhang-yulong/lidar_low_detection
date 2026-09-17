# Phase 3-B：基于 Cell 几何方向可观测性的 STATIC OBB 稳定化（设计文档 **V3**）

> 日期：2026-09-16
> 状态：**设计 = 已实现并编译通过**（`[100%] Built target low_detection`）；运行验证待执行
> 本次实现改动清单见 `docs/Phase3B_v3_Implementation_ChangeList.md`
> 关系：本文档**取代** `docs/Phase3B_TrackGeometryRefinement_Design.md`（v2，含 L/W 历史 EMA 等已废弃机制）。
> 旧文件**保持原样未修改**（便于对照，未归档 git）；v2 与 v3 的具体差异见 §0.3。

---

## 0. 定位

### 0.1 目标

> **Phase 3-B 只解决一件事**：消除 STATIC 目标因当前帧 cell 排列变化（3 cells ↔ 4 cells ↔ 5 cells …）
> 导致的 **PCA 主方向随机跳变**，同时**保持当前帧真实检测到的空间几何**。

段落职责（互不重叠）：

```
Phase 3-A：判断 UNKNOWN / STATIC / MOVING     （地图系位置证据 + 迟滞状态机，已实现）
Phase 3-B：只对 STATIC 做 OBB geometry refinement（本文档）
Phase 3-C：检测丢失时的位置/几何预测（coasting）  ← 本阶段明确不做
```

### 0.2 一句话算法

```
当前 cells 能否可靠表达方向？
   YES → 用当前 PCA yaw（但与历史明显冲突时保守使用历史 yaw）
   NO  → 用 STATIC Track 的历史 yaw（若有效；否则 RAW）
无论 yaw 来自哪里：
   L/W    ← 用最终 yaw 在当前帧 cells 上重新投影
   center ← 当前 RAW center（不做任何修正）
   corners ← 用 (RAW center, final yaw, current L/W) 重建
```

### 0.3 v2 文档 → v3 实现的关键差异（避免两套方案混读）

| 项 | v2 文档 | **v3（当前实现）** |
|---|---|---|
| L/W | 历史先验 + 非对称 EMA + 一致性门 + 慢更新/确认 | **不使用任何历史 L/W**；只用 final yaw 对当前 cells 投影重算 |
| yaw 可靠性判据 | `n_cells≥4` + `L≥0.15m` + `(L-W)/(L+W)≥0.3` | **只用 λ1/λ2 ≥ 4**（cell 数仅为 PCA 必要条件，不是可靠性条件） |
| yaw 更新方式 | 双角法平滑（α=0.3）+ 3 帧翻转确认 | **无平滑、无翻转确认**：可观测→PCA（或冲突时 HISTORY）；不可观测→HISTORY；无历史→RAW |
| 与历史冲突的处理 | 3 帧一致才接受翻转 | 直接保守使用历史 yaw（不搜索第三个角） |
| center | RAW（一致） | RAW（一致） |
| corners | 重建（一致） | 重建（一致） |
| 输出通道 | `outTracks` 副本（一致） | `outTracks` 副本（一致） |
| 历史 yaw 存储 | 侧表 `TrackGeometryPrior` | 复用 `MapAnchoredTrack`，+2 字段 |
| 阈值数量 | 生命周期 + Size + Yaw 共 ~18 个 | **2 个主阈值** + 1 个数值保护 + 开关 |

---

## 1. 决策逻辑（唯一版本）

```
                      Motion State (Phase 3-A)
                              │
              ┌───────────────┼───────────────┐
          UNKNOWN          MOVING           STATIC
              │               │               │
             RAW             RAW              ↓
                                 当前帧匹配 Cluster 有效？(lastSeen==0 && has_obb && pose valid)
                                           │              │
                                          NO             YES
                                           │              │
                                          RAW             ↓
                                            PCA 主方向可观测？(λ1/λ2 ≥ 阈值)
                                                 │               │
                                                YES              NO
                                                 │               │
                                       与历史 yaw 连续？     有历史 yaw？
                                     ┌─────┴─────┐        ┌─────┴─────┐
                                    YES          NO      YES          NO
                                     │            │       │            │
                                 PCA yaw     HISTORICAL HISTORICAL    RAW
                                     │         yaw        yaw          │
                                     └───────────┴─────────┘           │
                                                 │                    │
                                            Final yaw                 │
                                                 │                    │
                            current cells → 投影 U/V → current L/W      │
                                                 │                    │
                                        current RAW center ───────────┤
                                                 │                    │
                                          Refined OBB            （输出 RAW）
```

### 1.1 逐条规则（与实现一一对应）

| 条件 | yaw | L/W | center | 历史 yaw 记忆 |
|---|---|---|---|---|
| `motion_state != STATIC` | RAW | RAW | RAW | MOVING → **清除**；UNKNOWN → 保留 |
| `lastSeen != 0`（miss） | RAW | RAW | RAW | 保留（预测属 Phase 3-C） |
| 找不到匹配 Cluster | RAW | RAW | RAW | 保留 |
| `cluster.has_obb == false` | RAW | RAW | RAW | 保留 |
| `pose_valid == false` | RAW | RAW | RAW | 保留（无法投影） |
| 可观测 + 无历史 | **PCA** | 当前 cells | RAW | **初始化** |
| 可观测 + 与历史连续（≤30°） | **PCA** | 当前 cells | RAW | **更新** |
| 可观测 + 与历史明显冲突（>30°） | **HISTORY** | 当前 cells | RAW | 不更新（保守） |
| 不可观测 + 有历史 | **HISTORY** | 当前 cells | RAW | 不更新 |
| 不可观测 + 无历史 | RAW | RAW | RAW | —（不凭空创造方向） |

---

## 2. 方向可观测性（本阶段的核心数学依据）

### 2.1 判据

```
IsOrientationObservable(λ1, λ2, n_cells):
    if (n_cells < 3)        return false;   // PCA 不成立（必要非充分）
    if (!(λ1 > 1e-9))       return false;   // 几何退化 / NaN
    if (λ2 <= 0)            return true;    // 严格共线 → 强方向（不是“不可靠”！）
    return (λ1 / λ2) >= 4.0;
```

关键认识：

- **PCA 几乎总能给出一个 eigenvector**，但"有输出" ≠ "方向可靠"；真正要判断的是
  **最大特征值方向是否明显区别于另一个方向**（`λ1 >> λ2`）。
- **`λ2 ≈ 0` 绝不等价于"不可靠"**：共线结构（含对角共线）方向极其明确 ⇒ 判为**可观测**。
  实现上是显式分支（`λ2 <= 0 → true`），不会写成 `if (λ2 < eps) return false`。
- **`cell_count` 不是可靠性门槛**：同样的 3 cells，直线与 L 型的可靠性完全不同；
  3/4/5 之类的 cell 数**不构成**方向可靠的充分条件，只作为"PCA 是否成立"的必要条件。
- **不因 yaw 接近 0°/45°/90° 而降低可靠性**：这些都可能就是真实目标方向；
  也**不写** `same_x → yaw=…` 之类的硬编码格点规则，方向完全由 cell 中心分布经 PCA 决定。

### 2.2 阈值取值（实测标定，非拍脑袋）

用与 `ComputeClusterOBB` **完全相同**的公式（cell 中心协方差 `cov/n`）计算，0.1m grid：

| cell 形态 | λ1 | λ2 | λ1/λ2 | 可观测(≥4.0) |
|---|---|---|---|---|
| 3 cells 竖直 / 横向 | 0.006667 | **0** | ∞ | ✅ |
| 3 cells 对角共线 | 0.013333 | **0** | ∞ | ✅ |
| 4 cells 直线 / 5 cells 直线 | 0.0125 / 0.02 | **0** | ∞ | ✅ |
| 4 cells 直线 + 1 个偏置 | 0.014062 | 0.001138 | 12.36 | ✅ |
| 4 cells 折线（staircase） | 0.006545 | 0.000955 | 6.85 | ✅ |
| 5 cells T 型 | 0.010472 | 0.001528 | 6.85 | ✅ |
| 4 cells "3+1 在端部"（薄 L） | 0.0075 | 0.00125 | 6.00 | ✅ |
| 3 cells 斜向折线（zigzag） | 0.008451 | 0.000438 | 19.28 | ✅ |
| **5 cells L 型** | 0.010 | 0.0028 | **3.57** | ❌ |
| **3 cells L 型 / stair / "2×2+1"** | 0.003333 | 0.001111 | **3.00** | ❌ |
| **4 cells T 型（竖在中间）** | 0.005 | 0.001875 | **2.67** | ❌ |
| **4 cells 2×2 方块** | 0.0025 | 0.0025 | **1.00** | ❌ |
| **5 cells 十字型** | 0.004 | 0.004 | **1.00** | ❌ |

实测分布呈**双峰**，间隙落在 **[3.57, 6.00]**，因此取

```
kObbEigenRatioThreshold = 4.0      // 落在间隙内部，两侧均有余量
```

对应各验收 Case（§27）：

| Case | 形态 | 期望 | 实测 |
|---|---|---|---|
| A | 3-cell 竖直 | 方向强可观测 | ✅ λ2 = 0 → observable |
| B | 3-cell 横向 | 方向强可观测 | ✅ λ2 = 0 → observable |
| C | 3-cell L 型 | **不**因 cells==3 就认为可靠 | ✅ ratio 3.00 → **不可观测** |
| D | 4-cell 2×2 | 允许 observable=false | ✅ ratio 1.00 → **不可观测** |
| E | 5-cell 十字 | 对称 ⇒ 区分度不足 | ✅ ratio 1.00 → **不可观测** |

### 2.3 为什么不做候选角搜索

不做 `0°~180°` 全角度搜索 / `历史 yaw ±15°` 每 1° 搜索 / 最小面积矩形 / 面积-紧致度-形状代价优化：

1. 最小面积矩形的目标函数只由**极值点**决定，对 3~5 个稀疏 cell 比 PCA（用全部点的一阶/二阶矩）更易被单点误导；
2. PCA 可靠时它是冗余的；PCA 不可靠（近方形/对称）时面积面又平坦（最优角不稳定）⇒ 两端都没有增量收益；
3. 与项目既有的 PCA OBB 形成**第三套 orientation 定义**，引入不必要的耦合与调参面。

---

## 3. yaw convention 与无向角工具

### 3.1 保持项目既有 convention

项目 OBB 的 `obb_angle` 为弧度、表示**无向长轴**（`θ ≡ θ+180°`）。
Phase 3-B **不改变**这个 convention（不切到 `[0, π)`，也不改 `obb_angle` 的存储语义），
只在内部统一归一化：

```cpp
NormalizeYaw180(yaw)  →  [-π/2, π/2)      // π/2 → -π/2；-3π/2 → -π/2；π → 0
AngularDistance180(a,b) → [0, π/2]        // 无向轴夹角
```

### 3.2 禁止 `fabs(yaw1 - yaw2)`

```
yaw1 = 1°  , yaw2 = 179° → 应为 2°   （不是 178°）
yaw1 = 88° , yaw2 = -89° → 应为 3°   （应判为方向连续）
```

`AngularDistance180()` 天然覆盖两件事：

1. 无向轴的 180° 等价；
2. **eigenvector 的 ±v 符号歧义**（相邻帧出现 `+10°` 与 `-170°` 时，应识别为同一主轴，而不是"目标转了 180°"）。

> 任何 yaw continuity 判断**都禁止**直接 `fabs(yaw1 - yaw2)`。
> 项目里 `HistFoldYawDeg()` 是**度制**折角（旧 3-B 专用，已停用）；Phase 3-B 使用弧度制唯一实现 `NormalizeYaw180()`。

---

## 4. 历史 yaw 的来源、坐标系与生命周期

### 4.1 存什么、在哪存

| 项 | 方案 |
|---|---|
| 存什么 | **只存"被接受的稳定 yaw"**（不是每帧 RAW PCA yaw） |
| 坐标系 | **地图系**（`static_yaw_map_rad`，弧度，无向长轴，已 fold 到 `[-π/2, π/2)`） |
| 存哪里 | 复用 `m_mapTracks`（`MapAnchoredTrack`）+ 2 字段：`has_static_yaw` / `static_yaw_map_rad` |
| 谁维护 | `RefineStaticObbGeometry()` 唯一写入；`UpdateMapAnchors()` **不修改**这两个字段 |

**为什么必须是地图系**：自车 heading 变化时，静止目标的雷达系 yaw 会随自车一起转；
只有地图系 yaw 才是目标的稳定属性。用雷达系存会导致"自车转弯 ⇒ 判定为方向跳变 ⇒ 永远 HOLD 旧 yaw"。

**为什么不能从 `map_corners` 反推**：那等于把"本帧 RAW 的 yaw"当作历史，历史 fallback 会立刻失去意义。

### 4.2 时序（先读后写）

```
上一帧之前积累的 historical yaw
        ↓  读取（本帧 PCA 计算之前）
当前 Cluster PCA（λ1/λ2 + pca_yaw）
        ↓
判断可观测性 → 选择 final yaw
        ↓
输出 refined geometry
        ↓
仅在“采用了可靠 PCA 方向”时写回历史 yaw（下一帧使用）
```

### 4.3 生命周期

```
首次可观测（无历史）           → 初始化历史（并输出 PCA yaw）
可观测 + 与历史连续            → 更新历史
可观测 + 与历史冲突            → 历史不变（保守）
不可观测                      → 历史不变（保留）
MOVING                        → 清除历史（姿态已是时变量，避免 MOVING→STATIC 后误用陈旧方向）
UNKNOWN                       → 保留（“单帧不可靠不失效”）
Track 被 tracker 删除          → UpdateMapAnchors 的 id 差集清理自动移除
nextTrackID 回绕(>50000→9999)  → 先验侧表按 id 差集清理 + 现有 `age/lastSeen` 语义兜底（见 §11 风险）
```

---

## 5. L/W：用 final yaw 在当前 cells 上重新投影

```
U = (cos θ, sin θ)        θ = final yaw
V = (-sin θ, cos θ)

对当前 Cluster 的每个 cell 中心 c_i：
    u_i = c_i · U
    v_i = c_i · V

length = max u_i - min u_i
width  = max v_i - min v_i
```

要点：

1. **尺寸永远来自当前帧实际存在的 cells**，**不使用 `historical_length/width`**，不做 EMA / average / clamp / min-max 历史；
2. 尺寸定义与既有 `ComputeClusterOBB` 一致（只覆盖 cell 中心，**不含 ±半格 padding**），**不擅自改变项目既有尺寸语义**；
3. 本阶段的目的是"避免 yaw 不稳定导致 L/W 坐标轴跟着旋转、出现长度/宽度互换或剧烈跳变"，
   **不是**让尺寸永远保持上一帧数值；例如上一帧 `length = 0.20`、当前 cells 实际只覆盖 `0.10` ⇒ 输出 `0.10`。

---

## 6. center：恒为当前 RAW center

```
refined center = m_tracker.vtrackings[i].pos_x / pos_y   （原样）
```

禁止：`center EMA` / 历史位置平滑 / center correction / clamp / 历史 center 替换。
理由：Phase 3-A 已负责 STATIC/MOVING 判定与地图系位置证据，Phase 3-B 不重复承担位置稳定职责。

---

## 7. corners 重建（明确输出步骤）

```
c0 = center - hl·U - hw·V    # 左下
c1 = center + hl·U - hw·V    # 右下
c2 = center + hl·U + hw·V    # 右上
c3 = center - hl·U + hw·V    # 左上        (hl = L/2, hw = W/2)
```

- 顺序与 `ComputeClusterOBB` / `TrackedObstacle` / `S2obstacleBox` **完全一致**（左下→右下→右上→左上，`c0→c1` 沿长轴）；
- **不使用历史 corners**，也不做"历史 corners + 当前 cells"的混合；
- 坐标系、UDP 协议、`newS2AviodObject` 接口、`height/pos_z` 全部不变。

---

## 8. 修改前的代码检查结论（10 问）

| # | 问题 | 结论 |
|---|---|---|
| 1 | `ClusterObstacleGrid()` → OBB 的 cell 传递 | BFS 收集索引 → `BuildClusterFromCells()` 存入 `GridCluster::cell_indices` → `ComputeClusterOBB()` 使用；**转 `TrackedObstacle` 时 cells 被丢弃** ⇒ 必须能回到本帧 `GridCluster` |
| 2 | `ComputeClusterOBB()` 是否已算 eigenvalues | **已算但未保存**（`lambda_min/lambda_max/orientation_confidence` 均为局部变量）⇒ 只做保存 |
| 3 | eigenvector / yaw 的范围与 convention | `obb_angle = atan2(axis_primary.y, axis_primary.x)` ∈ (-π, π]，未归一化；角点始终沿主轴构造 ⇒ 无向长轴语义；v3 不改其存储语义 |
| 4 | historical geometry 是否已存 yaw | **没有**；`m_mapTracks` 当前也未被维护 ⇒ 最小扩展 2 字段（且不能从 corners 反推） |
| 5 | `UpdateTrackMotionStates()` 输出 | `motion_state` + map 位置/速度/证据 + 计数器；Phase 3-B **只消费 `motion_state`**，不重算状态 |
| 6 | `ApplyHistoricalGeometry()` 调用位置 | 位于 `SuTengDriver.cpp:1101` 且**被注释**（其依赖的 Phase 2 块 `:1095` 亦注释）⇒ 不在 pipeline，v3 不启用 |
| 7 | `UpdateMapAnchors()` 与 3-B 的正确时序 | 当前未调用；v3 接线为 `tracker.update → 3-A → UpdateMapAnchors(RAW) → 3-B → Debug/UDP`，无反馈回路 |
| 8 | RAW / refined 当前如何区分 | **无法区分**（只有一份几何且被 Debug/visBuffer/UDP 直接消费）⇒ v3 引入 `outTracks` 副本 |
| 9 | 最小需改哪些文件 | 6 个（新 header、`ElevationMapGroundFilter.{h,cpp}`、`historical_feedback.h`、`track.cpp`、`SuTengDriver.{h,cpp}`） |
| 10 | 可复用的工具 | `HistCellCenter()`（cell 中心）、`CoordinateTransformer::mapToVehicle/vehicleToMap()`（yaw 两点法投影，自动含 `heading + gridHeadingOffsetDeg`）；新增唯一的弧度制 `NormalizeYaw180()` |

---

## 9. 实现映射（改动清单）

详见 `docs/Phase3B_v3_Implementation_ChangeList.md`（7 点）。摘要：

| # | 文件 | 改动性质 |
|---|---|---|
| 1 | `include/static_obb_refinement.h`（新增） | 策略内核（常量 + 纯函数 + 决策） |
| 2 | `include/ElevationMapGroundFilter.h` | `GridCluster` +3 字段（保存 λ1/λ2/conf） |
| 3 | `include/ElevationMapGroundFilter.cpp` | `ComputeClusterOBB()` +3 行赋值（不改算法） |
| 4 | `include/historical_feedback.h` | `MapAnchoredTrack` +2 字段（历史 yaw 记忆） |
| 5 | `include/track.cpp` | V1 matched 分支 +1 行（暴露本帧匹配 cluster id，对齐 V2） |
| 6 | `include/SuTengDriver.h` | 声明 `RefineStaticObbGeometry()` |
| 7 | `include/SuTengDriver.cpp` | include + 启用 `UpdateMapAnchors` + 输出副本 + 实现 + 3 处消费换源 |

---

## 10. 时序与 RAW/Refined 隔离

### 10.1 时序

```
tracker.update()
   → UpdateTrackMotionStates()        Phase 3-A
   → UpdateMapAnchors()               RAW 几何 → map 锚点（必须在 refinement 之前）
   → RefineStaticObbGeometry()        只读 vtrackings → 写 outTracks
   → Debug / visBuffer / UDP          读 outTracks
```

**禁止**形成：`Refined OBB → Map Anchor → Historical Association → Refined OBB`。

### 10.2 隔离边界

```
RAW      : center / L / W / corners / vx,vy / age / lastSeen / motion_state   （m_tracker.vtrackings，只由 tracker 修改）
REFINED  : depth / width / corners（center 恒等 RAW）                        （outTracks 副本）
```

Phase 3-B **不能影响**：`SimpleTracker` association / 速度 / age / lastSeen / cluster matching / motion state / track deletion；
也不能让 refined 结果进入下一帧的 tracker prediction 或 association。

---

## 11. 日志与验证

### 11.1 逐 STATIC Track 日志

```text
[GeomRefine] Track=10010 state=STATIC cells=3 lambdaMax=0.006667 lambdaMin=0.000000 eigenRatio=inf pcaYaw=-70.3 histYaw=11.9 yawDelta180=82.2 observable=1 yawSource=HISTORY finalYaw=11.9 rawL=0.20 rawW=0.08 refinedL=0.17 refinedW=0.09 axisMismatch=1 center=(3.41,0.88)
```

字段：`cells / lambdaMax / lambdaMin / eigenRatio / pcaYaw / histYaw / yawDelta180 / observable / yawSource / finalYaw / rawL / rawW / refinedL / refinedW / axisMismatch / center`。
跳过时打印 `skip=` 决策码（`NO_MATCH / NO_CLUSTER / NO_OBB / NO_POSE / NO_USEFUL_YAW`）。
每帧另有 `[GeomRefineSummary]` 汇总（含 `notStatic / raw / axisMismatch` 等计数）。

### 11.2 决策码

```
yawSource : PCA | HISTORY | RAW
skip      : NO_MATCH | NO_CLUSTER | NO_OBB | NO_POSE | NO_USEFUL_YAW
size      : 无独立决策（L/W 恒由当前 cells + final yaw 决定）
```

### 11.3 验证要观察的 6 个问题

| # | 问题 | 看哪个字段 |
|---|---|---|
| 1 | `3→4→3→5 cells` 变化时 PCA yaw 是否跳变 | `pcaYaw` 逐帧序列 |
| 2 | 跳变时能否识别出方向不可靠 | `observable` / `eigenRatio` |
| 3 | `observable=0` 时是否正确使用历史 yaw | `yawSource=HISTORY` + `finalYaw=histYaw` |
| 4 | 即使 `finalYaw` 来自历史，L/W 是否仍来自当前 cells | `refinedL/refinedW` 随 cells 变化而 `finalYaw` 不变 |
| 5 | center 是否恒为当前 RAW center | `center=` 与同帧 tracker 日志 `Track id: … Center(...)` 对齐 |
| 6 | 是否意外改变 association / motion state / map anchor | `[GeomRefineSummary] notStatic/noMatch` 计数 + ID 连续性与 `kObbRefineEnable=false` 逐字节一致 |

### 11.4 A/B

```
kObbRefineEnable = false → Debug/UDP = RAW（outTracks == vtrackings 的内容）
kObbRefineEnable = true  → STATIC: center=RAW, L/W=refined, yaw=PCA|HISTORY, corners 重建
                           UNKNOWN / MOVING / miss / pose 无效 / 无可用 yaw : RAW
```

当前配置 `pcapRunningModel=1 / onlineModel=0` ⇒ 无 UDP 发送，离线验证以**日志 + DebugViewer** 为主。

### 11.5 建议的量化指标

```
① yaw 侧：pcaYaw 帧间跳变率(>45°)、yawDelta180 分布、yawSource 占比、observable 占比
② 尺寸侧：refinedL/refinedW 帧间步长 std/p95（对比 RAW 的 rawL/rawW）
③ corners 侧（最重要）：Σ|c_j(t)-c_j(t−1)| 的 std/p95，RAW vs REFINED 对比
④ 回归：ID 连续性 / 新增 ID 率（期望与 baseline 完全相同）
```

---

## 12. 阈值表（全部为 initial tuning value）

| 参数 | 初值 | 物理含义 | 是否需要标定 |
|---|---|---|---|
| `kObbRefineEnable` | `true` | A/B 总开关 | — |
| `kObbRefineVerbose` | `true` | 逐 Track 日志 | — |
| `kObbEigenRatioThreshold` | `4.0` | PCA 主方向可观测性（λ1/λ2） | 已有实测分布（间隙 3.57~6.00）；如需放宽可降到 ~3.0，但 3-cell L 型(3.00)会落到边界 |
| `kObbYawJumpMaxDeg` | `30.0` | 与历史 yaw 的最大允许无向轴夹角 | ⚠️ **需用 `yawDelta180` 分布标定** |
| `kObbLambdaDegenerate` | `1e-9` | 退化保护（非调参项） | — |
| `kObbMinCellsForPCA` | `3` | 与上游 `n<3 ⇒ has_obb=false` 一致 | — |

---

## 13. 明确不做的事

```
历史 cell union / 历史轮廓融合 / 用历史 cell 补当前 cluster        （旧方向，已废弃）
历史 L/W 平滑（EMA / average / clamp / min-max history）
center 平滑 / center 历史修正 / center clamp
Kalman / EMA(yaw) / Hungarian / JPDA / MHT / 第二套 tracker / 第二套 Track ID
候选 yaw 搜索（0°~180°、历史 yaw ±15° 每 1°）/ 最小面积矩形 / 面积-紧致度-形状代价优化
raw point cloud history / 5cm image / contour fusion / morphology
修改 Ground Filter / Grid resolution / ROI / BFS / Region Growing
修改 HDMap / Localization / CoordinateTransformer 数学
修改 UDP 协议 / newS2AviodObject / S2obstacleBox
修改 SimpleTracker 的匹配、门限、代价函数、删除策略
修改 Phase 3-A 的 UpdateTrackMotionStates()
把 refined geometry 写回 Tracker 或让其影响下一帧 tracker
在 Phase 3-B 内做 miss 帧预测（属 Phase 3-C）
```

---

## 14. 风险与边界情况

| # | 风险 | 说明 / 缓解 |
|---|---|---|
| R1 | `axisMismatch`：采用 HISTORY yaw 且与当前 cells 长轴不一致 ⇒ `refinedL < refinedW` | 按公式如实输出（**不做交换**，交换等于变相改变 yaw），只记日志与计数；若下游假定 `depth ≥ width` 需另行决策 |
| R2 | pose 无效 | 该帧不做 yaw refinement（输出 RAW），历史 yaw 也不更新（不伪造） |
| R3 | 历史 yaw 陈旧 | `MOVING` 时清除记忆；`UNKNOWN` 保留；Track 删除时随锚点清理 |
| R4 | `nextTrackID > 50000 → 9999` 回绕 | 侧表按 id 差集清理；如出现异常可加 `age` 单调性校验 |
| R5 | `W` 退化为 0（对角 cell 在 cell-center 定义下可能出现） | 本阶段**不改输出约定**（不人为加地板）；如需加地板需下游确认 |
| R6 | 阈值对小目标不合适 | 可观测性阈值已有实测支撑；`kObbYawJumpMaxDeg` 待标定（过大 ⇒ 不保守；过小 ⇒ 过度 HOLD） |
| R7 | 真实目标姿态变化（STATIC 目标被撞歪/被搬动） | 按设计**保守**：一直与历史冲突 ⇒ 一直使用历史 yaw（并由 `axisMismatch` 暴露异常），不做自动翻转接受机制 |
| R8 | 自车运动中的静止目标（最有价值场景） | 地图系历史 yaw + 两点法投影可正确处理自车旋转；依赖定位精度（Phase 3-A 已依赖） |
| R9 | HOLD 历史 yaw 时 refined 框可能不紧包当前 cells | refined 框仅用于输出/显示；**禁止**用于过滤/关联/漏检判定 |
| R10 | 上游 `len_pri < len_sec` 分支（角点仍按主轴构造而 `obb_angle` 取次轴） | 直接用 `obb_angle` 会差 90°；如后续需要更强健壮性，可改为从 corners 反解 (L,W,yaw)（本次未做，保持最小改动） |

---

## 15. 后续可选项（本次未实现，按需再议）

1. `axisMismatch` 持续发生时的降级策略（例如连续 K 帧冲突后接受 PCA 方向）；
2. yaw 的轻度平滑（双角向量法）——目前完全不平滑；
3. 从 corners 反解 RAW (L,W,yaw) 以彻底规避 R10；
4. `W` 的地板（若下游对退化宽度敏感）；
5. miss 帧（Phase 3-C）与 STATIC 历史几何的结合。
