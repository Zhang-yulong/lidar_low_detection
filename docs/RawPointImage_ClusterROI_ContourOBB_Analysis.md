# Raw Point Image + Cluster ROI + Contour OBB —— 第一阶段分析文档（Phase 3-B'）

- 对应 Prompt：`docs/Phase 3-B'：Raw Point Image + Grid Cluster ROI + Contour OBB.md.md`
- 参考早期架构：`docs/MapFrameHistoricalFeedbackRawPointImage_FinalArchitecture.md`
- 状态：**仅分析，未修改任何算法代码**（除本文件外工作区无改动）
- 行号基准：当前工作区代码实测（`include/ElevationMapGroundFilter.{h,cpp}`、`include/SuTengDriver.cpp`、`include/DebugViewer.{h,cpp}`、`include/track.{h,cpp}`、`config/debug_config.yaml`）
- 结论口径：`[已验证]` = 已由代码/配置确认；`[待实测]` = 需要日志或真机数据确认

---

## 0. 结论摘要（先看这里）

| # | 结论 | 性质 |
|---|------|------|
| C1 | Raw Point Image **不需要新的坐标约定**：直接定义为 `m_grid_rows × m_grid_cols`，像素 `(row, col)` 与 `GridIndex(row, col)` 同索引，与 Grid 严格 1:1，无翻转/无 origin offset/无 row-col 交换 | `[已验证]` |
| C2 | **图像内容必须做“非地面”筛选**。若把 `BuildGrid` 阶段的所有原始点都栅格化，ROI 会被地面点填满，轮廓退化成 ROI 矩形，方案自毁 | `[已验证]`（推理 + 配置） |
| C3 | 最佳插入点是 **`ReclassifyPointCloud()`（`ElevationMapGroundFilter.cpp:1843`，其点云循环在 `:1855`）**：该循环已存在、已做 `WorldToGrid`、且已完成“地面/障碍物”判定 → **零新增点云遍历** | `[已验证]` |
| C4 | 稀疏点云下 **`findContours` 会产生“一个孤立像素 = 一个轮廓”**，不能简单“取最大轮廓”。需要“轮廓空间关联 + 像素并集”策略 | `[已验证]`（OpenCV 定义） |
| C5 | `findContours` / `minAreaRect` 均属于 `opencv_imgproc`，本机 `libopencv_imgproc.so 3.3` 存在，`${OpenCV_LIBS}` 已在 `CMakeLists.txt:120` 链接 → **无需改构建**（但建议把 OpenCV 依赖隔离在新文件里，保持 GroundFilter 不依赖 OpenCV 的现状） | `[已验证]` |
| C6 | A/B 基线：**`ComputeClusterOBB()`（`:2377`）一行不改**，新增 `img_obb_*` 字段与之并列；输出层先只改 `outTracks`（Debug + UDP），不动 tracker 输入 | `[已验证]` |
| C7 | 第一版建议 `kObbRefineEnable = false`（`static_obb_refinement.h:50`），即屏蔽 `RefineStaticObbGeometry()`；历史 yaw 记忆随之停用（`UpdateMapAnchors` 本体必须保留） | `[已验证]` |
| C8 | 预期语义变化：IMG OBB 是**“可见点云表面”的最小外接旋转矩形**，`W` 会系统性小于 PCA OBB 的 `W`（PCA 基于 cell 中心，`W ≥ 1 格 = 0.1m`）。必须记录、暂不补偿 | `[待实测]` |
| C9 | 期望收益的机理：cell 中心被量化到 0.1m 网格 → 排列变化引起 ±30° 级 yaw 跳变；真实点位置帧间只有 ±0.03m 级抖动 → `minAreaRect` 方向更稳定 | `[待实测]`（本阶段要验证的核心命题） |

### 0.1 需要用户决策的 3 个点（本阶段实施前必须定）

1. **图像内容**：只栅格化“判为障碍物”的点（推荐，见 §4）／还是全部原始点（严格照 Prompt 字面，但方案失效）。
2. **多轮廓策略**：`union`（关联轮廓像素并集，推荐，抗稀疏）／`largest`（严格照 Prompt §13“确定哪个 contour 对应当前 Cluster”）。
3. **输出生效范围**：只替换 `outTracks`（Debug/UDP，推荐第一版）／还是同时替换 `detections`（会进入 tracker，影响关联与速度，风险大）。

---

## 1. 当前代码数据流（含行号）

```text
SutengDriver::ProcessPcapCloud()
  └─ PointCloudTransform(pInputCloud, R, pFilteredPointCloud)        SuTengDriver.cpp:995
  └─ ProcessWithObstacleDetection(pFilteredPointCloud, ...)          SuTengDriver.cpp:1015
        ├─ BuildGrid(*cloud)                                         ElevationMapGroundFilter.cpp:145
        │     └─ for (point : cloud.points)  ← 全量遍历 #1（:233 ~ :279）
        ├─ ComputeGroundHeight()  :368
        ├─ ComputeSlope()         :420
        ├─ RegionGrowing()        :557
        ├─ GenerateGroundMask()   :782
        ├─ ComputeGroundReference()  :1090
        ├─ AnalyzeVerticalOccupancy(*cloud, ...)  :1418
        │     └─ for (point : cloud.points)  ← 全量遍历 #2（:1462 ~ :1483，layer_histogram）
        │     └─ ReclassifyPointCloud(cloud, ...)  :1800 → 定义 :1843
        │           └─ for (point : cloud.points)  ← 全量遍历 #3（:1855 ~ :1938）
        └─ ClusterObstacleGrid()  :1977
              └─ BuildClusterFromCells()  :2145
                    └─ ComputeClusterOBB()  :2377   ← 当前 OBB 唯一来源（PCA on cell 中心）
  └─ HDMapFilter::filterClusters(...)                                SuTengDriver.cpp:1046
  └─ m_debugViewer->DrawClusterOverlay(...)                          SuTengDriver.cpp:1052
  └─ ConvertClustersToTrackedObstacles(outputClusters, detections)   SuTengDriver.cpp:1059 / 定义 :2213
  └─ m_tracker.update(detections, ts)                                SuTengDriver.cpp:1060
  └─ UpdateTrackMotionStates(...)                                    SuTengDriver.cpp:1080
  └─ UpdateMapAnchors(...)                                           SuTengDriver.cpp:1088
  └─ RefineStaticObbGeometry(outputClusters, ..., outTracks)         SuTengDriver.cpp:1095 / 定义 :1606
  └─ m_debugViewer->DrawMapAndAllOverlay(..., outTracks, ...)        SuTengDriver.cpp:1139
  └─ m_visBuffer.Publish(..., outTracks)                             SuTengDriver.cpp:1176
  └─ ConvertTrackToS2ObstacleBox(outTracks, ...)（仅 onlineModel）    SuTengDriver.cpp:1186
```

要点：

- 生产路径（`ProcessWithObstacleDetection`）中**已经存在 3 次全量点云遍历**（`#1/#2/#3`）。`SplitPointCloud`（`:890`，内含第 4 次遍历 `:913`）在生产路径中**已被注释**（`:988`），不在链路内。
- 当前 OBB 的**唯一下游来源**是三处，全部消费 Track/Cluster 的 OBB 字段：
  - `ConvertClustersToS2ObstacleBox()`（`ElevationMapGroundFilter.cpp:2302` 附近）→ 仅在 online 模式；
  - `DrawClusterOverlay()`（`DebugViewer.cpp:1265`）→ 画 `cluster.obb_corners`；
  - `ConvertClustersToTrackedObstacles()`（`SuTengDriver.cpp:2213`）→ 写进 Track，被 `DrawMapAndAllOverlay` / `m_visBuffer` / UDP 消费。
- **track ↔ cluster 的桥已存在**：`SimpleTracker::update()` V1 matched 分支已写回 `cluster_id`（`track.cpp:423`），因此输出层可以把“本帧 cluster 的 IMG OBB”映射回对应 Track（`RefineStaticObbGeometry` 已按 `t.cluster_id` 查表，`SuTengDriver.cpp:1690` 附近）。

---

## 2. 当前 BuildGrid 如何遍历 Raw Point

`BuildGrid()`（`:145`）关键事实：

| 事实 | 位置 | 说明 |
|------|------|------|
| 单次遍历 `cloud.points` | `:233` | 每点：`isfinite` 检查 → `WorldToGrid` → `GridIndex` → 更新 `min_z/max_z/mean_z/point_num` |
| Grid 尺寸 | `:180~:187` | `cols = ceil((roi_x_max − eff_x_min)/res)`（x→col，前向）；`rows = ceil((roi_y_max − roi_y_min)/res)`（y→row，左向） |
| `eff_x_min` | `:170` | `m_effective_roi_x_min = max(roi_x_min, car_half_x + body_filter_x_threshold)` |
| 点云变换结果 | `SuTengDriver.cpp:995` | 传入 BuildGrid 的 `cloud` 就是 `PointCloudTransform` 之后的 **车体/主雷达系**点云（Raw Point 的正确定义域） |
| 已有调试残留 | `:250~:265` | 循环内有一段硬编码中心 `(4.363379, 0.691507)` 的测试点计数，与本方案无关 |

**结论**：BuildGrid 是“世界坐标 → Grid 索引”的唯一权威实现，其量化规则 `WorldToGrid()`（`:281`）必须原样复用，绝对不要另写一套。

---

## 3. Raw Point Image 的插入位置选择

| 候选 | 位置 | 语义正确性 | 额外点云遍历 | 结论 |
|------|------|-----------|-------------|------|
| A. BuildGrid 循环内 | `:233` 循环体 | ✗ 此时 `ground_reference_z` 尚未计算（`:1090` 才产生），无法区分地面/障碍物 → 见 §4 | 0 | 仅可作为“全部点占据”诊断图层（debug 开关） |
| B. AnalyzeVerticalOccupancy Phase 2 | `:1462` 循环体 | △ 有 cell + z + ground_reference_z，但此处还没有最终分类（分类在 `ReclassifyPointCloud`） | 0 | 备选；会让“地面判定口径”出现两套，不建议 |
| **C. ReclassifyPointCloud 循环内（推荐）** | `:1855` 循环体 | ✓ 与 `obstacle_cloud` 完全同源：**凡进入 `obstacle_cloud` 的点，写一个像素** | 0 | **推荐** |
| D. 新起一次遍历 | 任意 | ✓ | +1 次 20w 点 | 违反 Prompt §7/§18 的性能要求 |

采用 C 后，数据流变成（注意：`BuildGrid` 里同时 `resize` 图像，保证行列一致）：

```text
ReclassifyPointCloud(cloud)          // 既有第 3 次遍历，零新增
    for (point : cloud.points)
        ...原有分类逻辑...
        if (点被判为 obstacle) {
            obstacle_cloud.push_back(point);
            m_rawPointImage[row * m_grid_cols + col] = 1;   // ← 本阶段唯一新增的写入
        }
```

注意细节：

- 写像素必须在 `ReclassifyPointCloud` 的**早退分支之前**判断（`WorldToGrid` 失败 / `!cell.valid` 两个分支也 `push_back` 到 `obstacle_cloud`）。这两个分支的点**没有有效 `(row,col)` 或落在 ROI 外**，因此只能“不写像素”，需在日志里统计其数量（影响 ROI 边缘 cluster 的几何，见 §13 风险 R4）。
- `debug_config.yaml` 中 `min_points_per_cell: 1`，即“非空 cell 全部 valid”，`!cell.valid` 分支实际极少触发（可忽略但不删）。
- 图像内存：`rows × cols = 60 × 71 = 4260` 字节（`CV_8UC1` 或 `std::vector<uint8_t>` 均可），可忽略。

---

## 4. 图像内容语义：为什么不能栅格化“全部原始点” `[关键]`

Prompt §4 把 Raw Point Image 定义为“原始点云 XY 投影”。但如果**不加筛选**地投影：

```text
Cluster = 3 个 is_obstacle_candidate cell（例如路沿石，顶面高出地面 10~40cm）
每个 cell（0.1m×0.1m）内除了障碍物点，还必然含大量地面点
（cell 的 top_height ∈ [0.10, 0.40] 意味着 max_z 高于地面，但地面点仍在同一 cell 内）
→ 栅格化后 ROI 几乎被地面点像素填满
→ findContours 得到的就是“ROI 边界矩形”
→ minAreaRect ≈ ROI 矩形（yaw ≈ 0 或 90° 的轴向框）
→ 等价于回到 AABB，比现在的 PCA OBB 更差
```

因此 **Raw Point Image 的内容必须是“Raw Point Cloud 中属于障碍物的那部分真实点”**，而“障碍物”的判定口径必须与下游一致 → 直接用 `ReclassifyPointCloud` 的判定（`point.z > cell.ground_reference_z + layer_resolution`）。

自洽性检查（`[已验证]`）：cluster 的 cell 满足 `top_height = max_z − ground_reference_z ≥ 0.10m > margin(=layer_resolution=0.05m)`，因此**每个 cluster cell 至少贡献 1 个障碍物像素**，图像不会整块为空。

> 备选（若用户希望严格照 Prompt 字面）：保留 `m_rawPointImageUseAllPoints` 开关，在 BuildGrid 循环内额外填一张“全点占据图”仅用于可视化对照，**不参与 `findContours`**。

---

## 5. 分辨率定义：0.10m / pixel（与 Grid 完全一致）

- `grid_resolution = 0.1`（`config/debug_config.yaml:102`）。
- 图像 **1 pixel = 1 grid cell**，即 `0.1m × 0.1m`。
- 明确不引入 `kGridPixelScale = 10`（`DebugViewer.h:297`）——那是**可视化放大倍数**，不是算法分辨率。
- ROI：`roi_x ∈ [eff_x_min=0.9, 8.0)`（`cols=71`），`roi_y ∈ [−3.0, 3.0)`（`rows=60`）。

---

## 6. Grid ↔ Image 坐标映射（严格 1:1，无翻转）

```cpp
// 世界 → 图像（与 BuildGrid 完全同一套量化，直接复用，不新写公式）
int row, col;
if (!WorldToGrid(x, y, row, col)) return;      // 同时完成 ROI 与边界判定（半开区间 [min, max)）
image[row * m_grid_cols + col] = 1;            // 与 m_vGridCell 同索引

// 图像 → 世界（像素索引 ↔ cell 中心；与 GridIndexToWorld 等价）
//   u = col, v = row（连续像素坐标）
x = m_effective_roi_x_min + (u + 0.5f) * grid_resolution;
y = roi_y_min            + (v + 0.5f) * grid_resolution;
```

逐项回答 Prompt §6 的四个陷阱：

| 陷阱 | 本设计结论 |
|------|-----------|
| y 翻转 | **无**。图像 row 与 Grid row 同为 `+y（车左）` 方向；翻转只发生在 DebugViewer 显示时（见 §14） |
| origin offset | **无**。两者共用 `m_effective_roi_x_min` / `roi_y_min`，车身盲区/外扩已在 `eff_x_min` 内统一处理 |
| ROI offset | **无**。图像尺寸就是 Grid 尺寸，不存在“裁掉盲区后再偏移”的第二套约定 |
| row/col 交换 | **无**。两者都是 `x→col`、`y→row`（`GridIndex = row*cols + col`，`ElevationMapGroundFilter.h:621`） |

**量化误差（必须在文档里承认的精度上限）**：任何点被量化到 cell 中心，单轴误差 ≤ 0.05m；由像素跨度得到的 `L/W` 只能是 0.1m 的整数倍（与 PCA OBB 基于 cell 中心跨度的量化粒度一致，因此 A/B 对比是公平的）。

> `DebugViewer::WorldToPixel()`（`DebugViewer.cpp:379`）用的是**另一套**变换（放大 10 倍 + 盲区偏移 + 上边距 + 翻转）。它只用于显示，**绝不能**作为算法映射（见 §14）。

---

## 7. Cluster Grid bbox → Image ROI

`GridCluster`（`ElevationMapGroundFilter.h:258`）**目前没有存 cell 的 row/col 范围**（只有世界系 AABB）。ROI 需要现场推导，代价 `O(n_cells)`：

```cpp
int min_row = INT_MAX, max_row = INT_MIN, min_col = INT_MAX, max_col = INT_MIN;
for (int idx : cluster.cell_indices) {
    int r = idx / grid_cols, c = idx % grid_cols;
    min_row = min(min_row, r); max_row = max(max_row, r);
    min_col = min(min_col, c); max_col = max(max_col, c);
}
```

### padding：建议 `1 pixel`（不是 0）

理由（对应 Prompt §10 的 4 条）：

1. **量化边界**：cell 是半开区间，真实点可能落在相邻 cell；该邻 cell 若是 `is_obstacle_candidate` 会在同一 cluster 内，但若相邻 cell 因 `top_height` 差一点没通过判定，其点仍属于同一物理物体 → `padding=1` 恰好捞回来。
2. **点云越界**：`ReclassifyPointCloud` 把 ROI 外/无 cell 的点也当障碍物，但这些点根本没有像素（§3 细节），`padding` 无法救，只能靠日志统计。
3. **OBB 贴边**：E 型/L 型 cluster 的极值点常在 bbox 四角，`padding=0` 时 `minAreaRect` 的边会压在 ROI 边界上，`findContours` 的 1px 连通性判定会让边界轮廓形态更脆。
4. **邻近障碍物污染**：`padding` 越大，邻物点进入 ROI 的概率越高 → 与 §9 的关联规则配合即可（关联采用**未 padding 的 cluster 矩形**做判定，padding 只用于“收集候选像素”）。

第一版固定 `padding = 1`，并 **clamp 到图像边界**；日志同时输出 `padding=0` 的像素数用于离线对照（不做动态 padding）。

```cpp
int r0 = max(0, min_row - padding), r1 = min(rows - 1, max_row + padding);
int c0 = max(0, min_col - padding), c1 = min(cols - 1, max_col + padding);
```

---

## 8. findContours 的真实输入与“稀疏点”问题 `[关键]`

输入（Prompt §11 要求）：**ROI 内 Raw Point Image 的二值像素**（`CV_8UC1`，0/255），由 `m_rawPointImage` 的 ROI 子矩阵拷贝得到（OpenCV 3.3 的 `findContours` 会修改输入图，**必须传副本**）。

关键发现 `[已验证]`：`findContours` 的“连通”是 8 邻域像素连通。低矮障碍物点稀疏时：

```text
ROI(5×4) 实际像素:
  . . X .
  X . . .
  . . . X
→ 3 个孤立像素 = 3 个 contour，每个 contour 1 个点
```

即 **contour 数 ≈ 像素数**，“取最大轮廓”会退化成“取 1 个点”，`minAreaRect` → 面积为 0。

因此在 Prompt §11~§13 的“findContours”与“minAreaRect”之间，必须插入一步**轮廓空间关联**（不引入 morphology，不引入阈值，不扫描点云）：

```text
Raw Point Image ROI (二值)
      │  findContours(RETR_LIST, CHAIN_APPROX_SIMPLE)
      ▼
contours[0..K]  (每个 contour = 一组像素坐标)
      │  关联规则：contour.boundingRect 与 cluster 未 padding 矩形相交（或用轮廓像素落入该矩形）
      ▼
associated contours  → 像素并集 P（策略 union）/ 最大面积者（策略 largest）
      │
      ▼  minAreaRect(P)
```

- `union` 与 `largest` 都写成可切换的枚举（默认 `union`，见 §0.1 决策 2），并**双路输出诊断**：`K`（总轮廓数）、`K_used`（关联成功数）、`pixel_count`、`used_pixel_count`。
- 若实测发现“绝大多数 cluster 只有 1 个有效轮廓”，后续可简化为 `largest`，届时以数据为准。

---

## 9. 多 contour 的目标确定规则

Prompt §13 给了方向（用 Cluster Grid bbox 与 contour bbox 做空间关联），具体化为：

1. 关联判定用 **未 padding 的 cluster cell 矩形** `[min_row..max_row] × [min_col..max_col]`；
2. 判定方式：`contour` 的**像素**中有任意一个落在该矩形内 → 关联成功（等价于 bbox 相交，但更严格，避免对角刚接触的邻物被吸收）；
3. 关联失败 → 该 contour 判为 stray point，丢弃（记录数量）；
4. 关联成功 ≥1 → 按 §8 策略生成点集；关联成功 = 0（例如 cluster cell 的像素全被“ROI 外/无 cell”那类点占据，概率极低）→ **fallback**（§11）。

明确不做（Prompt §12 要求）：`threshold` / `morphologyEx(open/close)` / `dilate` / `erode` / `blur` / 轮廓平滑。
明确不做（Prompt §3、§9 要求）：不建立 `Cluster → Raw Point` 映射、不重新扫描点云、不二次 rasterize、不把 cell 画成白块。

---

## 10. `minAreaRect()` 输出 → 项目 OBB 语义

### 10.1 版本陷阱

OpenCV 3.3 的 `cv::minAreaRect` 返回 `RotatedRect`，其 `angle` 约定与 OpenCV ≥4.5（`[0, 90)`）**不同**（3.x 为 `(-90, 0]`），`size.width/height` 也不保证 `width ≥ height`。**不要把 `RotatedRect.angle` 直接当项目 yaw 用**。

推荐做法（版本无关、且能一次成型项目约定）：

```cpp
cv::RotatedRect rr = cv::minAreaRect(points_pixel);        // 输入为像素索引 (u=col, v=row)
cv::Point2f v[4]; rr.points(v);                            // 4 顶点（ROI 局部像素坐标）
// ① 加回 ROI 原点，转到世界系（§6 公式）
// ② 用两相邻边定轴：e1 = P1-P0, e2 = P2-P1
// ③ 若 |e1| < |e2| 交换，使 u 为长轴，v = rot90(u) = (-u.y, u.x)（右手/逆时针）
// ④ yaw = atan2(u.y, u.x)，归一化到 [0, π)
// ⑤ center = mean(4 顶点)
// ⑥ corners（与 ComputeClusterOBB 语义一致）：
//     C0 = c - (L/2)u - (W/2)v      // 左下
//     C1 = c + (L/2)u - (W/2)v      // 右下
//     C2 = c + (L/2)u + (W/2)v      // 右上
//     C3 = c - (L/2)u + (W/2)v      // 左上
```

### 10.2 与现有约定的一致性 `[已验证]`

| 约定 | 现有代码 | 新实现 |
|------|---------|--------|
| yaw 范围 | PCA `obb_angle = atan2(主轴)` ∈ `[−π/2, π/2)`（`ElevationMapGroundFilter.h:304`） | Prompt §14 要求统一为 `[0, π)`；**新字段用 `[0, π)`，PCA 字段不动**（A/B 必须 mod π 比较） |
| length ≥ width | `ComputeClusterOBB` Step 6 显式保证（`:2530` 附近） | 同一规则（③） |
| corners 顺序 | `C0=左下, C1=右下, C2=右上, C3=左上`（`:2549~:2552`） | 同一顺序（⑥） |
| `C0→C1` 为长轴 | 下游依赖：`DrawMapAndAllOverlay` miss 重建用 `corners[1]-corners[0]` 反算 yaw、`corners[3]-corners[0]` 反算 width（`DebugViewer.cpp:1620~1660`）；`RefineStaticObbGeometry` 同源 | 必须成立（⑥ 构造保证） |
| 角度比较 | `static_obb_refinement.h` 已有 `NormalizeYaw180` / `AngularDistance180`（无向轴） | A/B 的 Δyaw 必须调用它们，禁止 `fabs(a−b)` |

`corners` 顺序错误会直接破坏 miss 帧可视化与 UDP 输出的框方向，属于本阶段最高优先级的兼容性检查项。

---

## 11. fallback 链（三档）

```text
IMG OBB (Raw Image Contour)       判定：has_img_obb
        │ 失败原因：ROI 空 / used_pixel_count < 2 / 点共线(W≈0) / L < 1 pixel
        ▼
PCA OBB (ComputeClusterOBB)       判定：cluster.has_obb（n_cells ≥ 3，实际恒真）
        ▼
AABB                              （现有 !has_obb 分支）
```

- fallback 原因必须进 `GridCluster.img_obb_reason`（枚举或短字符串）并逐帧日志化，否则无法判断“方案没效果”与“方案大多走了 fallback”。
- **第一版不允许**为了减少 fallback 而放宽到“1 像素也出框”：单像素 `minAreaRect` 在 3.3 下会给出 `size=(0,0)`，其 `yaw` 无意义。

---

## 12. 性能与复杂度分析（Prompt §1/§18）

| 环节 | 复杂度 | 实测/估算（`debug_config.yaml`） |
|------|--------|-------------------------------|
| 图像生成 | `O(N)`，与既有遍历合并 → **额外成本 ≈ 1 次 store/点** | `N ≈ 20w` → ≈ 0.2M 次写（可忽略） |
| 图像内存 | `O(rows·cols)` | `60 × 71 = 4,260 B`（一张 4KB 图，常驻） |
| Cluster→ROI | `O(n_cells)`（求 cell row/col 极值） | 3~5 cells → 个位数 |
| ROI 拷贝 + findContours | `O(ROI 面积)` | 3~5 cells → ROI ≈ 4×4 ~ 6×5 ≈ 16~30 px |
| minAreaRect | `O(k log k)`，`k = used_pixel_count` | ≤ 30 |
| **总计** | `O(N + Σ ROI_area)` | 20 clusters × 30 px = 600 px → 微秒级 |

**不存在 `O(cluster_count × point_count)`**；`Raw PointCloud` 仍只被处理 3 次（与现状相同），图像不再触发任何点云遍历。

---

## 13. 边界情况与风险清单（Prompt §12/§16 的“3~5 cell、少量点”）

| ID | 情况 | 现状分析 | 处理 |
|----|------|---------|------|
| R1 | 3~5 cells 但障碍物点只有 3~8 个 | 像素多为孤立点，`findContours` → 多个 1 像素轮廓 | §8 `union` 策略；`large` 策略会退化 |
| R2 | 点几乎共线（路沿石顶面只有一排点） | `minAreaRect` 的 `W ≈ 0`（1 像素跨度） | 保留原始 `W`，`has_img_obb=true` 但日志记 `W<0.1m`；**不做**人为加宽（避免掩盖真实语义） |
| R3 | 物体实际静止但点云稀疏抖动 | 像素集合逐帧变化（点可能与上一帧错开 1 像素） | 这正是要验证的命题；用 `Δyaw / ΔL / ΔW / Δcenter` 逐帧统计（§14） |
| R4 | cluster 靠近 ROI 边界（`x→8m`、`|y|→3m`）与 ROI 外点 | `ReclassifyPointCloud` 的“ROI 外/无 cell”点无像素 | 日志统计该帧此类点数；边界 cluster 结论需谨慎 |
| R5 | 邻近障碍物点落入 ROI | padding=1 后概率上升 | §9 关联规则（像素必须落在 cluster 矩形内）隔离 |
| R6 | 同一物理物体被 8 邻域 BFS 拆成 2 个 cluster | 各自 ROI 只含自己那份像素 | 属既有聚类行为，本阶段不改（Prompt §19）；记录 `n_cells` 与 boxes 对照 |
| R7 | `W` 系统性偏小（§0 C8） | PCA `W ≥ 0.1m`（cell 中心跨度），IMG `W` 可低至 0.03m | 记录差异；UDP/规划侧影响需评审（本阶段不改语义） |
| R8 | 竖直结构/人腿点进入 ROI | `is_vertical_structure` cell 不进 cluster，但其像素在 ROI 内 | 由 §9 关联（像素落在 cluster 矩形内）大概率排除；若仍在，日志可查 |
| R9 | 定位/HDMap/Tracker 相关 | 本方案完全不涉及 | 不动（Prompt §19） |

---

## 14. 与当前 PCA OBB 的 A/B Debug 方案（Prompt §15/§16）

### 14.1 数据结构（只增不改）

`GridCluster` 新增一组**纯诊断/并列**字段（`ElevationMapGroundFilter.h`，紧邻既有 `obb_lambda_*` 之后）：

```cpp
// ---- Phase 3-B' Raw Point Image Contour OBB（并列诊断字段，不影响 PCA 字段）----
bool  has_img_obb = false;
int   img_roi_rows = 0, img_roi_cols = 0;        // ROI 尺寸（含 padding）
int   img_roi_min_row = 0, img_roi_max_row = 0;  // 未 padding 的 cluster cell 矩形
int   img_roi_min_col = 0, img_roi_max_col = 0;
int   img_pixel_count = 0;        // ROI 内被占据像素数
int   img_contour_count = 0;      // findContours 总轮廓数
int   img_used_contour_count = 0; // 关联成功轮廓数
int   img_used_pixel_count = 0;   // 关联轮廓像素并集大小
float img_obb_center_x = 0, img_obb_center_y = 0;
float img_obb_length = 0, img_obb_width = 0;
float img_obb_yaw_rad = 0;        // [0, π)
Point2D img_obb_corners[4] = {};
const char* img_obb_reason = "";  // fallback 原因
```

### 14.2 日志（A/B 一行一 cluster，便于离线统计跳变）

```text
[ImgOBB] frame=%d cluster=%d cells=%d pts=%d | ROI %dx%d px=%d K=%d K_used=%d px_used=%d
         | PCA  yaw=%.1f L=%.2f W=%.2f c=(%.2f,%.2f)
         | IMG  yaw=%.1f L=%.2f W=%.2f c=(%.2f,%.2f) reason=%s
         | dYaw=%.1f (mod180) dL=%.2f dW=%.2f
[ImgOBB-Track] frame=%d track=%d cluster=%d dYaw_prev_frame=%.1f  dL=%.2f  dCorner=%.3f
```

第二行按 **Track** 统计（用 `track.cluster_id` 关联，`track.cpp:423` 已写回），直接产出 Prompt §16 要的“同一 Track 跨帧 yaw/L/corner 跳变”指标。

### 14.3 可视化（用户提示：参考 `DrawMapAndAllOverlay` 的坐标系）

分两块，**互不干扰**：

1. **在既有 overlay 图里加一层（推荐，改动最小）**
   - `DrawClusterOverlay()`（`DebugViewer.cpp:1265`）：画 cluster 的 IMG OBB（例如青色）/ PCA OBB（现有紫/白）；
   - `DrawMapAndAllOverlay()`（`DebugViewer.cpp:1554`）：画 Track 的 IMG OBB。
   - 复用 `WorldToPixel()`（`DebugViewer.cpp:379`）与 `ComputeGridImageGeometry()`（`:358`），无需新坐标约定。

2. **新增“Raw Point Image”窗口**（算法图，非放大显示）
   - 用 `cv::resize`（最近邻，`INTER_NEAREST`）放大 `kGridPixelScale=10` 倍，再按下面的**精确整数映射**贴到 overlay 坐标里：

     ```text
     映射结论（debug_config.yaml 参数代入）：
       eff_x_min = 0.9, roi_y_min = -3.0, res = 0.1
       rows = 60, cols = 71
       blind_cols = 9 → px_blind_offset = 90
       img_w = 90 + 71*10 + 100 = 900, img_h = 60*10 + 200 = 800

       算法像素 (row r, col c) → 显示块：
         x ∈ [140 + 10c, 150 + 10c)
         y ∈ [200 + (rows-1-r)*10, 210 + (rows-1-r)*10)     // 显示时 y 翻转
     ```

   - 注意 `WorldToPixel` 与算法映射差“半个 cell”（其 `py` 用 `rows` 而非 `rows-1`，见 `DebugViewer.cpp:399` 被注释的旧写法）。**画 ROI 框与算法像素块必须使用上面同一套整数块映射**，否则框与点会差 5px。

### 14.4 离线验证口径（建议直接作为本阶段验收）

| 指标 | 含义 | 期望 |
|------|------|------|
| `dYaw_jitter` | 同一 Track 相邻帧 `Δyaw`（mod 180）分布 | IMG 显著小于 PCA（Prompt §16 目标：消除 −22°→0°→−32° 级跳变） |
| `dCorner_step` | 4 角点平均位移（逐帧） | IMG 更小 |
| `dL / dW` | 尺寸步进 | IMG 不劣化（注意 C8/R7 的偏小语义） |
| `fallback_rate` | `has_img_obb=false` 占比 | 越低越好；若 >30% 说明稀疏点云下方案不可行 |
| `ID 连续性` | Track ID 序列 | 必须与 baseline **完全一致**（本阶段不改 tracker 输入即可保证） |

重点观察对象：Prompt 点名的 Track `10008 / 10010 / 10011`（及日志中 3/4/5 cell 排列变化的 Track）。

---

## 15. 对 `RefineStaticObbGeometry()` 的最小修改建议 + 可停用逻辑（Prompt §19/§20-19/20）

### 15.1 最小修改（推荐顺序）

1. **停用（一行）**：`include/static_obb_refinement.h:50` → `kObbRefineEnable = false`。
   - 效果：`RefineStaticObbGeometry()`（`SuTengDriver.cpp:1606`）在开头即 `outTracks = m_tracker.vtrackings` 后返回 → **输出即 RAW**，不引入任何历史 yaw/L/W 逻辑，风险为零。
   - 该函数内维护的 map 系 yaw 记忆（`MapAnchoredTrack::has_static_yaw / static_yaw_map_rad`）随之不再写入；`UpdateMapAnchors()`（`:1223`）本体与 Phase 2 锚点/map 位置/关联**必须保留**。
2. **新增（替换其位置）**：`SuTengDriver` 内新增 `ApplyContourObbToOutput(clusters, outTracks)`，只做一件事：
   ```text
   for (Track& t : outTracks)
       if (t.lastSeen == 0 && 能找到 cluster_by_id[t.cluster_id] && cl.has_img_obb)
           t.pos_x/pos_y = img_obb_center; t.depth = img_obb_length;
           t.width = img_obb_width; t.corners = img_obb_corners;   // 其余字段一律不动
   ```
   - **不修改** `motion_state / id / age / lastSeen / vx / vy / map_*`；**不写回** `m_tracker.vtrackings`。
   - 调用点：`SuTengDriver.cpp:1095`（原 `RefineStaticObbGeometry` 处），仍在 `UpdateMapAnchors()`（`:1088`）之后 → 锚点永远基于 RAW 几何的时序约束保持不变。
3. **不改** `ConvertClustersToTrackedObstacles()`（`:2213`）：`detections` 仍用 PCA OBB → tracker 关联/速度/ID 与 baseline 完全一致（这正是 §14.4 “ID 连续性必须一致”的前提）。
4. 后续（下一阶段，需单独评估）：若 IMG OBB 被证明更优，再把 `detections` 也切到 IMG OBB，让关联与 UDP 一致同源。

### 15.2 可以暂时停用的历史逻辑清单

| 项 | 位置 | 处理 |
|----|------|------|
| STATIC OBB 方向决策（PCA/HISTORY/RAW yaw 三选一） | `static_obb_refinement.h` + `SuTengDriver.cpp:1606` | 由 `kObbRefineEnable=false` 整体停用 |
| 历史 yaw 记忆（map 系） | `MapAnchoredTrack::has_static_yaw / static_yaw_map_rad`（`historical_feedback.h`） | 随函数停用；字段保留不删（便于回滚） |
| 历史 cluster 轮廓融合 / cell 补充 | `historical_geometry.h` + `ApplyHistoricalGeometry`（调用点 `SuTengDriver.cpp:1101` 已注释） | 保持注释状态 |
| Historical Feedback 实验 | `ComputeHistoricalFeedback`（`SuTengDriver.cpp:1105` 段已注释） | 保持注释状态 |
| yaw jump / stable yaw / EMA 等阈值 | 散落在 `static_obb_refinement.h` 常量 | 停用后自然不生效；**不新增**同类阈值 |

---

## 16. 实施改动清单（下一阶段执行时用，本阶段不改）

| 文件 | 改动性质 | 内容 |
|------|---------|------|
| `include/ElevationMapGroundFilter.h` | 新增字段/方法 | `m_rawPointImage`（`std::vector<uint8_t>`，`rows*cols`）+ `GetRawPointImage()` 只读访问器；`GridCluster` 增加 §14.1 的 `img_obb_*` 字段；声明 `ComputeRawImageContourObb(cluster)` |
| `include/ElevationMapGroundFilter.cpp` | 少量新增 | `BuildGrid()` 中 `resize` 图像并清零（与 `m_grid_rows/cols` 同处）；`ReclassifyPointCloud()` 循环内 1 行写像素（§3）；`ClusterObstacleGrid()`/`BuildClusterFromCells()` 之后调用一次 `ComputeRawImageContourObb()` 填充字段 |
| `include/raw_point_image_obb.h/.cpp`（**新文件**） | 新增 | 把 `findContours/minAreaRect` 与像素↔世界换算全部封装在此，**OpenCV 依赖只出现在这里**（保持 `ElevationMapGroundFilter` 不依赖 OpenCV 的现状）；接口：`RawImageObbResult ComputeRawImageContourObb(const std::vector<uint8_t>& img, int rows, int cols, const std::vector<int>& cell_indices, int grid_cols, const ElevationGridConfig& cfg, float eff_roi_x_min, RawContourStrategy strategy)` |
| `include/static_obb_refinement.h` | 一行 | `kObbRefineEnable = false` |
| `include/SuTengDriver.cpp` | 替换调用 | 新增 `ApplyContourObbToOutput()`，替换 `:1095` 处的 `RefineStaticObbGeometry`；新增 A/B 日志（§14.2） |
| `include/DebugViewer.h/.cpp` | 新增一层 | `DrawClusterOverlay` / `DrawMapAndAllOverlay` 内加 IMG OBB（青色）+ 可选新窗口显示 Raw Point Image（§14.3） |
| `CMakeLists.txt` | **无需改动** | `${OpenCV_LIBS}`（`:120`）已含 `imgproc`；`libopencv_imgproc.so.3.3` 已确认存在；`AUX_SOURCE_DIRECTORY(./include SRC)`（`:66`）会自动收集新 `.cpp` |
| 配置 | 新增开关 | `kRawImageContourObbEnable`（代码常量）+ 可选 `padding` / `strategy` 常量；**不引入 morphology/阈值参数** |

---

## 17. 待确认问题（需要用户拍板或后续实测）

1. §0.1 的 3 个决策点。
2. `W` 系统性偏小（C8/R7）是否可接受？若不可接受，是否需要“`W += 1 pixel`”这类补偿（会引入一个非物理参数，本阶段默认不加）。
3. 是否需要**同时**输出“全点占据图”作为可视化对照层（Prompt §4 字面版）？
4. A/B 验证是跑 pcap 离线（当前 `pcapRunningModel` 路径）还是真机 online？—— 由日志+DebugViewer 决定即可，但需确认配置。
5. 是否同意第一版**只改输出层**（`outTracks`），暂不让 `detections` 使用 IMG OBB？

---

## 18. 最终目标架构（与 Prompt §21 对齐，落到本仓库真实函数名）

```text
                    ProcessPcapCloud()                       SuTengDriver.cpp:995
                         │  pFilteredPointCloud（车体系）
             ┌───────────┴───────────────────────────────┐
             ▼                                           ▼
   ProcessWithObstacleDetection                  [新增] ReclassifyPointCloud 内
     BuildGrid :145 ── resize 图像 ───────────────→  m_rawPointImage[row*cols+col]=1
     Ground/Slope/Region/Mask/GroundRef                    （0.1m/pixel, None 新增遍历）
     AnalyzeVerticalOccupancy :1418
     ReclassifyPointCloud :1843 ─── 写像素 ────────→
     ClusterObstacleGrid :1977 → BuildClusterFromCells :2145
             │                     ├─ ComputeClusterOBB :2377      （PCA，A/B 基线，不改）
             │                     └─ [新增] ComputeRawImageContourObb()
             │                             └─ cluster bbox → ROI(padding=1)
             │                                → findContours → 关联 → minAreaRect
             ▼
        GridCluster{obb_* , img_obb_*}
             │
             ├─ HDMap filter（不动）
             ├─ ConvertClustersToTrackedObstacles（不动，仍用 PCA → tracker 不变）
             ├─ m_tracker.update（不动）→ UpdateTrackMotionStates（不动）→ UpdateMapAnchors（不动）
             └─ [替换] ApplyContourObbToOutput(clusters, outTracks)   ← 原 RefineStaticObbGeometry 位置
                        │
                        └─ DrawMapAndAllOverlay / m_visBuffer / ConvertTrackToS2ObstacleBox
                           （Debug + UDP 消费 IMG OBB；tracker 内部永远 RAW）
```

三条核心原则（与 Prompt 一致）：

1. **Raw PointCloud 只为图像处理一次**（且是复用既有遍历，不是新增遍历）。
2. **Cluster 不重新寻找 Raw Point，只告诉图像“目标在哪里”**（cluster cell 矩形 → ROI）。
3. **`findContours` 的对象是 Raw Point Image 中被真实点投影占据的像素，不是 Grid Cell**；当前阶段不使用历史轮廓／历史 yaw／EMA／复杂 geometry 阈值。

---

## 附：Prompt §20 的 20 项要求 → 本文对应章节

| Prompt 项 | 本文章节 |
|-----------|---------|
| 1 当前代码数据流 | §1 |
| 2 BuildGrid 如何遍历 Raw Point | §2 |
| 3 Raw Point Image 最合适的插入位置 | §3 |
| 4 能否复用 BuildGrid 的遍历 | §3（结论：语义不允许，改为复用 `ReclassifyPointCloud` 遍历） |
| 5 Image 0.10m/pixel 坐标定义 | §5 |
| 6 Grid ↔ Image 坐标映射 | §6 |
| 7 Cluster bbox → Image ROI | §7 |
| 8 ROI padding | §7 |
| 9 findContours 输入 | §8 |
| 10 多 contour 如何定目标 | §9 |
| 11 `minAreaRect()` 输出转换 | §10.1 |
| 12 yaw `[0°,180°)` 转换 | §10.2 |
| 13 corner 顺序兼容 | §10.2 |
| 14 20w+ points 性能分析 | §12 |
| 15 多 Cluster 复杂度分析 | §12 |
| 16 3~5 Cell / 少量 Raw Points 边界 | §13（R1/R2） |
| 17 与 PCA OBB 的 A/B debug 方案 | §14 |
| 18 fallback 方案 | §11 |
| 19 对 `RefineStaticObbGeometry()` 的最小修改建议 | §15.1 |
| 20 哪些历史 yaw/geometry refinement 逻辑可停用 | §15.2 |
