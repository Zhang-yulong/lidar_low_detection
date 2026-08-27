# ElevationMapGroundFilter 可视化调试方案设计

## 1. 设计目标

目前 `ElevationMapGroundFilter::ProcessWithObstacleDetection()` 已经实现了基于 Grid 的地面分割与低矮障碍物检测，但缺少直观的可视化调试工具，导致算法参数（如坡度阈值、高度阈值、Region Growing、Ground Reference 等）只能依靠日志分析，调试效率较低。

建议将可视化作为算法调试的重要组成部分，而不是简单地输出 Ground PointCloud。

整个算法流程如下：

```text
Point Cloud
      │
BuildGrid
      │
Grid
      │
ComputeSlope
      │
RegionGrowing
      │
Ground Label
      │
Occupancy Analysis
      │
Obstacle Candidate
      │
Cluster
```

因此，可视化也建议按照算法流程逐层展示。

------

# 2. 第一层：Ground Mask（最高优先级）

建议在完成 `GenerateGroundMask()` 后，对整个 Grid 的分类结果进行可视化。

每个 Grid Cell 使用颜色表示：

- 绿色：Ground
- 红色：Non-Ground
- 灰色：Invalid

示意图：

```text
□□□□□□□□□□

GGGGGGGGGG

GGGGGGGGGG

GGGRRRRGGG

GGGRRRRGGG

GGGGGGGGGG
```

该可视化可以快速验证：

- Region Growing 是否正常扩展；
- 地面是否被错误截断；
- 是否出现大量误分类。

这是整个 Ground Filter 调试过程中最重要的一幅图。

------

# 3. 第二层：Height Map（Grid 高程热力图）

Ground Mask 只能看到分类结果，而无法反映高程信息。

建议同时绘制每个 Grid 的：

```text
min_z
```

采用热力图表示：

- 蓝色：低
- 绿色：中
- 黄色：高
- 红色：最高

例如：

```text
蓝 蓝 蓝 蓝

蓝 蓝 黄 黄

蓝 黄 红 红
```

该图能够帮助检查：

- ComputeGroundHeight 是否正确；
- Grid 高程是否连续；
- 是否存在异常跳变。

------

# 4. 第三层：Slope HeatMap（坡度热力图）

建议将每个 Grid Cell 的坡度进行颜色编码：

```text
cell.slope
```

颜色建议：

```text
蓝 → 绿 → 黄 → 红
```

分别表示：

- 小坡度
- 中坡度
- 大坡度
- 极大坡度

该图能够快速定位：

- Region Growing 为什么停止；
- 是否因为动态坡度阈值设置过低；
- 是否因为局部噪声导致坡度异常。

尤其当前算法已经引入：

```cpp
GetDynamicSlopeThreshold()
```

Slope HeatMap 对调试具有重要意义。

------

# 5. 第四层：Ground Reference

目前算法已经增加：

```cpp
ComputeGroundReference()
```

建议将：

```text
ground_reference_z
```

绘制成连续热力图。

颜色示例：

- 蓝：-1.30
- 绿：-1.20
- 黄：-1.10
- 红：-1.00

主要用于验证：

- Ground Reference 是否连续；
- 插值是否成功；
- Smooth 是否有效；
- 是否出现局部跳变。

对于当前 Ground Reference 算法，这一层具有非常高的调试价值。

------

# 6. 第五层：Obstacle Candidate

在：

```cpp
AnalyzeVerticalOccupancy()
```

完成之后，不建议直接显示点云。

更推荐直接显示 Grid 分类结果。

颜色建议：

- 绿色：Ground
- 黄色：Obstacle Candidate
- 红色：Vertical Structure
- 灰色：Invalid

示意图：

```text
GGGGGGGGGG

GGGYYYGGGG

GGGRRRGGGG

GGGRRRGGGG
```

这样可以直接验证：

- 哪些 Cell 被判定为 Obstacle Candidate；
- 哪些 Cell 被判定为 Vertical Structure；
- 是否存在误分类。

Grid 可视化比点云更加适合算法调试。

------

# 7. 第六层：Cluster 可视化（最高优先级）

聚类完成后，不建议只打印：

```text
Cluster 0

Cluster 1

Cluster 2
```

建议每个 Cluster 使用不同颜色显示：

```text
111111

111111

333333

222222

222222
```

例如：

- Cluster0：蓝色
- Cluster1：绿色
- Cluster2：黄色
- Cluster3：紫色

可以快速判断：

- BFS 是否正确；
- Cluster 是否发生错误合并；
- 是否出现碎片化。

这是低矮障碍物调试过程中价值最高的可视化之一。

------

# 8. 第七层：Bounding Box

聚类结束后，可以绘制：

- AABB
- OBB（推荐）

由于算法已经计算：

```cpp
obb_angle

obb_corner
```

因此建议直接显示 OBB。

这样可以检查：

- 障碍物方向是否正确；
- 长宽是否合理；
- 是否存在旋转错误。

------

# 9. 第八层：PointCloud Overlay

最后，将：

- Ground Point（绿色）
- Obstacle Point（红色）

叠加到 Grid 上。

示意图：

```text
+------------------+

|■ ■ ■ ■ ■

|■ ■ □ □ □

|■ ● ● ●

|□ ● ● ●
```

该层主要用于最终效果验证：

- Grid 与点云是否一致；
- Ground Mask 是否准确；
- Bounding Box 是否覆盖目标。

------

# 10. 第九层：Tracker Overlay（跟踪结果点云叠加图）

> 注：该层已实际落地到 `DebugViewer::DrawOverlay()` 的 `TrackedObstacle` 重载版本，
> 配置项为 `DebugViewer.TrackerOverlay`（enable / show / save），窗口与保存文件名为 `TrackerOverlay`。

跟踪器（`SimpleTracker`）为障碍物分配跨帧稳定 ID 之后，将以下内容叠加到与第 1~8 层完全一致的 Grid 坐标系上：

- Ground Point（绿色）
- Obstacle Point（红色）
- 跟踪器 `corners[4]` 多边形（白色线框）
- 跟踪器中心点（黄色）
- 跟踪器稳定 ID（白色文本：优先 `t.id`，其次 `t.cluster_id`，最后为数组索引）
- 速度向量（青色箭头，按 `t.vx / t.vy` 与像素/米比例绘制）

## 10.1 与第八层的区别

| 对比项   | 第八层 Overlay              | 第九层 TrackerOverlay                  |
| -------- | --------------------------- | -------------------------------------- |
| 数据来源 | `std::vector<GridCluster>`  | `std::vector<TrackedObstacle>`         |
| 包围框   | Cluster 的 OBB / AABB       | Tracker 的 `corners[4]` 多边形          |
| ID       | Cluster 临时 ID（每帧跳变） | Tracker 稳定 ID（跨帧一致）             |
| 速度     | 无                          | 绘制速度向量箭头                       |

## 10.2 运行时调用位置

在 `SutengDriver::ProcessPcapCloud()` 中、`m_tracker.update()` 之后调用：(2026年8-27日已修改调用位置)

```cpp
if (m_debugViewer)
{
    m_debugViewer->DrawOverlay(*pGroundCloud, *pObstacleCloud,
                               m_tracker.vtrackings,
                               m_pElevationMapGroundFilter->GetConfig());
}
```

> ⚠️ 注意：该层是"点云叠加 + 跟踪框"的叠加图。即使 `m_tracker.vtrackings` 为空，
> 地面点（绿）与障碍物点（红）仍会照常绘制；`trackers` 只控制白色跟踪多边形/中心/ID 部分。

## 10.3 调试价值

- 验证 `SimpleTracker` 跨帧 ID 是否稳定；
- 验证速度向量方向与大小是否合理；
- 与第八层对比"原始 Cluster"与"跟踪结果"的差异。

------

# 11. 建议实现统一的 DebugViewer

建议不要只输出 GroundCloud，而是在每一帧生成如下调试界面：

```text
DebugViewer

├── ① Grid Height Map
│
├── ② Ground Mask
│
├── ③ Slope HeatMap
│
├── ④ Ground Reference
│
├── ⑤ Obstacle Candidate
│
├── ⑥ Cluster Label
│
├── ⑦ Bounding Box
│
├── ⑧ Final PointCloud Overlay
│
└── ⑨ Tracker Overlay（跟踪结果叠加）
```

其中：

- ①～⑥ 主要用于算法调参；
- ⑦、⑧ 用于验证最终检测效果；
- ⑨ 用于验证跟踪器的跨帧 ID 稳定性。

------

# 12. 可视化优先级建议

| 优先级 | 可视化内容                      | 调试价值                                   |
| ------ | ------------------------------- | ------------------------------------------ |
| ⭐⭐⭐⭐⭐  | Ground Mask（地面/非地面 Grid） | 验证 Region Growing 是否正确               |
| ⭐⭐⭐⭐⭐  | Cluster Label（聚类结果）       | 验证聚类是否合理                           |
| ⭐⭐⭐⭐☆  | Height Map（min_z 热力图）      | 检查地形和高程统计                         |
| ⭐⭐⭐⭐☆  | Ground Reference                | 验证参考地面高度连续性                     |
| ⭐⭐⭐⭐☆  | Obstacle Candidate              | 检查 `AnalyzeVerticalOccupancy()` 分类结果 |
| ⭐⭐⭐☆☆  | Slope HeatMap                   | 分析 Region Growing 停止原因               |
| ⭐⭐☆☆☆  | PointCloud Overlay              | 验证最终检测效果                           |
| ⭐⭐☆☆☆  | OBB / AABB                      | 验证障碍物包围框                           |
| ⭐⭐☆☆☆  | Tracker Overlay                 | 验证跟踪器跨帧 ID 稳定性                   |

## 总结

结合当前项目的开发阶段，建议优先实现一个轻量级的二维 Grid DebugViewer。每个 Grid Cell 使用颜色表示不同状态，即可覆盖 Ground Filter、Ground Reference、Obstacle Candidate、Cluster 与 Tracker 等核心模块的调试需求。相比直接显示三维点云，这种方式实现简单、运行效率高，而且能够更加直观地反映算法每一步的内部状态，是后续参数优化和算法迭代的重要工具。

------

# 13. 坐标系与像素映射实现说明

第 1~9 层共用同一套"物理坐标 → 像素坐标"映射，保证各层图像可以逐像素对齐比对。
该坐标系由三个内部函数共同实现：`ComputeGridImageGeometry()`、`WorldToPixel()`、`AddBackGround()`。

## 13.1 统一坐标系约定

- 物理坐标：X 为前向（正前方），Y 为左右（左为 +Y）；
- 网格索引：`col` 沿 X 方向（前向），`row` 沿 Y 方向（左右）；
- 像素方向：X+ 向右，Y+（左侧）向上；
- 车前盲区：物理 `x ∈ [0, blind_dist_x)` 不参与建网格，在图像上单独占一段像素宽度。

## 13.2 ComputeGridImageGeometry() —— 计算图像几何

```cpp
void DebugViewer::ComputeGridImageGeometry(const ElevationGridConfig& gridCfg,
                                           int& rows, int& cols,
                                           int& px_blind_offset,
                                           int& img_w, int& img_h) const;
```

内部操作：

1. `blind_dist_x = car_half_x + body_filter_x_threshold`：车前盲区距离（物理米）；
2. `blind_cols = round(blind_dist_x / grid_resolution)`：盲区占用的网格列数；
3. `px_blind_offset = blind_cols * kGridPixelScale`：盲区在图像上的像素偏移；
4. `rows = ceil((roi_y_max - roi_y_min) / grid_resolution)`、`cols = ceil((roi_x_max - blind_dist_x) / grid_resolution)`：与算法 `m_grid_rows / m_grid_cols` 保持一致；
5. 图像尺寸：
   - `img_w = px_blind_offset + cols * kGridPixelScale + expand_img_w`（盲区 + 有效网格 + 右边距）
   - `img_h = rows * kGridPixelScale + expand_img_h`（有效网格 + 上下边距）

## 13.3 WorldToPixel() —— 世界坐标 → 像素坐标

```cpp
void DebugViewer::WorldToPixel(float wx, float wy, int& px, int& py,
                               const ElevationGridConfig& gridCfg) const;
```

内部操作（先调用 `ComputeGridImageGeometry()` 拿到几何参数，再做线性映射）：

```text
px = px_blind_offset + (wx - blind_dist_x) * inv_res * kGridPixelScale + expand_img_w / 2

#(rows - 1)从图像上看会导致Car(0,0)这个点上移一个grid
py = ((rows - 1) - (wy - roi_y_min) * inv_res) * kGridPixelScale + expand_img_h / 2

#现在用这个
float py_f = ((rows) - (wy - gridCfg.roi_y_min) * inv_resolution) * kGridPixelScale
                 + (expand_img_h / 2);
```

其中 `inv_res = 1 / grid_resolution`。

- 公式一：物理 X（前向）减去盲区距离后，按 `inv_res * kGridPixelScale`（像素/米）映射到像素列，再加左边距 `expand_img_w / 2`；
- 公式二：物理 Y 先通过 `(wy - roi_y_min) * inv_res` 得到行号，再用 `(rows - 1) - 行号` 翻转，使图像顶部为 Y+（左侧）、底部为 Y-（右侧），最后加 `expand_img_h / 2`；
- 与第 1~6 层绘制 Grid Cell 的 `GridToPixel()` 完全一致，保证包围盒/点云与网格 Cell 逐像素对齐；
- 输出前对 `px / py` 进行 `[0, img_w-1] × [0, img_h-1]` 边界裁剪，防止越界绘制。

## 13.4 AddBackGround() —— 绘制坐标轴与网格背景

```cpp
void DebugViewer::AddBackGround(cv::Mat& image, const int img_w, const int img_h,
                                const ElevationGridConfig& gridCfg);
```

内部操作：

1. 计算物理原点 `(0, 0)` 的像素位置：
   - `px_origin_x = (0 - roi_x_min) * inv_res * kGridPixelScale + expand_img_w / 2`
   - `px_origin_y = round((0 - roi_y_min) * inv_res) * kGridPixelScale + expand_img_h / 2`
2. 绘制坐标轴（红色）：X 轴向右（标注 `Col++ (X+)`），Y 轴向上（标注 `Row++ (Y+ Left)`），并在原点画实心圆与 `Car(0,0)` 标签；
3. 按物理坐标循环绘制 1 米大网格线（灰色）：水平线沿物理 Y 从 `roi_y_max` 递减到 `roi_y_min`，垂直线沿物理 X 从 `roi_x_min` 递增到 `roi_x_max`；
4. 每条网格线在坐标轴处绘制刻度与物理坐标标签（如 `0.0 / 1.0 / 2.0`）；
5. 网格线按 `(phys - roi_min) * inv_res * scale` 计算，与 `WorldToPixel()` / `GridToPixel()` 的 Cell 边界对齐，因此网格线恰好落在整米 Cell 边界上，便于读数。

> 注意：第 1~6 层在画完 Cell 后调用 `AddBackGround()`（网格线覆盖在 Cell 上）；
> 第 7~9 层（BoundingBox / Overlay / TrackerOverlay）先调用 `AddBackGround()` 再画包围框/点云，
> 避免灰色网格线遮挡障碍物数据。

------

# 14. show 模式窗口布局（初始尺寸与自动排布）

`ShowOrSave()` 在 `viewCfg.show` 为真时创建 OpenCV 窗口。为避免启动时窗口过小、
以及多个窗口（例如 Overlay 与 TrackerOverlay 同时开启）重叠，DebugViewer 对窗口做了
**初始尺寸固定**与**自动上下排布**：

- `CountShownWindows()`：统计本帧开启 show 的窗口总数（依据 `SELF_DEBUG_CONFIG` 中各层配置的 enable && show）；
- `LayoutWindow()`：仅窗口首次创建时执行一次——
  1. 按屏幕可用区域（`m_layoutMaxW × m_layoutMaxY`，默认 `1900 × 1000`，适配 1920×1080 屏幕）等比缩放图像（只缩小不放大），并用 `cv::resizeWindow()` 固定初始窗口大小，避免启动时窗口过小；
  2. 用 `cv::moveWindow()` 将窗口放到自动布局位置，多个窗口按“一上一下”垂直排布、互不重叠（Overlay 在上、TrackerOverlay 在下）；
  3. 垂直空间按窗口总数均分，保证所有开启的窗口同时可见；
- 首次定位后不再干预，用户仍可手动拖动 / 缩放窗口。

布局参数位于 `DebugViewer.h` 的成员变量中（`m_layoutMaxW` / `m_layoutMaxY` / `m_windowGap` 等），
可按实际屏幕分辨率调整。