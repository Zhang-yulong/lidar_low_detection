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

# 10. 建议实现统一的 DebugViewer

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
└── ⑧ Final PointCloud Overlay
```

其中：

- ①～⑥ 主要用于算法调参；
- ⑦、⑧ 用于验证最终检测效果。

------

# 11. 可视化优先级建议

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

## 总结

结合当前项目的开发阶段，建议优先实现一个轻量级的二维 Grid DebugViewer。每个 Grid Cell 使用颜色表示不同状态，即可覆盖 Ground Filter、Ground Reference、Obstacle Candidate 和 Cluster 等核心模块的调试需求。相比直接显示三维点云，这种方式实现简单、运行效率高，而且能够更加直观地反映算法每一步的内部状态，是后续参数优化和算法迭代的重要工具。