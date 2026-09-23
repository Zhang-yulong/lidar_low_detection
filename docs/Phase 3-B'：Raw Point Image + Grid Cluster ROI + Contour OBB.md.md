# Phase 3-B'：Raw Point Image + Grid Cluster ROI + Contour OBB

## 一、任务背景

当前低矮障碍物项目已经完成：

```text
UNKNOWN / MOVING / STATIC
```

三种 MotionState 的判断。

当前生产路径：

```text
SuTengDriver::ProcessPcapCloud()
    ↓
PointCloudTransform()
    ↓
m_pElevationMapGroundFilter->ProcessWithObstacleDetection()
    ↓
BuildGrid
    ↓
ComputeGroundHeight
    ↓
ComputeSlope
    ↓
RegionGrowing
    ↓
GenerateGroundMask
    ↓
ComputeGroundReference
    ↓
AnalyzeVerticalOccupancy
    ↓
ClusterObstacleGrid
    ↓
outputClusters
    ↓
HDMap filter
    ↓
ConvertClustersToTrackedObstacles
    ↓
m_tracker.update()
    ↓
UpdateTrackMotionStates()
    ↓
UpdateMapAnchors()
    ↓
RefineStaticObbGeometry()
    ↓
m_debugViewer->DrawMapAndAllOverlay
```

当前 Grid resolution 为：

```text
0.10m × 0.10m
```

当前 Cluster OBB 主要根据 Grid Cell Center 做 PCA。

低矮障碍物经常只有 3~5 个 Grid Cell，因此不同帧由于 Cell 排列变化，即使物体实际静止，PCA 得到的：

```text
yaw
length
width
corners
```

也会发生明显跳变。

之前尝试过：

1. 历史 Cluster 轮廓融合；
2. 历史 yaw 约束；
3. stable yaw；
4. 历史 yaw 统计；
5. yaw jump threshold；

但这些方案逐渐引入大量阈值和特殊逻辑。

现在重新采用之前：

```text
/home/zyl/echiev_low_lidar_detection/docs/MapFrameHistoricalFeedbackRawPointImage_FinalArchitecture.md
```

中提出的核心架构思想：

```text
Grid
    = 检测 / Ground Analysis / Cluster

Raw Point Image
    = 当前帧高保真空间结构

Map-frame Track
    = 时间上的 Track 状态
```

本阶段的目标非常纯粹：

> **不要再通过历史 OBB 去修正当前 PCA（涉及的函数是“SutengDriver::RefineStaticObbGeometry”，本次Phase 3-B'修改可以屏蔽这个函数），而是利用当前帧 Raw Point Cloud 投影得到的 Raw Point Image，在 Cluster 对应的 Image 区域内直接提取真实点云轮廓，再计算当前帧最小旋转矩形 OBB。**

------

由于MapFrameHistoricalFeedbackRawPointImage_FinalArchitecture.md生成的时间比较早，目前的项目中已经完成《2. Localization 前置条件（✅ 已确认真实接口）》和《13. UNKNOWN / STATIC / MOVING（📐 运动判断设计）》的代码设计，并已经验证过可以使用。本次Phase 3-B'不再思考这两方面内容。《12. Measurement vs Track State（📐 最小修改方案）》内部涉及数据来源不统一，我这里m_tracker.update函数修改的是TrackedObstacle中涉及雷达系的变量，UpdateTrackMotionStates函数修改的是TrackedObstacle中涉及地图系的变量。UpdateMapAnchors函数内设计了一个MapAnchoredTrack的侧表。我的意思是前面ClusterObstacleGrid函数生成的使用OBB计算的矩形框，Phase 3-B'阶段再生成由图像findContour生成的轮廓后计算最小矩形。这两种方式得到的雷达系变量我都想保留，进行对比。

# 二、最重要的设计原则

必须严格按照下面的数据流理解本任务：

```text
                Current Frame Raw PointCloud
                           │
                           │
                 PointCloudTransform()
                           │
             ┌─────────────┴─────────────┐
             │                           │
             ▼                           ▼
        BuildGrid()               Raw Point Image
             │                           │
             │                     0.10m/pixel
             │                           │
             ▼                           │
     Ground / Region /                 │
     ClusterObstacleGrid               │
             │                           │
             ▼                           │
       outputClusters                  │
             │                           │
             └──────────────┬────────────┘
                            ▼
                  Cluster Grid Coordinates
                            │
                            ▼
                 Locate corresponding
                 Image ROI directly
                            │
                            ▼
                    findContours()
                            │
                            ▼
                     minAreaRect()
                            │
                            ▼
                     Current OBB
```

核心原则：

> **Raw Point Cloud 只需要在生成 Raw Point Image 时扫描一次。**

之后：

> **Cluster 不允许再次扫描整帧 Raw PointCloud。**

------

# 三、明确禁止的实现方式

以下方案不要实现：

## 1. 禁止每个 Cluster 扫描整个 PointCloud

不要：

```cpp
for (cluster : clusters)
{
    for (point : pointCloud)
    {
        ...
    }
}
```

当前每帧 Raw Point 数量可能达到：

```text
200,000+
```

如果 Cluster 数量达到 20 个甚至更多，这种：

```text
O(cluster_count × point_count)
```

不可接受。

------

## 2. 禁止重新建立 Cluster Raw Point 列表

不要设计成：

```text
Cluster
    ↓
遍历所有 Raw Points
    ↓
point → Grid Cell
    ↓
判断 Cell 是否属于 Cluster
    ↓
Cluster Raw Points
```

本方案不需要这个步骤。

------

## 3. 禁止 Cluster Raw Points 再 Rasterize

不要设计成：

```text
Cluster Raw Points
    ↓
Rasterize
    ↓
Cluster Image
```

因为 Raw Point Image 应该在前面已经建立。

------

## 4. 禁止把 Grid Cell 本身画成白色矩形

不要：

```text
Cluster Cells
    ↓
每个 Cell 画成 10cm × 10cm 白块
    ↓
findContours()
```

这没有解决原始问题。

那样得到的仍然是 Grid 的离散几何，而不是 Raw Point Geometry。

------

# 四、正确理解 Raw Point Image

Raw Point Image 的含义是：

> **当前帧经过 PointCloudTransform 后，原始点云 XY 平面投影得到的图像。**

例如一个 Grid Cell 是：

```text
10cm × 10cm
```

其中真实 Raw Points 可能只有：

```text
        •
   •
             •
      •
```

那么 Image 中保存的是这些真实点对应的像素，而不是整个 Cell：

```text
██████████
██████████
██████████
```

所以：

```text
Grid
    → 告诉我们障碍物 Cluster 在哪里

Raw Point Image
    → 告诉我们该空间区域里面真实点云是什么样子
```

------

# 五、Raw Point Image 分辨率

当前第一版明确优先尝试：

```text
0.10m / pixel
```

原因：

```text
Grid resolution = 0.10m
Image resolution = 0.10m
```

使：

```text
Grid coordinate
    ↕
Image pixel coordinate
```

能够建立直接、稳定、容易验证的映射。

不要第一版擅自改成：

```text
0.05m/pixel
```

也不要为了“更高精度”自行增加 Image resolution。

本阶段首先验证：

> **0.10m/pixel 的 Raw Point Image 是否已经能够明显改善 OBB。**

如果未来实验显示 10cm Image resolution 仍然不足，再单独讨论 Image resolution。

------

# 六、Grid 与 Image 的坐标必须严格对齐

请首先分析：

```text
BuildGrid()
GridIndexToWorld()
Grid row / col
Grid origin
ROI
PointCloudTransform()
```

然后设计 Raw Point Image：

```text
world x/y
    ↓
image row/col
```

必须明确：

```text
pixel_x = ...
pixel_y = ...
```

以及：

```text
Grid[row][col]
```

与：

```text
Image[row][col]
```

是否可以做到一一对应。

如果当前 Grid row/column 与 Image row/column 存在：

- y 翻转；
- origin offset；
- ROI offset；
- row/column 交换；

必须明确处理。

不要默认二者一定完全相同。

------

提示一点：我认为Raw Point Image是不是可以参考DebugViewer::DrawMapAndAllOverlay函数的坐标系，毕竟这样方便可视化，查看哪里的问题。
# 七、Raw Point Image 的生成时机

建议数据流：

```text
PointCloudTransform()
        │
        ├──────────────→ BuildGrid()
        │
        └──────────────→ BuildRawPointImage()
```

或者在现有 BuildGrid 遍历 Raw Point 的过程中，同时完成 Image projection。

需要重点分析：

> **能否直接复用 BuildGrid 当前已经遍历 Raw Point 的过程，从而避免再次扫描 20w+ 点？**

如果 BuildGrid 当前本身已经：

```cpp
for (point : cloud)
{
    ...
}
```

那么优先考虑：

```cpp
for (point : cloud)
{
    existing Grid processing

    // 同时生成 Raw Point Image
}
```

而不是新增第二次完整 PointCloud traversal。

这是本方案重要的性能要求。

------

# 八、Image Pixel 如何表示 Raw Point

请分析第一版最简单的数据表示。

例如：

```text
Raw Point
    ↓
pixel(row,col)
    ↓
pixel = occupied
```

如果多个 Raw Points 投影到同一个 pixel：

```text
pixel = 255
```

即可。

第一阶段不需要在 pixel 中保存：

```text
point count
point index
z
intensity
```

除非代码分析证明后续 contour/OBB 必须使用。

当前目标只是：

```text
XY spatial occupancy
```

------

# 九、Cluster 不需要找 Raw Points，而是找 Image ROI

这是整个设计的关键。

假设：

```text
Cluster 5:

cell:
(53,60)
(54,60)
(54,61)
```

那么首先得到 Cluster 的 Grid bounding box：

```text
minRow = 53
maxRow = 54
minCol = 60
maxCol = 61
```

由于：

```text
Grid = 0.10m
Image = 0.10m/pixel
```

因此可以直接定位：

```text
Raw Point Image ROI
```

即：

```text
Image
┌──────────────────────────────┐
│                              │
│          ┌──────┐            │
│          │ • •  │            │
│          │  •   │            │
│          └──────┘            │
│                              │
└──────────────────────────────┘
```

然后：

```text
ROI
 ↓
findContours()
```

注意：

> ROI 内存在的是 Raw Point Image 的真实像素点，而不是 Grid Cell 白块。

------

# 十、Cluster ROI 是否需要 padding

请分析是否需要：

```text
Cluster Grid bounding box
        ↓
+ N pixel padding
        ↓
Image ROI
```

重点考虑：

1. Raw Point 落在 Cell 边界；
2. Image pixel quantization；
3. Cluster OBB 是否可能贴近 ROI 边缘；
4. 邻近障碍物是否会进入 ROI。

第一版可以考虑：

```text
padding = 0
padding = 1 pixel
```

但不要增加复杂的动态 padding。

请让 Agent 根据现有坐标定义和真实数据分析后决定。

------

# 十一、findContours 的真正输入

明确：

```text
findContours()
```

的输入必须是：

> **Raw Point Image 在 Cluster ROI 内的二值像素。**

不是：

```text
Grid Cluster Mask
```

不是：

```text
Grid Cell rectangles
```

不是：

```text
Cluster Raw Point vector
```

而是：

```text
Raw Point Image ROI
```

------

# 十二、findContours 第一版保持最简单

当前低矮障碍物经常只有：

```text
3~5 Grid Cells
```

每个 Cell 中 Raw Point 数量也可能很少。

因此第一版不要加入：

```text
threshold
morphology open
morphology close
dilation
erosion
blur
contour smoothing
```

保持：

```text
Raw Point Image
    ↓
Cluster ROI
    ↓
findContours()
```

即可。

原因：

> 任何 morphology 都可能人为扩大、缩小或改变低矮障碍物的真实几何。

------

# 十三、多个 contour 的情况

由于 Cluster ROI 可能包含：

- 当前 Cluster 的点；
- 邻近障碍物的点；
- 零散 stray points；

所以：

```text
findContours()
```

可能得到：

```text
Contour 0
Contour 1
Contour 2
...
```

请重点分析：

> 如何确定哪个 contour 对应当前 Cluster。

优先考虑使用：

```text
Cluster Grid bounding box
```

与：

```text
Contour bounding box / center / overlap
```

进行空间关联。

不要再次遍历原始 PointCloud。

也不要建立：

```text
Cluster → Raw Point
```

映射。

------

# 十四、minAreaRect

得到当前 Cluster 对应 contour 后：

```text
contour
    ↓
cv::minAreaRect()
```

输出：

```text
center
length
width
yaw
4 corners
```

保持当前项目已有定义：

### length

较长轴。

### width

较短轴。

### yaw

最终统一：

```text
[0°, 180°)
```

### corners

保持当前项目已有四角点顺序。

必须检查现有：

```text
ComputeClusterOBB()
```

确保新的 OBB 输出不会破坏下游：

```text
tracker
debug viewer
UDP
```

------

# 十五、必须保留当前 PCA OBB 作为 baseline

第一阶段不要删除：

```text
ComputeClusterOBB()
```

需要同时计算：

```text
Grid PCA OBB
```

和：

```text
Raw Image Contour OBB
```

Debug 输出：

```text
Frame
Cluster ID
Cell count
Raw image pixel count
Contour count

PCA OBB:
    center
    length
    width
    yaw

Raw Image OBB:
    center
    length
    width
    yaw
```

这样才能验证：

> Cell arrangement 改变时，Raw Image OBB 是否比 PCA OBB 稳定。

------

# 十六、重点测试场景

优先测试已经发现问题的 Track，例如：

```text
Track 10008
Track 10010
Track 10011
```

特别关注：

```text
3 cells
4 cells
5 cells
```

排列变化：

```text
横向
纵向
L 型
其他排列
```

比较：

```text
Grid PCA yaw
```

与：

```text
Raw Image Contour yaw
```

是否仍然出现：

```text
-22°
→ 0°
→ -32°
```

这样的跳变。

------

# 十七、不要把 MotionState 和 OBB 重新复杂绑定

当前：

```text
UNKNOWN
MOVING
STATIC
```

已经存在。

本阶段首先验证：

```text
UNKNOWN
    → Raw Image OBB

MOVING
    → Raw Image OBB

STATIC
    → Raw Image OBB
```

即：

> **当前帧有成功检测时，三个状态都优先使用当前帧 Raw Image Geometry。**

不要在第一版重新加入：

```text
history yaw
stable yaw
yaw EMA
history L/W
historical contour fusion
```

如果 Raw Image OBB 已经解决当前帧几何稳定问题，就不需要这些机制。

Track 历史信息仍然主要用于：

```text
Map-frame Track
MotionState
Detection miss prediction
```

而不是当前帧 OBB 几何重建。

------

# 十八、性能要求

当前单帧点云可能：

```text
200,000+
```

UDP 输出当前最多：

```text
20 obstacles
```

未来 Cluster 数量可能更多。

因此必须避免：

```text
O(number_of_clusters × number_of_points)
```

目标是：

```text
Raw Point Image generation
    ≈ O(number_of_points)
```

最好直接复用 BuildGrid 已有 Raw Point traversal。

之后：

```text
Cluster ROI extraction
    ≈ O(total ROI pixels)
```

而不是重新访问全部 PointCloud。

请分析：

```text
BuildGrid()
```

当前是否已经遍历每个 Raw Point。

如果是，优先在同一遍历过程中完成：

```text
Grid processing
+
Raw Point Image projection
```

避免第二次：

```text
for(point : 200k points)
```

------

# 十九、不要修改的模块

本阶段除非绝对必要，不修改：

```text
Ground Filter
Grid resolution
RegionGrowing
AnalyzeVerticalOccupancy
ClusterObstacleGrid
HDMap filtering
Localization
Tracker matching
Tracker ID
Tracker velocity
MotionState 判定
Map-frame anchor
UDP protocol
```

本阶段主要增加：

```text
Raw Point Image generation
Cluster → Image ROI
findContours
minAreaRect
```

以及必要的 OBB 输出替换接口。

------

# 二十、第一阶段先生成分析文档，不要立即大规模修改

请先生成：

```text
RawPointImage_ClusterROI_ContourOBB_Analysis.md
```

内容至少包括：

1. 当前代码数据流；
2. 当前 BuildGrid 如何遍历 Raw Point；
3. Raw Point Image 最合适的插入位置；
4. 是否可以复用 BuildGrid 的 Raw Point traversal；
5. Image 0.10m/pixel 的坐标定义；
6. Grid ↔ Image 坐标映射；
7. Cluster Grid bounding box → Image ROI；
8. ROI padding 是否需要；
9. findContours 输入；
10. 多 contour 如何确定目标 contour；
11. `minAreaRect()` 输出转换；
12. yaw [0°,180°) 转换；
13. corner 顺序兼容；
14. 20w+ points 的性能分析；
15. 多 Cluster 的复杂度分析；
16. 3~5 Cell / 少量 Raw Points 的边界情况；
17. 与当前 PCA OBB 的 A/B debug 方案；
18. fallback 方案；
19. 对 `RefineStaticObbGeometry()` 的最小修改建议；
20. 哪些历史 yaw / geometry refinement 逻辑可以暂时停用。

------

# 二十一、最终目标架构

最终希望得到：

```text
                    Current Frame
                         │
                 Transformed Cloud
                         │
             ┌───────────┴───────────┐
             │                       │
             ▼                       ▼
         BuildGrid()          Raw Point Image
             │                 0.10m/pixel
             │                       │
             ▼                       │
      Ground / Cluster               │
             │                       │
             ▼                       │
      ClusterObstacleGrid            │
             │                       │
             ▼                       │
        Cluster Cells                │
             │                       │
             └──────────┬────────────┘
                        ▼
              Cluster Grid Bounding Box
                        │
                        ▼
                 Image ROI extraction
                        │
                        ▼
                  findContours()
                        │
                        ▼
                 Target Contour
                        │
                        ▼
                 minAreaRect()
                        │
                        ▼
                  Current Frame OBB
                        │
                        ▼
                 Tracker / Debug / UDP
```

核心原则：

> **Raw PointCloud 只在生成 Raw Point Image 时处理一次。**

> **Grid Cluster 不负责重新寻找 Raw Points，只负责告诉 Raw Point Image “目标在哪里”。**

> **findContours() 的对象是 Raw Point Image 中真实投影出来的像素点，而不是 Grid Cell。**

> **当前阶段不使用历史轮廓、不使用历史 yaw、不使用 EMA、不使用复杂 geometry threshold。**

最终先验证一个问题：

> **在保持当前 Grid 检测流程完全不变的情况下，Raw Point Image + Cluster ROI + findContours + minAreaRect 是否能够从根本上解决 3~5 Cell 低矮障碍物因为 Grid 排列变化而导致的 OBB yaw/corner jitter。**