# HDMap 白线加载与可视化实现说明

本文整理两部分：

1. 项目 A 是如何加载并绘制地图白线的
2. 项目 B 采用什么方式“按最小侵入原则”补上等价的地图白线展示

---

## 一、项目 A：地图白线是如何加载并画出来的

### 1.1 入口：最终图像输出链路

项目 A 的地图白线并不是单独的一个“地图模块”，而是嵌在最终检测结果图像生成中：

- 主流程入口：`lidar_capture.cpp`
- 最终图像由 `TrackManager::DrawResult(...)` 生成
- 在这条链路上，地图白线是通过 ROI / HDMap 多边形构造后，再画在图像上

核心思路：

- 先从 HDMap 中定位当前车辆所在 lane / section
- 再生成当前 section 及相邻 section 的道路边界多边形
- 将这些边界转换到车体坐标系
- 最后在 OpenCV 图像里画白线/白色封闭边界

---

### 1.2 地图加载：`read_hdmap`

项目 A 的地图数据来自高精地图二进制文件，核心类型和接口在：

- `roifilter/read_hdmap.h`
- `roifilter/read_hdmap.cpp`

关键点：

- `read_hdmap::load(...)` 负责加载 `hdmap.bin`
- `STR_MAP` 保存地图结构体
- `STR_SECTION` 保存 section 的左右边界：`pstrLeft` / `pstrRight`
- 这些边界点就是后续白线/ROI 多边形的基础

这一步非常关键：地图白线的“来源”不是从某个图片读取，而是从高精地图的 section 边界线中构建出来的。

---

### 1.3 定位当前车辆所在的道路区域

在项目 A 的 `get_roi.cpp` 中，流程大致是：

1. 读取当前融合定位结果 `STR_FUSION_LOCATION`
2. 取出车辆在地图中的坐标：
   - `fXInMap`
   - `fYInMap`
   - `fHeadingInMap`
3. 调用：
   - `GetLaneIdByPoint(...)`
   - `GetSectionIdByLaneId(...)`
4. 由此找到当前车辆所处的 section，以及其 incoming / outgo section

代码中关键逻辑：

- `STR_ID_ARRAY laneIDs = rd_map->GetLaneIdByPoint(car_coordinate_in_map);`
- 之后循环 laneIDs，取出对应 section
- 结果是 `sectionsID`，其包含当前 section 及其相连 section

也就是说：项目 A 不会把整张地图全画出来，而是“只画当前车辆附近的一段可行驶区域”。

---

### 1.4 把地图边界转入车体坐标系

`get_roi.cpp` 中最关键的核心是坐标转换。

先看原理：

- 地图坐标是全局地图坐标系
- 目标是把道路边界点转换成车体坐标系，便于在前视/俯视图中绘制

代码中的旋转变换本质上是：

- 先把地图点减去车当前位置
- 再做绕车体坐标系的旋转

对应逻辑：

```cpp
float offsetx = - pstrFusionLocation->fXInMap;
float offsety = - pstrFusionLocation->fYInMap;
float heading = -(90 - pstrFusionLocation->fHeadingInMap);

double myCos = cos(heading*pi/180.0);
double mySin = sin(heading*pi/180.0);
```

随后每个边界点执行：

```cpp
float current_x = left_x * myCos - left_y * mySin;
float current_y = left_x * mySin + left_y * myCos;
```

这一步相当于：

- 将地图坐标转换到“车体相对坐标”
- 使白线边界在渲染时与车前方向对齐

这也是项目 A 能把路边白线画进最终图像的关键。

---

### 1.5 构造道路多边形：左右边界扩宽

项目 A 并不是直接画原始 section 左右边界，而是会对边界做“扩宽/偏移”，使白线覆盖更宽的道路区域。

关键代码逻辑：

- `delta_left` / `delta_right`
- `g_global_lane_width_extension`
- `g_lane_extension`

示意逻辑：

```cpp
currentStartLeft.x = currentStartLeft_x - delta_left * (currentStartRight_x - currentStartLeft_x) / mod;
currentStartLeft.y = currentStartLeft_y + delta_left * (currentStartLeft_y - currentStartRight_y) / mod;
currentStartRight.x = currentStartRight_x + delta_right * (currentStartRight_x - currentStartLeft_x) / mod;
currentStartRight.y = currentStartRight_y - delta_right * (currentStartLeft_y - currentStartRight_y) / mod;
```

这里的含义是：

- 根据左右边界线方向，向路中心/路外侧偏移一段距离
- 生成一个更实际的道路区域/白线区域

之后：

- 取当前 section 的 left/right 边界点
- 以及 incoming / outgo section 的边界点
- 形成四边形/闭合多边形
- 追加到 `roi_polygons`

也就是说，项目 A 的“白线”不是单纯一条线，而是“一个道路边界区域闭合多边形”。

---

### 1.6 把多边形整理成闭合区域

当 `roi_polygon` 形成四个点后，会调用：

```cpp
correctOrder(p1, p2, p3, p4);
roi_polygon.push_back(p1);
roi_polygon.push_back(p2);
roi_polygon.push_back(p3);
roi_polygon.push_back(p4);
roi_polygons.push_back(roi_polygon);
```

这一步的目标是：

- 让四个点按正确顺序排列
- 避免四边形自交 / 方向错误
- 保证最后能成为合法闭合 polygon

这正是“地图白线/ROI 区域”的最核心构造。

---

### 1.7 最终绘制：OpenCV 画白线

项目 A 在最终图像绘制时，并不是直接用 HDMap 结构对象来画，而是把这些 ROI polygon 转换成图像坐标后绘制。

其最终图像输出入口在：

- `TrackManager::DrawResult(...)`
- 由 `lidar_capture.cpp` 调用

该函数会把：

- 检测结果框
- 目标跟踪框
- 车道/路面/ROI 区域
- 白线轮廓

一起画进最终图像。

本质上，A 的地图白线可以归结为：

- 读 map -> 找 section -> 构造 polygon -> 转到图像像素 -> 画白线

---

## 二、项目 B：如何按最小侵入原则加上地图白线

### 2.1 B 里相对应的模块

项目 B 已经存在一套更清晰、更小范围的模块：

- 定位：`mapfilter/localization_manager.*`
- 坐标转换：`mapfilter/coordinate_transformer.*`
- HDMap：`mapfilter/hdmap_manager.*`
- 过滤：`mapfilter/hdmap_filter.*`
- 可视化：`include/DebugViewer.*`

它们分别负责：

- 定位当前车辆在地图中的 pose
- 把地图点转换到车体坐标
- 把道路 section 组织成 polygon
- 在渲染层叠加这组多边形

这正是 B “最小迁移”的关键：不搬 A 的整套 ROI 流程，而是取其思想和坐标转换原理。

---

### 2.2 B 的完全做法：`HDMapManager::buildDrivablePolygons()`

在项目 B 中，地图白线/路面区域由 `HDMapManager` 负责构造：

- `loadMap(...)`
- `getCurrentSectionIndex(...)`
- `buildDrivablePolygons(...)`

核心代码思路：

1. 先通过 `GetLaneIdByPoint(...)` 定位当前车在 map 中的 lane/section
2. 再把当前 section 和相关 income/outgo section 归并起来
3. 对每个 section 的左边界和右边界，按顺序拼出闭合 polygon

这和 A 的思路是同源的，只是 B 更抽象，也更轻量：

```cpp
bool HDMapManager::buildDrivablePolygons(
    double map_x, double map_y,
    std::vector<std::vector<STR_POINT2F>>& polygons_out,
    int* section_count) const;
```

它输出的是：

- `std::vector<std::vector<STR_POINT2F>>`
- 每个内层 vector 是一个道路 polygon
- 这些 polygon 可用于 debug 渲染

---

### 2.3 B 的坐标转换：`CoordinateTransformer`

B 里完整保留了项目 A 的坐标系思路：

- `vehicleToMap(...)`
- `mapToVehicle(...)`

对应文件：

- `mapfilter/coordinate_transformer.h`
- `mapfilter/coordinate_transformer.cpp`

关键公式：

```cpp
const double dx = map_x - pose.x;
const double dy = map_y - pose.y;
const double theta = -(90.0 - pose.heading_deg) * kDeg2Rad;

veh_x = c * dx - s * dy;
veh_y = s * dx + c * dy;
```

这与 A 的 `PointTransform` 思路和数学关系一致，说明 B 的设计在语义上和 A 是对齐的。

---

### 2.4 B 的定位接入：`LocalizationManager`

B 的定位逻辑做的是：

- 通过传感器/融合定位回调不断更新 pose
- 仅在 `valid == true` 时才参与地图白线渲染

对应：

- `LocalizationManager::Pose`
- `LocalizationManager::instance().getPose(...)`

这让 B 不需要在算法主链路里强耦合地图逻辑：

- 地图 polygon 只在需要时由 `DebugViewer` 去调用
- 过滤逻辑仍然保持本身的 HDMapFilter 规则
- 地图绘制只是观察层行为

---

### 2.5 B 的实际可视化入口：`DebugViewer::DrawHdMapOverlay()`

在项目 B 中，真正加地图白线的地方，是：

- `include/DebugViewer.h`
- `include/DebugViewer.cpp`

新增的方法：

```cpp
void DrawHdMapOverlay(cv::Mat& image,
                      const std::vector<std::vector<STR_POINT2F>>& mapPolygons,
                      const LocalizationManager::Pose& pose,
                      const ElevationGridConfig& gridCfg);
```

其工作流程：

1. 检查 `pose.valid` 是否有效
2. 检查 `mapPolygons` 是否为空
3. 对每个 polygon 中的每个点：
   - 调用 `CoordinateTransformer::mapToVehicle(...)`
   - 转成车辆系坐标
   - 再调用 `WorldToPixel(...)` 生成图像坐标
4. 用 `cv::polylines(..., white, 2)` 画出白色边界

也就是说：

- B 的白线并不是从 A 复制过来的渲染代码
- 它是基于 B 自己的坐标系 + B 自己的像素投影实现

---

### 2.6 B 在调用链中的接入位置

最后在 `SutengDriver::ProcessPcapCloud()` 中调用：

- `m_hdmapManager.buildDrivablePolygons(...)`
- `LocalizationManager::instance().getPose(...)`
- `m_debugViewer->DrawOverlay(..., mapPolygons, loc_pose)`

这表示：

- 只在调试可视化层触发地图白线画出
- 真实检测逻辑和跟踪逻辑完全不受影响
- 代码修改最小，且符合工程架构

---

## 三、总结：A 与 B 的关系

### 项目 A 的本质
项目 A 的地图白线，是通过：

- 读地图库
- 找 section / lane
- 形成道路 polygon
- 转换到车体坐标
- 最终画到二维图像中

### 项目 B 的迁移方式
项目 B 保留了这一设计思想，但不搬 A 的整套 legacy 代码，而是采用：

- `HDMapManager` 负责构造道路 polygon
- `CoordinateTransformer` 负责坐标换算
- `LocalizationManager` 负责 pose
- `DebugViewer` 负责最终白线渲染

这就是“最小迁移”的核心：

- 画法保留
- 逻辑保持在渲染层
- 不碰算法核心

---

## 四、最终一句话

项目 A 的地图白线，本质上是“从 HDMap 的 section 边界构造道路 polygon，再投影到图像中绘制白线”；项目 B 的实现方式，是在自己的 `HDMapManager + CoordinateTransformer + DebugViewer` 体系中，按同样的思想做可视化层叠加，且不引入 A 的非必要依赖。

---

## 五、航向基准对齐：HDMap 白线叠在墙上的根因与修复（2026-08-26）

### 5.1 现象
- `TrackerOverlay` 中 HDMap 道路白线错误叠在道路两旁的墙壁上；白线“应往屏幕上方倾斜”却“向下倾斜”。
- 用临时 `DrawHdMapOverlayCalib` 校准：1m×1m 参考框 = 100px、`actual_px/m ≈ 100` → 像素/米比例精确正确，**排除缩放/原点假说**，问题必在“地图 ↔ 点云”坐标变换侧。

### 5.2 根因：补偿角 = 90° + fLidar2Vehicle_Heading（可推导，非经验值）
点云侧坐标变换（`SuTengDriver::ProcessPcapCloud`）：

```
R_Combined = R_X_Exchange_Y * R_ToCar * R_EMXToMainLidar0
```
- `R_ToCar` 由 lidar.cfg `fLidar2Vehicle_Heading`（当前 -87.888°）构造：`[[cosθ, sinθ],[-sinθ, cosθ]]`；
- `R_EMXToMainLidar`（EMX 模式，`test_EMX=true`）≈ Y 镜像 `[[1,0],[0,-1]]`；`R_X_Exchange_Y` 交换 X/Y。

令 θ = fLidar2Vehicle_Heading，合并化简可得 **R_Combined ≈ R(90° + θ)**（标准逆时针旋转角）：
- θ = -90° → 净旋转 0（点云正好前向，无需补偿）；
- θ = -87.888° → 净旋转 **2.112°**（实测验证 +2.12 正确，-2.12 错误；`90 + (-87.888) = 2.112`）。

**因此**：Grid/点云前向 与 融合 IMU 前向 相差 `90° + fLidar2Vehicle_Heading`。
- `CoordinateTransformer::mapToVehicle / vehicleToMap` 直接使用 `pose.heading_deg`（IMU 航向），少了这个补偿角 → HDMap 多边形绕车旋转 ~2.112°，白线倾斜方向反、贴墙。
- 补偿：`heading_used = pose.heading_deg + (90° + fLidar2Vehicle_Heading)`。

### 5.3 为什么之前没加
- `CoordinateTransformer` 从项目 A 移植，隐含假设“点云变换后就在融合 IMU 前向坐标系里”（即 R_Combined 净旋转 = 0）。
- 本配置下该假设仅在 `fLidar2Vehicle_Heading = -90°` 时成立；当前为 -87.888°，残留 2.112° 系统偏差，之前从未校验，直到 HDMap 白线叠加到墙上才暴露。
- 该角随 lidar.cfg 变化（不同车型/雷达不同），**必须由 lidar.cfg 计算，不能写死**（否则换雷达会忘记改）。

### 5.4 修复（由 lidar.cfg 自动计算，统一应用）
1. **计算**：`main.cpp::GetAllTransformConfigInfo()` 中（`fLidar2Vehicle_Heading` 已在该处读取为 `fAngle_ToCar`）：
   ```cpp
   fLidar2GridHeadingCorrectionDeg = 90.0f + fAngle_ToCar;   // 存入 STR_ALL_LIDAR_CONFIG_INFO
   ```
2. **注入**：`main()` 中调用
   ```cpp
   CoordinateTransformer::setGridHeadingOffsetDeg(strAllLidarTransformInfo.fLidar2GridHeadingCorrectionDeg);
   ```
3. **统一应用**：`CoordinateTransformer::mapToVehicle / vehicleToMap` 内部把该补偿角加到 heading 上
   （`heading_used = pose.heading_deg + offset`，两函数用同一 offset，保持互逆）。
   - 可视化：`DebugViewer::DrawHdMapOverlay / DrawHdMapOverlayCalib`（map → vehicle）自动生效；
   - 过滤：`HDMapFilter::filterClusters`（vehicle → map）自动生效。
   - 两处共用同一补偿，杜绝调用点遗漏/重复（正因如此，调用点不再需要手动加）。

### 5.5 遗留
- 换车型/雷达配置无需改代码：补偿角自动由各自 lidar.cfg 的 `fLidar2Vehicle_Heading` 计算（启动日志会打印 `fLidar2GridHeadingCorrectionDeg` 与 `gridHeadingOffset` 便于核对）。
- 若某天 `R_Combined` 组合方式变化（如非 EMX 主雷达），补偿角公式需按新的组合重新推导（可打印 `R_Combined` 净旋转角核对）。
- 排查看完删除临时 `DrawHdMapOverlayCalib`。
