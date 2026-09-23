# Phase 3-B' Implementation Analysis
## 1cm Raw Point Image + Grid ROI + minAreaRect（纯可视化 A/B 验证）

- 对应 Prompt：本轮对话给出的《Phase 3-B'：1cm Raw Point Image + Grid ROI + minAreaRect 纯可视化验证》
- 前一版分析：`docs/RawPointImage_ClusterROI_ContourOBB_Analysis.md`（0.1m/pixel + findContours 版本，**已被本轮方案取代**，仅保留坐标/性能部分的结论可用）
- 本文档性质：**实施前分析，不改代码**；所有行号/字段均已在当前工作区代码中核对
- 结论口径：`[已验证]` = 读代码确认；`[需实测]` = 需跑数据确认

---

## 一、任务边界（实现层保证，不靠"注意"）

本阶段**只做**：

```text
obstacle_cloud ──► 1cm Raw Point Image ──► Cluster Grid ROI ──► minAreaRect ──► new_corners ──► DebugViewer A/B
```

对 Prompt 第三章 20 条禁止项的**代码级隔离保证**：

| 禁止项 | 实现层保证 |
|--------|-----------|
| 1~5 历史/EMA/Kalman | 本阶段**不新增任何**跨帧状态；`new_corners` 每帧整体重算，无记忆变量 |
| 6~8 tracker/关联/MotionState | `new_corners` **只写在 `outTracks` 副本**（与 Phase 3-B v3 输出隔离同一做法），`m_tracker.vtrackings` 永不含该字段 |
| 9 `detections` | `ConvertClustersToTrackedObstacles()`（`SuTengDriver.cpp:2213`）只读 `cluster.obb_*`，**不读 `img_*`** → 天然隔离 |
| 10 HDMap | `HDMapFilter::filterClusters()` 只读 `cell_indices/in_road` 等 → 天然隔离 |
| 11~13 UDP/现有 OBB | `ConvertTrackToS2ObstacleBox()` 只读 `depth/width/height/pos_*/corners` → `new_corners` 不进 UDP；`corners/depth/width` **一个字节都不改** |
| 14 复杂阈值 | 只有 1 个常量：`img_res = 0.01f`；失败判据只有 `points.size() < 2` |
| 15~16 morphology | 不 include `opencv2/imgproc.hpp` 的形态学函数，不调用 `dilate/erode/blur/morphologyEx` |
| 17 CSR | 图像只有二值（+可选计数器），**不存点 ID / 不存 XYZ** |
| 18 逐 cluster 重扫 20w 点 | 图像在 `ReclassifyPointCloud()` 既有循环内写入；cluster 只扫自己 ROI |
| 19 cell 白块 + findContours | 本阶段**完全不用** `findContours` |
| 20 历史统计 | 无 |

> 特别说明（Prompt 第十八条）：**保持 `RefineStaticObbGeometry()` 启用**（`static_obb_refinement.h:50` 的 `kObbRefineEnable` 不动），因此 `outTracks` 的 `corners/depth/width` 仍是当前系统的输出；`new_corners` 与其**并存且互不影响**。A/B 对比的是「系统输出框 vs 实验框」两个独立数据。

---

## 二、Prompt 第三十条「实施前 8 项检查」逐条结论

### (1) `obstacle_cloud` 的类型、来源、生命周期 `[已验证]`

| 项 | 结论 |
|----|------|
| 类型 | `PointCloud2Intensity::Ptr`，其中 `typedef pcl::PointCloud<pcl::PointXYZI> PointCloud2Intensity;`（`include/Type.h:14`） |
| 来源 | `ElevationMapGroundFilter::ReclassifyPointCloud(cloud, ground_cloud, obstacle_cloud)`（`:1843`，点云循环 `:1855`）；由 `AnalyzeVerticalOccupancy(...)`（`:1418`）在结尾 `:1800` 调用 |
| 调用链 | `ProcessWithObstacleDetection(...)`（`:964`）→ `AnalyzeVerticalOccupancy(*cloud, *ground_cloud, *obstacle_cloud)`（`:1005`）→ `:1800` `ReclassifyPointCloud` |
| 生命周期 | `SuTengDriver::ProcessPcapCloud()` 内每帧 `pObstacleCloud->clear()` 后传入；本帧有效，帧末不再需要（不做跨帧保留） |
| 填充规则（**关键**） | `:1900~1940` 五个分支 push 进 `obstacle_cloud`：① `!WorldToGrid`（ROI 外）② `!cell.valid` ③ `cell.is_ground && z > ground_reference_z + 0.05`（margin = `layer_resolution`）④ `cell.is_obstacle_candidate` ⑤ **其余（高大物体 / 孤立噪点）**；只有 `cell.is_ground && z ≤ ground_ref + 0.05` 才进 `ground_cloud` |

> **重要含义**：`obstacle_cloud` ≈「全部非地面点」，**不是**「低矮障碍物点」。其中包含墙面/行人/杆子等高大物体的点，以及 ROI 外的点。第一版按 Prompt 第八条用它生成图像；风险见第十三节 W4。

### (2) 如何从 `obstacle_cloud` 取每点 `(x,y)` `[已验证]`

- `obstacle_cloud->points[i].x / .y / .z`（`pcl::PointXYZI`），单位 m，坐标系 = `PointCloudTransform()` 之后的**车体/主雷达系**（`SuTengDriver.cpp:995`），与 Grid 完全同一坐标系。
- 第一版**不需要**第二遍遍历 `obstacle_cloud`：直接在第 (1) 条所述 `ReclassifyPointCloud` 既有循环里，对每个将进入 `obstacle_cloud` 的点做一次「世界 → 1cm 像素」写入即可（Prompt 第八条「不要新增完整点云遍历」的最强满足形式）。
- 备选（若坚持"只消费 obstacle_cloud"）：在 `ProcessWithObstacleDetection` 中 `ReclassifyPointCloud` 之后、`ClusterObstacleGrid` 之前，遍历一次 `obstacle_cloud`（只含障碍物点，约为全点云的子集）做栅格化。**推荐前者**（零额外遍历）。

### (3) 当前 Grid ROI 与配置 ROI 的关系 `[已验证]`

```text
配置 ROI（debug_config_EMX.yaml）:
    roi_x_min = 0.0    roi_x_max = 20.0
    roi_y_min = -4.0   roi_y_max = 4.0
    grid_resolution = 0.1
    car_half_x = 0.8   body_filter_x_threshold = 2.5

Grid 实际覆盖（BuildGrid :170）:
    m_effective_roi_x_min = max(roi_x_min, car_half_x + body_filter_x_threshold) = 3.3
    cols = ceil((20.0 - 3.3) * 10) = 167      # x ∈ [3.3, 20.0)
    rows = ceil((4.0 - (-4.0)) * 10) = 80     # y ∈ [-4.0, 4.0)
    cells = 167 × 80 = 13,360
```

⚠️ **配置来源提醒**：用户 Prompt 写的 `roi_x_max = 20 / roi_y ±4` 对应 `config/debug_config_EMX.yaml:97~101`；而 `config/debug_config.yaml:98~101` 是 `8.0 / ±3.0`。运行时真正加载的是 `src/main.cpp:32` 的 `/etc/echiev/low_detection/debug_config.yaml`（或 `-f` 覆盖），该文件当前在工作区不可见。
→ **实现要求：图像尺寸必须由 `ElevationGridConfig` 现算，禁止硬编码 2000×800**。

Raw Point Image 范围（按 Prompt 第七节）用**未扩展原始 ROI**：`x ∈ [roi_x_min, roi_x_max)`，`y ∈ [roi_y_min, roi_y_max)`，**不减去车身盲区**（不使用 `eff_x_min`），因此 x ∈ [0, 3.3) 的图像列永远不会有 cluster ROI 命中（Grid 不覆盖该区域），只浪费内存，无正确性影响。

### (4) `DebugViewer::ComputeGridImageGeometry()` 的实际坐标变换 `[已验证]`

`DebugViewer.cpp:358`（`WorldToPixel` 在 `:379`），参数代入 EMX 配置后：

```text
inv_res       = 10 ;  kGridPixelScale = 10 (DebugViewer.h:297)  → 100 px/m
blind_dist_x  = car_half_x + body_filter_x_threshold = 3.3
blind_cols    = round(3.3/0.1) = 33 → px_blind_offset = 330
rows = 80 ; cols = ceil((20 - 3.3)*10) = 167
img_w = 330 + 167*10 + 100 = 2100 ; img_h = 80*10 + 200 = 1000   (expand_img_w/h = DebugViewer.h:303)

cell (row r, col c) 的中心 → 像素:
    px = 330 + (x - 3.3)*100 + 50 = 380 + 10c      (x = 3.3 + (c+0.5)*0.1)
    py = (80 - (y + 4.0)*10)*10 + 100 = 895 - 10r  (y = -4.0 + (r+0.5)*0.1)
```

**结论（Prompt 第七条要求区分）**：
- 这是**显示坐标**（100 px/m + 盲区偏移 + 上下 margin + y 翻转）；
- 算法 Raw Point Image 是**另一个独立坐标**（1cm/pixel，原点 `roi_x_min/roi_y_min`，row = +y）；
- 两者**不得互相复用换算公式**。新调试函数只需：`WorldToPixel(世界系角点)` 即可绘制，**不需要**把算法图像搬进显示图（若将来要显示算法图像，需按整数块映射，见旧分析文档 §14.3）。

### (5) `DrawMapAndAllOverlay()` 当前 OBB corner 绘制逻辑 `[已验证]`

`DebugViewer.cpp:1554` 起，Track 循环内：

```cpp
std::vector<cv::Point> pts(4);
for (j=0..3) WorldToPixel(t.corners[j].x, t.corners[j].y, px, py, gridCfg), pts[j] = {px,py};
```

- **matched 帧**（`!need_map_rebuild`）：`cv::polylines(..., cv::Scalar(255,255,255), 2)` → **白色 2px**
- **miss 帧**（`need_map_rebuild`）：`cv::polylines(..., cv::Scalar(220,220,0), 5)` → **青色 5px**

⚠️ **与 Prompt 第二十条/第二十一条的冲突（必须指出）**：Prompt 假设「漏检 Track 仍为白色」，但**当前代码是：matched=白、miss=青**。
如果按 Prompt 把 IMG OBB 也画成青色 `(220,220,0)`，则在同一窗口内 miss Track 与 IMG OBB **完全无法区分**。
→ 建议（符合 Prompt「不要为了视觉重叠重新设计颜色语义」的要求）：
1. IMG OBB 画在**新窗口**（新函数），与 `TrackerOverlay` 分离 → 旧窗口颜色约定**一字不改**；
2. 新窗口内：白 = Grid PCA OBB（`cluster.obb_corners`，**不是** `track.corners`，因为后者可能已被 `RefineStaticObbGeometry` 改成 HISTORY yaw 框）、青 = IMG OBB，用**线宽 + 文字标签**区分（IMG OBB 3px + `IMG` 前缀文字）。

### (6) `TrackedObstacle` 定义与 `cluster_id` 关联路径 `[已验证]`

- 定义：`include/track.h:52`；`Point2D corners[4] = {};`（`:58`）；`int cluster_id = -1;`（`:53`）。
- ⚠️ **手写拷贝语义**：`TrackedObstacle` 有**手写拷贝构造**（`track.h:110~143`）与**手写 `operator=`**（`:146~215`）→ **新增字段必须在这两处同步拷贝**，否则 `std::vector<TrackedObstacle> outTracks = m_tracker.vtrackings;` 会丢字段（历史上 `current_exist` 就踩过这个坑，代码里还有注释提醒）。
- `cluster_id` 写回：`track.cpp:423`（V1 matched 分支，已启用）、`:459`（新建 track）、`:694/:768`（V2，未启用）。
- 复用现成查表写法：`RefineStaticObbGeometry()`（`SuTengDriver.cpp:1606`）里 `std::unordered_map<int, const GridCluster*> cluster_by_id`（`:1625` 附近）→ 新函数 `AttachRawImageObbs()` 照抄该结构，零学习成本。
- 其他消费者：`VisFrameBuffer::Publish()`（`VisFrameBuffer.h:24`）会**拷贝** `std::vector<TrackedObstacle>` → 因我们只写 `outTracks`，buffer 里的 `new_corners` 恒为默认值，`VisFrameBuffer` 的 `corners` 逻辑（`:198~205`）完全不受影响。

### (7) OpenCV 3.3 `minAreaRect()` 的实际用法 `[已验证]`

- 头文件：`#include <opencv2/imgproc.hpp>`（`minAreaRect` 在 **imgproc**，不在 core）。
- 库：`/home/zyl/3rdparty/echiev/windows/3rdparty/opencv/opencv_3.3/lib/libopencv_imgproc.so(.3.3)` **存在**；`CMakeLists.txt:120` 已链接 `${OpenCV_LIBS}`（`find_package(OpenCV REQUIRED)` @ `:18`）→ **构建零改动**。`track.h:14` 本来就 `#include "opencv2/opencv.hpp"`。
- 输入：`std::vector<cv::Point>`（整型像素坐标）或 `std::vector<cv::Point2f>`；返回 `cv::RotatedRect`。
- **版本陷阱**：3.3 的 `RotatedRect::angle` 语义与 4.5+ 不同（3.x 约为 `(-90, 0]`），且 `size.width/height` 不保证大小关系。→ **不读 `angle`**，改用 `rect.points(v4)` 取 4 顶点 + 边向量法（第八节）。
- 退化：`points.size()==1` → `size = (0,0)`；共线点 → 一条边约 0。

### (8) 如何保证 `new_corners` 不影响任何现有算法流程 `[已验证]`

隔离矩阵（改写自 Phase 3-B v3 的输出隔离方案）：

| 数据 | 谁写 | 谁读 | `new_corners` 是否参与 |
|------|------|------|----------------------|
| `m_tracker.vtrackings[i]` | `SimpleTracker::update()` | 关联/速度/age/motion/锚点 | **否**（永不写入） |
| `outTracks`（副本） | `RefineStaticObbGeometry()` → 新 `AttachRawImageObbs()` | `DrawMapAndAllOverlay`、`m_visBuffer.Publish`、`ConvertTrackToS2ObstacleBox` | 只写 `new_corners/has_new_corners`，**不动** `corners/depth/width/pos_*` |
| `detections` | `ConvertClustersToTrackedObstacles()` | `m_tracker.update()` | **否** |
| `GridCluster` | 新增 `img_*` 字段（并列于 `obb_*`） | 仅 Debug/日志 | 只加不改，`ComputeClusterOBB()` 一行不动 |
| UDP | `ConvertTrackToS2ObstacleBox()` | 外部 | `new_corners` 不在结构映射内 → **协议不变** |

---

## 三、坐标系统一说明（三套坐标，禁止混用）

```text
① 世界/车体系 (m)            —— 点云、Grid、Cluster、Track 都用它
        │
        ├──► ② Grid (0.1m/cell)      row = +y, col = +x        GridIndex = row*cols + col
        │
        ├──► ③ Raw Point Image (0.01m/px)  row = +y, col = +x
        │        col = floor((x - roi_x_min) / 0.01)
        │        row = floor((y - roi_y_min) / 0.01)
        │        像素 (col,row) 的"世界代表点" = (roi_x_min + (col+0.5)*0.01, roi_y_min + (row+0.5)*0.01)
        │
        └──► ④ DebugViewer 显示 (100px/m + 盲区 3.3m 偏移 + margin + y 翻转)  WorldToPixel()
```

**栅格化与反投影必须成对使用**：写入用 `floor`，反投影用 `+0.5`（cell/pixel 中心），单轴误差 ≤ 0.005m（1cm 分辨率下可忽略；对比 Grid PCA 的 0.05m 量化，是 10 倍改善）。Prompt 第十条「不能把 `row/col` 当作 Image pixel」= ② 与 ③ 是两个独立网格，本设计严格遵守（两者只通过世界坐标换算）。

---

## 四、数据流（最终形态）

```text
ProcessWithObstacleDetection()                                    ElevationMapGroundFilter.cpp:964
    │
    ├─ BuildGrid(*cloud)                                          :145
    │      ├─ 计算 m_grid_rows/m_grid_cols（不变）
    │      └─ 【新增】重置 Raw Point Image（尺寸由 config ROI + 1cm 现算）
    │
    ├─ ComputeGroundHeight/ComputeSlope/RegionGrowing/GenerateGroundMask/ComputeGroundReference
    │
    ├─ AnalyzeVerticalOccupancy(*cloud, *ground_cloud, *obstacle_cloud)   :1418
    │      ├─ Phase2 layer_histogram 循环                            :1462
    │      └─ ReclassifyPointCloud(...)                              :1800 / 定义 :1843
    │            for (point : cloud.points)                          :1855
    │               ├─ 【原有】分类 → obstacle_cloud / ground_cloud
    │               └─ 【新增】若该点进入 obstacle_cloud 且落在 ROI 内 → image[row*W+col] = 255
    │                                                                 (可选 counter[row*W+col]++)
    └─ ClusterObstacleGrid()                                          :1977
           └─ BuildClusterFromCells()                                 :2145
                 ├─ ComputeClusterOBB()   ← 现有 PCA OBB（不动）      :2377
                 └─ 【新增】ComputeRawImageOBB(cluster)  ← 存 cluster.img_*
                                │
                                ▼
                    GridCluster{ obb_*（现有）, img_*（新增） }
                                │
       ┌────────────────────────┼───────────────────────────────┐
       ▼                        ▼                               ▼
  HDMap filter           detections（PCA）                 DebugViewer（新函数）
  （不动）                    │                               白=PCA / 青=IMG
                        m_tracker.update（不动）
                              │
                    UpdateTrackMotionStates（不动）
                              │
                    UpdateMapAnchors（不动）
                              │
                 RefineStaticObbGeometry（保持启用，不动）→ outTracks
                              │
                 【新增】AttachRawImageObbs(clusters, outTracks)   ← 只写 new_corners/has_new_corners
                              │
              DrawMapAndAllOverlay / m_visBuffer.Publish / ConvertTrackToS2ObstacleBox（都不读新字段）
```

---

## 五、Raw Point Image 生成（Prompt 第四、五、八、二十五、二十六条）

### 5.1 存储

```cpp
// ElevationMapGroundFilter.h（私有成员）—— 用 std::vector<uint8_t> 保持本类不依赖 OpenCV
std::vector<uint8_t> m_rawPointImage;   // 0 / 255，尺寸 W*H
std::vector<uint8_t> m_rawPointCounter; // 可选：每像素点计数（饱和 255），用于日志 raw_pts
int m_rawImageW = 0, m_rawImageH = 0;
```

- 尺寸：`W = ceil((roi_x_max - roi_x_min)/0.01)`，`H = ceil((roi_y_max - roi_y_min)/0.01)`。
  EMX 配置 → `2000 × 800`；`0.01m/pixel` 常量 `kRawImageResolution = 0.01f`。
- 内存：`2000*800 = 1.6MB`（二值）+ 1.6MB（计数器，可选）= 3.2MB。`[可接受]`
- 重置时机：`BuildGrid()` 内（与 `m_grid_rows/cols` 同处），用 `assign(W*H, 0)`；同尺寸时 `std::vector::assign` 不重新分配（`[已验证]` 标准行为：`assign` 会复用已有容量）→ 每帧 1.6MB memset ≈ 0.1ms 级。
- 公开只读访问器（供 DebugViewer 将来可选使用）：

```cpp
const std::vector<uint8_t>& GetRawPointImage() const { return m_rawPointImage; }
int GetRawImageWidth()  const { return m_rawImageW; }
int GetRawImageHeight() const { return m_rawImageH; }
float GetRawImageResolution() const { return kRawImageResolution; }
```

> 若用户坚持用 `cv::Mat`：`ElevationMapGroundFilter.h` 需 `#include <opencv2/core.hpp>`，行为等价（`cv::Mat` 可零拷贝包装 `std::vector`），本分析默认用 `std::vector`（改动面更小，且保持类头文件无 OpenCV 依赖）。

### 5.2 写入位置与规则（零新增遍历）

```cpp
// ReclassifyPointCloud() 循环内（:1855 ~ :1938），在"即将 push 进 obstacle_cloud"的位置：
auto rasterize = [&](const pcl::PointXYZI& p) {
    if (m_rawImageW <= 0) return;
    int col = static_cast<int>((p.x - cfg.roi_x_min) * inv_img_res);
    int row = static_cast<int>((p.y - cfg.roi_y_min) * inv_img_res);
    if (col < 0 || col >= m_rawImageW || row < 0 || row >= m_rawImageH) return;   // ROI 外（对应分支①）
    const size_t k = static_cast<size_t>(row) * m_rawImageW + col;
    m_rawPointImage[k] = 255;
    if (m_rawPointCounter[k] < 255) m_rawPointCounter[k]++;   // 可选
};
```

- 只对「落在 ROI 内」的点写像素；分支①（`!WorldToGrid`）中**落在 ROI 内但 `!cell.valid`** 的点仍会写（用 `isfinite` + ROI 判定，不依赖 `WorldToGrid`）。
- `isfinite` 检查沿用现有循环（`:1857`）。
- 性能：20w 点 → 20w 次「2 次乘加 + 1 次越界判断 + 最多 2 次字节写」。`[可忽略]`

### 5.3 图像内容语义（必须写进代码注释）

图像 = **障碍物点（`obstacle_cloud` 同源）在 1cm 网格上的占据**，**不是**全部原始点（Prompt 第八条理由：全点会把 ROI 用地面的点填满）。

---

## 六、Cluster → Image ROI（Prompt 第十、十一、十二条）

### 6.1 ROI 推导

用 cluster 已有的世界系 AABB（`BuildClusterFromCells` 中已含 ±0.05 半格，`ElevationMapGroundFilter.cpp:2165~2172`）：

```cpp
const int c0 = static_cast<int>(std::floor((cluster.min_x - roi_x_min) * inv_img_res));
const int c1 = static_cast<int>(std::floor((cluster.max_x - roi_x_min) * inv_img_res));
const int r0 = static_cast<int>(std::floor((cluster.min_y - roi_y_min) * inv_img_res));
const int r1 = static_cast<int>(std::floor((cluster.max_y - roi_y_min) * inv_img_res));
// 夹到图像边界，包含式扫描 [c0..c1] × [r0..r1]
```

- **padding = 0**（Prompt 第十一条）：不做任何额外空间扩展。
- 上式用 `floor` 对 min/max 两侧取值，数学上等价于「0 padding + 边界像素包含」，额外包含量 ≤ 1 pixel = **1cm**（相对于 10cm cell 可忽略）。
- 一个 10cm cell → 11×11 px 的 ROI；3~5 cell cluster 的 ROI 典型为 **12×12 ~ 23×23 px（≈150~530 px）**。

### 6.2 ROI 内取点（Prompt 第十二条：不做 findContours）

```cpp
std::vector<cv::Point> pts;
pts.reserve((c1-c0+1) * (r1-r0+1));
int pixel_count = 0, raw_pt_count = 0;
for (int r = r0; r <= r1; ++r)
  for (int c = c0; c <= c1; ++c) {
      const size_t k = static_cast<size_t>(r) * W + c;
      if (m_rawPointImage[k]) { pts.emplace_back(c, r); ++pixel_count; }
      raw_pt_count += m_rawPointCounter[k];      // 可选：点数统计
  }
```

- `cv::Point(c, r)` 直接就是 prompt 第十二条要求的写法（`points.emplace_back(c, r)`）。
- 复杂度 `O(ROI 面积)`（Prompt 第二十五条允许）。20 clusters × ~400 px ≈ 8k 次读取，可忽略。
- **不做**：`findContours`、连通域选择、union/largest、morphology（Prompt 第十二、十三、十五条）。

### 6.3 每 cluster 扫描 ROI 与「Prompt 第十八条禁止项」的关系

禁止的是「逐 cluster 重扫 20w 点云」；此处只扫 1cm 图像的小 ROI，**不触发点云访问**，符合 Prompt 第二十五条明确允许的范围。

---

## 七、`minAreaRect` → 项目 OBB 语义（Prompt 第十四条）

### 7.1 像素 → 世界

```cpp
// rect 顶点为 ROI 图像像素坐标 (col, row)
wx = roi_x_min + (vx + 0.5f) * kRawImageRes;    // vx = rr.points()[i].x (col)
wy = roi_y_min + (vy + 0.5f) * kRawImageRes;    // vy = rr.points()[i].y (row)
```

### 7.2 4 顶点 → 项目 OBB（**不读 `RotatedRect::angle`**）

```cpp
cv::RotatedRect rr = cv::minAreaRect(pts);            // pts: std::vector<cv::Point>
cv::Point2f p[4]; rr.points(p);                       // 顺序依赖 OpenCV 版本 → 只用几何关系
Eigen::Vector2f P[4];                                 // 转世界坐标（上式）

// ① 两条相邻边，取"较长边"为长轴方向 u
Eigen::Vector2f e0 = P[1] - P[0], e1 = P[2] - P[1];
Eigen::Vector2f u = (e0.norm() >= e1.norm()) ? e0.normalized() : e1.normalized();
float L = std::max(e0.norm(), e1.norm());
float Wd = std::min(e0.norm(), e1.norm());

// ② 统一到 yaw ∈ [0, π)（无向轴；Prompt 第十四条）
if (u.y() < 0.0f || (u.y() == 0.0f && u.x() < 0.0f)) u = -u;
const float yaw = std::atan2(u.y(), u.x());          // ∈ [0, π)

// ③ 右手正交副轴：v = rot90(u)（逆时针 90°）
const Eigen::Vector2f v(-u.y(), u.x());

// ④ 中心 = 4 顶点均值
Eigen::Vector2f c = (P[0] + P[1] + P[2] + P[3]) * 0.25f;

// ⑤ corners（与 ComputeClusterOBB :2549~2552 完全同序：左下→右下→右上→左上，C0→C1 = 长轴）
C0 = c - (L/2)u - (Wd/2)v;   // 左下
C1 = c + (L/2)u - (Wd/2)v;   // 右下
C2 = c + (L/2)u + (Wd/2)v;   // 右上
C3 = c - (L/2)u + (Wd/2)v;   // 左上
```

**不变量（下游依赖，必须保证）**：

| 不变量 | 依赖方 |
|--------|--------|
| `length >= width`（`L >= Wd`） | 项目 OBB 语义（`ComputeClusterOBB` Step 6） |
| `corners[0] → corners[1]` = 长轴 | `DrawMapAndAllOverlay` miss 重建反算 yaw（`DebugViewer.cpp:1620` 附近）；`RefineStaticObbGeometry` |
| `corners[0] → corners[3]` = 短轴 | 同上（miss 重建反算 width） |
| 顺序自洽（逆时针） | 可视化 polylines、UDP 的 `corners[4]` |
| yaw ∈ `[0, π)` | 本阶段新增字段约定；与 PCA `obb_angle ∈ [-π/2, π/2)` 比较时**必须 mod π**（可用 `static_obb_refinement.h` 的 `AngularDistance180`） |

### 7.3 `GridCluster` 新增字段（只增不改）

```cpp
// ---- Phase 3-B': 1cm Raw Point Image OBB（实验字段，仅 Debug/日志；PCA 字段不受影响）----
bool    has_img_obb   = false;       // 实验 OBB 是否有效
Point2D img_corners[4] = {};         // 新的 4 角点（左下→右下→右上→左上）
float   img_center_x = 0.0f, img_center_y = 0.0f;
float   img_length = 0.0f, img_width = 0.0f;
float   img_yaw_rad = 0.0f;          // [0, π)
int     img_pixel_count = 0;         // ROI 内被占据像素数
int     img_raw_pt_count = 0;        // ROI 内障碍物点数（需计数器图像）
int     img_roi_min_row = 0, img_roi_max_row = 0;
int     img_roi_min_col = 0, img_roi_max_col = 0;
```

### 7.4 计算时机

在 `ClusterObstacleGrid()`（`:1977`）内 `BuildClusterFromCells()` 返回后立即调用（`:2049` 附近），使 cluster 离开 GroundFilter 时两套 OBB 都已就绪。

---

## 八、`TrackedObstacle` 实验字段（Prompt 第十五、十六、十七条）

### 8.1 字段（最小集）

```cpp
// ---- Phase 3-B' 实验字段：1cm Raw Point Image minAreaRect OBB（仅 Debug 可视化用）----
Point2D new_corners[4] = {};
bool    has_new_corners = false;
```

- 可选（Prompt 说"非必须"）：`new_center/new_length/new_width/new_yaw`。**建议第一版不加**：`new_corners` 已足够让 DebugViewer 画框，且少一套需要维护一致性的冗余数据。
- ⚠️ **必须同步修改 `track.h:110~143` 拷贝构造 与 `:146~215` 拷贝赋值**，各加 2 行，否则 `outTracks`/`VisFrameBuffer` 副本丢字段（`current_exist` 的历史教训见 `track.h:122` 注释）。

### 8.2 写入路径（只写 `outTracks`）

`SuTengDriver.cpp:1095` 之后新增：

```cpp
std::vector<TrackedObstacle> outTracks;
RefineStaticObbGeometry(outputClusters, loc_pose, have_pose, outTracks);   // 不动
AttachRawImageObbs(outputClusters, outTracks);                             // 新增（只写 new_corners）

void SutengDriver::AttachRawImageObbs(const std::vector<GridCluster>& clusters,
                                      std::vector<TrackedObstacle>& outTracks) const
{
    // 与 RefineStaticObbGeometry 同款查表
    std::unordered_map<int, const GridCluster*> by_id;
    for (const auto& cl : clusters) by_id[cl.id] = &cl;

    for (auto& t : outTracks) {
        t.has_new_corners = false;
        if (t.lastSeen != 0) continue;                 // 只处理本帧匹配到的 Track
        auto it = by_id.find(t.cluster_id);
        if (it == by_id.end() || !it->second->has_img_obb) continue;
        for (int j = 0; j < 4; ++j) t.new_corners[j] = it->second->img_corners[j];
        t.has_new_corners = true;
    }
}
```

- 调用位置必须在 `UpdateMapAnchors()`（`:1088`）之后、`DrawMapAndAllOverlay`（`:1139`）之前 → 锚点仍基于 RAW 几何（与既有约束一致）。
- **不写 `m_tracker.vtrackings`**、**不写 `detections`**。

---

## 九、DebugViewer 新函数（Prompt 第十九、二十、二十一、二十二条）

### 9.1 签名与配置

```cpp
// DebugViewer.h（public）
void DrawRawPointImageObbDebug(const std::vector<TrackedObstacle>& tracks,
                               const std::vector<GridCluster>&    clusters,
                               const ElevationGridConfig&         gridCfg,
                               const std::vector<std::vector<STR_POINT2F>>& mapPolygons,
                               const LocalizationManager::Pose&   pose);

// ReadYamlFile.h: SELF_DEBUG_CONFIG 内新增一行
DEBUG_VIEWER_CONFIG RawImageObb;   // 第十一层: 1cm Raw Point Image OBB A/B 对比图
// ReadYamlFile.cpp: 一行
config.RawImageObb = readDebugCfg(DebugViewerNode, "RawImageObb");
```

- `readDebugCfg`（`ReadYamlFile.cpp:242`）在 YAML 缺节点时返回**默认 `enable=false`** → 旧配置向后兼容，不会因为没加 YAML 而误开窗口。需要观察时在 yaml 的 `DebugViewer:` 下加：

```yaml
  RawImageObb:
      enable: 1
      show: 1
      save: 1
```

### 9.2 绘制内容（复用 `DrawMapAndAllOverlay` 的坐标与骨架）

| 元素 | 来源 | 颜色 |
|------|------|------|
| 背景网格 + 坐标轴 + ROI 边框 | `AddBackGround` + 现有 rectangle | 灰 |
| **当前 Grid PCA OBB** | `cluster.obb_corners`（`!has_obb` → cluster AABB） | **白色 `(255,255,255)` 2px** |
| **Raw Point Image OBB** | `track.new_corners`（fallback：`cluster.img_corners`） | **青色 `(220,220,0)` 3px** |
| 中心点 | cluster PCA 中心（黄）/ IMG 中心（青） | — |
| 文字标签 | 见下 | 白/青 |
| HDMap 白线 | `DrawHdMapOverlay`（可选，沿用现有调用习惯） | 白 |

标签（Prompt 第二十二条）：

```text
T{track_id} C{cluster_id} cells=4 RawPts=37 Px=31
PCA y=90.0 L=0.30 W=0.10
IMG y=87.4 L=0.27 W=0.08
```

- 颜色冲突处理：**新窗口 + 线宽 + 文字**，不修改 `DrawMapAndAllOverlay` 里 miss=青的既有语义（见第二节 (5) 的冲突说明）。

### 9.3 调用点

`SuTengDriver.cpp:1139`（`DrawMapAndAllOverlay`）之后紧邻调用，共用同一 `outTracks / outputClusters / hdmap_polygons / loc_pose`：

```cpp
m_debugViewer->DrawRawPointImageObbDebug(outTracks, outputClusters,
                                         m_pElevationMapGroundFilter->GetConfig(),
                                         hdmap_polygons, loc_pose);
```

窗口名：`"RawImageObb"`（`ShowOrSave` 会按 `m_frameCount` 存 PNG 到 `debug_save_dir`）。

---

## 十、日志（Prompt 第二十三条）

每 cluster / 每 Track 一行：

```text
[RawImgOBB] frame=272 track=10010 cluster=3 cells=4 raw_pts=42 px=38 roi=[c 380..390][r 425..435]
            PCA: c=(9.45,1.60) L=0.30 W=0.10 yaw=90.0
            IMG: c=(9.47,1.59) L=0.27 W=0.08 yaw=87.4 has=1
```

- `yaw` 打印为 **deg ∈ [0,180)**；A/B 差值用 `AngularDistance180`（mod π）后打印 `dYaw`。
- `frame` 用现有全局帧计数（`test_Frame_count`，`SuTengDriver.cpp` 已在用）。
- `raw_pts` 依赖可选计数器图像；若不加计数器，则用 `cluster.point_num`（该 cluster 各 cell 全部点数）替代并**明确标注 `pts_cell=...`**，避免误读为障碍物点数。

---

## 十一、失败条件（Prompt 第二十四条，只保留最小判据）

```text
if (pts.size() < 2)                     → has_img_obb = false;（不写 new_corners）
else                                     → minAreaRect → has_img_obb = true
// 附加：共线/单点退化时 minAreaRect 会给出 size 一侧为 0 —— 原样输出，
//      但日志打印 L/W，便于识别（不引入新阈值、不做兜底放大）
```

- `has_img_obb=false` 时 `AttachRawImageObbs` 跳过 → `has_new_corners=false` → DebugViewer 只画 PCA 白框。
- 现有 PCA OBB / tracker / UDP **完全不受影响**（Prompt 第二十四条要求）。

---

## 十二、性能与内存（Prompt 第四、五、二十五条）

| 项 | 计算 | 结论 |
|----|------|------|
| 图像尺寸 | `2000×800`（EMX） | 1.6MB；+计数器 1.6MB |
| 每帧清零 | `assign(1.6e6, 0)` | ≈0.1ms 级 |
| 栅格化 | 障碍物点数 × O(1) | 与既有循环合并 → 0 额外遍历 |
| ROI 扫描 | `Σ ROI 面积`，3~5 cell → ~150~530 px/cluster | 20 clusters ≈ 8k px |
| minAreaRect | `O(k log k)`，`k ≤ 530` | 微秒级 |
| 合计 | `O(N + ΣROI)`，无 `O(clusters × N)` | 相比当前帧 ~100ms 预算可忽略 |

---

## 十三、风险清单（本阶段必须靠日志回答）

| ID | 风险 | 说明 | 观测手段 |
|----|------|------|---------|
| **W1** | 配置与实际不符 | Prompt 的 20/±4 = `debug_config_EMX.yaml`；`debug_config.yaml` 是 8/±3；运行时加载 `/etc/echiev/...` 或 `-f` | 启动时日志打印 `img W×H` 与 `roi_*` |
| **W2** | ROI 内混入非目标点 | `obstacle_cloud` 含高大物体点/邻物点；0-padding 的 ROI 就是 cluster 的 cell AABB（10cm 粒度），**同一 cell 内的邻物点在 1cm 分辨率下同样被收进 ROI** | 每 cluster 的 `px / raw_pts` + Debug 图上目视 |
| **W3** | 点数太少 → 频繁 fallback | 稀疏点在 1cm 图像上可能只有 2~5 px；`px<2` 直接判失败 | `has=0` 比例统计 |
| **W4** | 共线退化 | 路沿石顶面一排点 → `W≈0.01m` | 日志 L/W；**不做补偿**（保持真实语义） |
| **W5** | IMG 的 `W` 系统性小于 PCA 的 `W` | PCA 基于 cell 中心 → `W ≥ 0.1m`；IMG 只覆盖可见表面点 → 可低至 0.01~0.05m | A/B 的 `dW` 分布（这是**预期现象**，不是 bug） |
| **W6** | 仅覆盖可见表面 | 点云只打到物体顶面，IMG OBB ≈ 顶面外接矩形，不等于 footprint | 记录即可 |
| **W7** | 拷贝语义遗漏 | `track.h` 手写拷贝构造/赋值漏加字段 → `outTracks` 丢 `new_corners`（现象：Debug 全是白框无青框） | 自检：首次运行应能看到青框 |
| **W8** | 颜色混淆 | 既有 miss Track 已是青色 `(220,220,0)` | 已用「新窗口 + 线宽 + 文字」规避 |
| **W9** | 1cm 量化 vs Grid 量化 | 1cm 误差 0.5cm，Grid 5cm → 不是新瓶颈 | — |
| **W10** | 坐标系用错 | ②Grid/③Image/④显示 三套，四则混淆会导致框整体偏移 | 先跑 1 帧目视对齐（青框应紧贴点的分布） |

---

## 十四、改动清单（文件 / 函数 / 行号锚点）

| # | 文件 | 位置 | 改动 | 风险 |
|---|------|------|------|------|
| 1 | `include/ElevationMapGroundFilter.h` | 私有区（`GridCell/GridCluster` 附近） | 新增 `img_*` 字段（`GridCluster`）；新增 `m_rawPointImage/m_rawPointCounter/m_rawImageW/H`、公开 getter、私有 `ResetRawPointImage()` / `ComputeRawImageOBB(GridCluster&)` | 低（纯新增） |
| 2 | `include/ElevationMapGroundFilter.cpp` | `BuildGrid()` `:170` 附近 | 计算并重置 1cm 图像（尺寸由 config 现算） | 低 |
| 3 | 同上 | `ReclassifyPointCloud()` `:1855` 循环 | 每个进入 `obstacle_cloud` 的点写 1 个像素（+计数器） | 低（0 额外遍历） |
| 4 | 同上 | `ClusterObstacleGrid()` `:2049` 附近 | `BuildClusterFromCells()` 后调用 `ComputeRawImageOBB()` | 低 |
| 5 | 同上 | 新函数（文件末尾或 `ComputeClusterOBB` 之后） | ROI + `minAreaRect` + 4 顶点→项目 OBB 语义 | 中（语义正确性是核心） |
| 6 | `include/track.h` | `:52~58` + `:110~143` + `:146~215` | 新增 `new_corners[4] / has_new_corners`，**并同步两处手写拷贝** | **中（漏改则静默失效，见 W7）** |
| 7 | `include/SuTengDriver.cpp` / `.h` | `:1095` 之后 + 新函数 | 新增 `AttachRawImageObbs()`；调用 `DrawRawPointImageObbDebug()`；新增 `[RawImgOBB]` 日志 | 低 |
| 8 | `include/DebugViewer.h/.cpp` | 头文件 public 区 + cpp 末尾（`DrawMapAndAllOverlay` 之后） | 新增 `DrawRawPointImageObbDebug()` | 低 |
| 9 | `common/ReadYamlFile.h/.cpp` | `:222` 附近 / `:267` 之后 | 新增 `DEBUG_VIEWER_CONFIG RawImageObb` + 解析一行 | 低（缺节点 → enable=false） |
| 10 | `config/debug_config_EMX.yaml` | `DebugViewer:` 段 | 新增 `RawImageObb: {enable/show/save}` | 低 |
| 11 | 构建 | — | **无需改动**（`libopencv_imgproc` 已链接，`AUX_SOURCE_DIRECTORY(./include SRC)` 自动收集 `.cpp`） | — |

> 实现位置建议：`ComputeRawImageOBB()` 的 `minAreaRect` + 4 顶点归一化部分，可放在 `ElevationMapGroundFilter.cpp` 内（直接 include `<opencv2/imgproc.hpp>`）。若希望保持"GroundFilter 头文件不依赖 OpenCV"，则把该函数实现放 cpp 即可（本方案已是如此，头文件只出现 `std::vector<uint8_t>`）。

---

## 十五、实施顺序与验证

1. **第 1 步（无 UI，先出数据）**：改动 1~5 + 10 → 编译 → 跑 pcap → 看 `[RawImgOBB]` 日志，确认：
   - `img W×H` 与实际 config 一致；
   - `px / raw_pts` 分布（回答 Prompt 第九条「obstacle_cloud 点是否够用」）；
   - `has=0` 比例。
2. **第 2 步（出图 A/B）**：改动 6~9 → 观察 `RawImageObb` 窗口，重点看 Prompt 第二十八条 4 个 Case（3 cell 横向/竖向、4~5 cell、同 Track 多帧）。
3. **回滚方式**：所有改动集中在 9 个文件、且均为「新增」；`RawImageObb.enable=0` 即可让 UI/日志静默，代码级回滚只需删新增块。
4. **验收（Prompt 第二十八条）**：不看"编译通过"，看 `dYaw/dL/dW/dCorner` 逐帧分布 + 目视框是否贴合真实点分布。

---

## 十六、需要用户确认的 3 个点（不影响先做第 1 步）

1. **点数计数器（`m_rawPointCounter`）是否需要**：Prompt 第二十二/二十三条要求日志输出 `RawPts`，二值图像无法提供该值 → 建议加（+1.6MB，1 次饱和自增/点）；若不要，则日志用 `cluster.point_num`（cell 全点数）并在字段名上标注区别。
2. **新窗口颜色细节**：IMG OBB 青 `(220,220,0)` 3px 已有约定；是否同时叠加 `track.corners`（可能是 HISTORY yaw 框）作第三条曲线？默认**不叠加**（避免三色混淆），保持 Prompt 的"白/青"二色。
3. **是否需要把算法图像本身画出来**（1cm 图像放大 10 倍贴到显示坐标）：默认**不做**（第一版只画框 + 文字统计），如需再单独加一层整数块映射（旧分析文档 §14.3 已给出精确公式）。

---

## 附：与旧分析文档的差异记录

| 项 | 旧版（`RawPointImage_ClusterROI_ContourOBB_Analysis.md`） | 本版（本轮 Prompt） |
|----|---------------------------------------------------|-------------------|
| 图像分辨率 | 0.1m/pixel（与 Grid 同格，像素与 `GridIndex` 同索引） | **0.01m/pixel**（独立网格，仅通过世界坐标换算，`W×H = 2000×800`） |
| 图像内容 | 障碍物点（同） | 障碍物点（同，`obstacle_cloud`） |
| 轮廓提取 | `findContours` + 轮廓关联 + union/largest | **不用 `findContours`**，ROI 内全部非零像素直接 `minAreaRect` |
| ROI padding | 1 pixel | **0** |
| 输出 | 替换 `outTracks` 几何 | **完全不替换**，只在 `TrackedObstacle::new_corners` 上做 Debug 实验 |
| `RefineStaticObbGeometry` | 建议停用 | **保持启用**（要求 A/B 的"系统框"仍为现状） |
| 可视化 | 加在既有 overlay + 新 Raw Point Image 窗口 | **新增独立窗口** `RawImageObb`，白=PCA / 青=IMG |
