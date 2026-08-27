# Agent 任务：正式修改项目 B 低矮障碍物检测项目，接入 Localization + HDMap，并实现 Cluster 级软约束

## 一、任务目标

现在开始正式修改我的“低矮障碍物检测项目 B”。

最终目标是：

> 将项目 A 中已经验证过的 HDMap 能力，以及项目 C 中已经确认的车辆定位获取方式，迁移到项目 B。

最终第一阶段只实现：

```text
车辆定位
+
HDMap
+
Coordinate Transform
+
Cluster 级 HDMap 软约束
```

**暂时不要实现 Grid 级过滤和 PointCloud 级过滤。**

---

# 二、必须首先阅读的设计文档

在修改任何代码之前，必须先阅读以下两份文档：

```text
/home/zyl/echiev_perception/perception_v5.1/迁移地图和定位docs/HDMap_Location_Integration_To_LowObstacle_Project.md
/home/zyl/echiev_perception/perception_v5.1/迁移地图和定位docs/Project_C_Localization_Queue_Analysis.md
```

其中：

### 第一份文档

```text
HDMap_Location_Integration_To_LowObstacle_Project.md
```

来自项目 A 的 HDMap 分析。

重点参考：

* HDMap 加载方式
* `read_hdmap.h/.cpp`
* `common/convert.cpp`
* 坐标系
* `vehicle ↔ map` 转换公式
* HDMap 查询方式
* Cluster 级过滤方案
* 实施步骤
* 推荐架构
* 后续代码修改清单

特别参考：

```text
31. 实施步骤
33. 最终推荐架构
34. 后续代码修改清单
```

---

### 第二份文档

```text
Project_C_Localization_Queue_Analysis.md
```

重点参考：

* 项目 C 的定位数据来源
* 公司内部队列
* `MC 0x0a @ 9110`
* `STR_FUSIONLOC`
* 定位生产/消费链路
* 项目 A / C 定位源对照
* 项目 B 推荐定位接入方式

特别参考：

```text
24. HDMap + Localization + LowObstacleDetection 推荐架构
```

其中已经确定：

```text
Project B
├── LocalizationManager
│      └── 订阅 MC 0x0a @ 9110
│
├── HDMapManager
│      └── 移植项目 A read_hdmap
│
├── CoordinateTransformer
│      └── 使用 A/C 等价的 vehicle ↔ map 公式
│
└── HDMapFilter
       └── Cluster 级软约束
```

---

# 三、重要原则：先理解当前项目 B，再修改

虽然两份 MD 已经给出了设计，但：

> **不能直接机械复制代码。**

必须先重新阅读当前项目 B 的实际代码，确认：

```text
main.cpp
SuTengDriver.h/.cpp
Type.h
ReadYamlFile.h/.cpp
ElevationMapGroundFilter.h/.cpp
CMakeLists.txt
```

以及当前：

```text
ProcessPcapCloud
```

的完整调用链。

尤其要确认：

```text
点云
 ↓
Ground Filter
 ↓
Grid
 ↓
Cluster
 ↓
TrackedObstacle
 ↓
Tracker
 ↓
UDP
```

在当前代码中的真实函数名和数据结构。

**不要因为 MD 中写的是某个函数名，就假设当前源码一定完全相同。**

---


# 五、最终目标架构

最终希望项目 B 形成：

```text
main.cpp
│
├── LocalizationManager
│      └── MC 0x0a @ 9110
│
├── HDMapManager
│      └── loadMap()
│
└── SutengDriver
       │
       └── ProcessPcapCloud()
              │
              ├── GetFusionLocation()
              │
              ├── PointCloudTransform
              │
              ├── ElevationMapGroundFilter
              │      ├── Grid
              │      ├── Ground
              │      ├── Obstacle
              │      └── Cluster
              │
              ├── HDMapFilter
              │      └── filterClusters()
              │
              ├── ConvertClustersToTrackedObstacles
              │
              ├── m_tracker.update()
              │
              └── sendUdpMsg()
```

其中：

> **HDMapFilter 必须位于 Cluster 生成之后、Tracker 之前。**

第一阶段不要修改 Grid 和 PointCloud 逻辑。

---

# 六、第一阶段代码修改：LocalizationManager

---

## 6.1 定位来源

严格按照：

```text
Project_C_Localization_Queue_Analysis.md
```

中已经确认的定位来源实现：

```text
MC 0x0a @ 9110
```

不要自己重新设计一个定位来源。

---

## 6.2 必须参考项目 A/C 实际代码

找到项目 A/C 中：

```text
OpenfusionLocMCClient
fusionLoc_set_callback
STR_FUSIONLOC
```

的实际定义和使用方式。

然后确认：

* 头文件
* namespace
* 初始化函数
* callback 类型
* client 生命周期
* 订阅方式
* 数据结构定义
* 停止/释放方式

如果项目 B 当前已经存在相关代码但被注释掉：

> 优先恢复和封装现有代码，而不是重新实现一套。来自8月20日提示：我目前已经将当前项目B中的CommInit函数注释掉的OpenfusionLocMCClient重新打开了，加了当前已有的3种模式的判断，在pcd模式下是不会启用hdmap的过滤的。

---

# 九、定位数据必须打印验证

第一阶段不要急着接 HDMap。

先让程序能够启动定位，并增加清晰日志，例如：

```text
Localization initialized
Localization callback received
Localization valid
x=...
y=...
z=...
yaw=...
timestamp=...
```

但不要每一帧疯狂打印。

建议：

```text
启动打印一次
```

以及：

```text
每 N 帧打印一次
```

或者：

```text
定位状态发生变化时打印
```

---

# 十、定位失败不能导致低矮检测退出

必须实现降级策略：

```text
定位有效
    ↓
允许 HDMap Filter

定位无效
    ↓
关闭 HDMap Filter
    ↓
低矮障碍物检测继续运行
```

不要：

```text
定位失败
 ↓
整个程序退出
```

除非项目 C/A 的底层定位库初始化本身是硬依赖，而且代码结构无法做到降级。

如果必须硬依赖，要在最终报告中说明原因。

---

# 十一、第二阶段：实现 HDMapManager

新增：

```text
/home/zyl/echiev_low_lidar_detection/mapfilter/hdmap_manager.h
/home/zyl/echiev_low_lidar_detection/mapfilter/hdmap_manager.cpp
```

或者按照当前项目 B 的目录结构。

---

## 11.1 HDMap 来源

参考项目 A：

```text
read_hdmap.h
read_hdmap.cpp
```

以及其中：

```text
STR_MAP
STR_LANE
STR_SECTION
...
```

等真实结构。

---

## 11.2 不要直接复制整个项目 A

目前8月20日提示，我已经移植了部分hdmap解析的文件到/home/zyl/echiev_low_lidar_detection/mapfilter文件夹下。再帮我继续分析是否还有没移植的文件

只迁移：

```text
HDMap 数据结构
HDMap 文件解析
HDMap 查询所需要的数据
```

剥离：

```text
项目 A 特有日志
项目 A 特有宏
项目 A 特有全局变量
项目 A 特有感知模块
项目 A 特有 PCL 依赖
项目 A 无关业务代码
```

---

# 十二、HDMap 路径必须参数化

默认的地址继续写：

```cpp
#define MAPDIR "/etc/echiev/hdmap/hdmap.bin"
```

不要：

```cpp
char g_ac_map_path[256] = "...";
```

作为 B 项目的最终方案。

应该通过yaml文件：

```text
debug_config.yaml
```

配置。

例如：目前8月20日提示，参考/home/zyl/echiev_low_lidar_detection/config/debug_config_EMX.yaml

```yaml
HdmapFilter:
  mapFilterModel: 1
  mapPath: "/home/zyl/echiev_low_lidar_detection/config/hdmap_shenzhen.bin"
```

另外分析还有一个修改的点，我想当mapPath: ""，为空的话，使用默认地址，不为空，就是mapPath内的地址，这个想法可行吗，尤其是/home/zyl/echiev_low_lidar_detection/common/ReadYamlFile.cpp加入了mapPath的判断。

---

# 十三、HDMapManager 最基本能力

第一阶段至少需要：

```text
loadMap(path)
isLoaded()
getMap()
```

如果项目 A 的地图查询已经有成熟接口：

> 优先封装现有查询能力，不要重新造一套地图解析。

---

# 十四、第三阶段：CoordinateTransformer

考虑是否新增：

```text
mapfilter/coordinate_transformer.h
mapfilter/coordinate_transformer.cpp
```

必须根据：

```text
HDMap_Location_Integration_To_LowObstacle_Project.md
```

中的坐标系分析实现。

重点实现：

```text
vehicleToMap()
mapToVehicle()
lonLatToMap()
```

如果项目实际只需要其中两个，就不要无意义增加接口。目前项目B的障碍物都是雷达系下，会使用udp发送给项目C，它里面会将障碍物转到车体系再转到地图系下。与定位是一个坐标系。

---

# 十五、坐标系必须严格按照已经分析出的定义

目前已确认：

```text
项目 B LiDAR/车辆感知坐标：
x = forward
y = left
z = up
```

但是：

> HDMap 坐标系和 `STR_FUSIONLOC` 的具体定义必须以之前分析文档和项目 A/C 实际代码为准。

不要重新猜测。

特别检查：

```text
yaw
heading
degree/radian
x/y 顺序
ENU
Map origin
```

---

# 十六、必须增加坐标转换单元测试

至少测试：

```text
vehicle → map → vehicle
```

对于若干已知点：

```text
P_vehicle
→ P_map
→ P_vehicle2
```

检查：

```text
error < 合理阈值
```

阈值根据坐标系和浮点误差确定。

---

# 十七、第四阶段：实现 HDMapFilter

新增：

```text
mapfilter/hdmap_filter.h
mapfilter/hdmap_filter.cpp
```

第一阶段：

> **只实现 Cluster 级过滤。**

不要实现：

```text
filterPointCloud()
filterGrid()
```

除非后续确实需要。

---

# 十八、HDMapFilter 输入

推荐逻辑：

```text
Localization Pose
+
GridCluster
+
HDMap
```

例如：

```cpp
filterClusters(
    clusters,
    pose,
    map
);
```

实际接口根据当前项目 B 的 `GridCluster` 真实定义设计。

---

# 十九、Cluster 级过滤流程

必须实现：

```text
Cluster
   ↓
cluster.center
   ↓
vehicleToMap()
   ↓
得到 map 坐标
   ↓
查询 HDMap
   ↓
判断 cluster 是否位于道路区域
   ↓
设置 in_road / confidence
   ↓
软过滤
```

---

# 二十、特别注意：不要直接删除 Cluster

当前方案是：

> **Cluster 级软约束。**

因此第一阶段不要简单：

```cpp
clusters.erase(...)
```

而应该保留 cluster，并在ElevationMapGroundFilter.h的GridCluster结构中增加类似：

```text
in_road
confidence
map_valid
```

等状态。

实际字段根据当前代码结构决定。

---

# 二十一、为什么必须软约束

低矮障碍物可能正好位于：

```text
正常道路
```

例如：

```text
纸箱
石块
轮胎
锥桶
小型障碍物
```

所以：

```text
in road = no obstacle
```

这种逻辑是错误的。

HDMap 第一阶段只作为：

```text
额外空间先验
```

而不是：

```text
绝对真值
```

---

# 二十二、第一阶段推荐过滤策略

如果 Cluster：

```text
地图查询失败
```

则：

```text
保留
```

如果：

```text
定位无效
```

则：

```text
保留
```

如果：

```text
HDMap 未加载
```

则：

```text
保留
```

如果：

```text
cluster 位于道路区域
```

则：

```text
in_road = true
confidence = ...
```

如果：

```text
cluster 位于非道路区域
```

不要立即删除。

应该：

```text
in_road = false
```

然后根据最终过滤策略决定是否降低置信度。

---

# 二十三、如果当前项目 B 没有 confidence 字段

不要为了这个功能大规模修改整个障碍物数据结构。

先分析当前：

```text
GridCluster
TrackedObstacle
```

的结构。

如果最小修改方式是：

```cpp
bool in_road;
float map_confidence;
```

可以增加。

如果现有数据结构已经有类似字段：

> 优先复用。

---

# 二十四、Cluster 中心点必须确认

当前文档使用：

```text
cluster.center
```

但是必须检查项目 B 的真实 Cluster 数据结构。

确认：

```text
center 是在哪里计算的？
```

以及：

```text
center 是车辆坐标系吗？
```

如果 Cluster 中心已经处于：

```text
x forward
y left
z up
```

则直接：

```text
vehicleToMap(center)
```

如果不是：

> 必须先进行正确的坐标转换。

---

# 二十五、地图点面测试

如果项目 A 已经存在：

```text
HdmapROIFilter
Bitmap2D
polygon_mask
```

不要第一阶段全部搬过来。

首先检查项目 A 是否已有简单的：

```text
point in polygon
```

查询。

如果没有，则第一阶段可以实现一个简单可靠的：

```text
ray casting
```

点面测试。

但是：

> 不要为了 Cluster 级过滤引入大量复杂依赖。

---

# 二十六、第五阶段：接入 ProcessPcapCloud

最终接入位置：

```text
ProcessPcapCloud()
```

推荐：

```text
PointCloudTransform
        ↓
ElevationMapGroundFilter
        ↓
Grid
        ↓
Ground
        ↓
Obstacle
        ↓
Cluster
        ↓
HDMapFilter.filterClusters()
        ↓
ConvertClustersToTrackedObstacles
        ↓
m_tracker.update()
        ↓
sendUdpMsg()
```

---

# 二十七、不要破坏原有低矮检测流程

这是非常重要的要求。

HDMap 功能必须是：

```text
可选
```

配置：

```yaml
HdmapFilter:
  mapFilterModel: 1
```

时：

> 项目 B 的行为必须尽可能与修改前完全一致。

即：

```text
mapFilterModel: 0
```

则：

```text
不初始化 HDMap
不执行地图过滤
不改变 Cluster
不改变 Tracker
```

---

# 二十八、Localization.enable 也必须可控

例如：

```yaml
Localization:
    enable: 1
```

如果：

```yaml
Localization:
    enable: 0
```

则：

```text
不启动定位订阅
HDMapFilter 自动关闭
低矮检测仍可运行
```

---

# 二十九、推荐增加明确状态

程序运行时最好能够看到：

```text
[Localization] enabled
[Localization] initialized
[Localization] valid

[HDMap] enabled
[HDMap] loaded
[HDMap] map objects = xxx

[HDMapFilter] enabled
[HDMapFilter] mode = cluster_soft
```

如果关闭：

```text
[HDMap] disabled
```

如果定位无效：

```text
[HDMapFilter] disabled: localization invalid
```

---

# 三十、配置文件修改

修改：

```text
common/ReadYamlFile.h
common/ReadYamlFile.cpp
```

以及当前项目 B 实际 YAML 配置结构。

增加：

```yaml

Localization:
    enable: 1

```

需要定义清晰关系：

```text
mapFilterModel = 0
    ↓
MapFilter 自动不可用

Localization.enable = 0
    ↓
MapFilter 自动不可用
```

---

# 三十一、CMake 修改

将新增 `.cpp` 文件加入当前：

```text
AUX_SOURCE_DIRECTORY
```

或者当前 CMake 的源码列表。

新增：

```text
hdmap_manager.cpp
coordinate_transformer.cpp
localization_manager.cpp
hdmap_filter.cpp
```

具体路径以项目实际结构为准。

---

# 三十二、外部依赖必须谨慎

优先使用项目 B 已经存在的：

```text
C++
Eigen
PCL
yaml-cpp
```

不要因为 HDMap 功能随意新增第三方库。

尤其：

> 不要重新引入项目 A 中可能存在的旧 Boost/PCL/FLANN 依赖。

我之前已经遇到过：

```text
Boost 1.58
Boost 1.71
PCL
FLANN
```

版本冲突导致运行时 Segmentation Fault 的问题。

所以本次迁移必须特别注意：

> **不要把项目 A 编译环境中的旧第三方库整体带入项目 B。**

如果项目 A 的 HDMap 代码依赖某个库：

1. 先确认项目 B 是否已经存在；
2. 再确认 ABI/版本；
3. 能纯 C++ 实现就不要增加依赖；
4. 不要直接复制 A 的链接库列表。

---

# 三十三、定位库同样必须检查依赖

如果：

```text
OpenfusionLocMCClient
fusionLoc_set_callback
```

来自外部库：

请先确认：

```text
头文件在哪里？
库在哪里？
CMake 怎么链接？
运行时需要什么 so？
```

不要直接复制项目 C 的全部链接参数。

只提取定位功能实际需要的依赖。

---

# 三十四、修改顺序必须严格分阶段

不要一次性修改全部代码。

推荐：

## Phase 1：Localization

只完成：

```text
LocalizationManager
↓
MC 0x0a @ 9110
↓
STR_FUSIONLOC
↓
getPose()
```

---

## Phase 2：HDMap

加入：

```text
HDMapManager
↓
load hdmap.bin
```

但：

> 此时先不要过滤 Cluster。

只验证：

```text
HDMap loaded
map object count
```



---

## Phase 3：CoordinateTransformer

实现：

```text
vehicleToMap
mapToVehicle
```

增加测试。

确认坐标正确。

---

## Phase 4：HDMapFilter

只实现：

```text
filterClusters()
```

暂时不要修改 Grid。

---

## Phase 5：接入 ProcessPcapCloud

最终：

```text
Cluster
↓
HDMapFilter
↓
Tracker
```



---

# 三十六、必须增加 Debug 日志

为了验证地图过滤，在std::vector<GridCluster> ElevationMapGroundFilter::ClusterObstacleGrid函数中需要能够输出：

```text
frame_id
cluster_id
cluster.center(vehicle)
cluster.center(map)
in_road
map_valid
confidence
```


# 三十八、必须处理边界和定位异常

至少考虑：

```text
定位无效
定位长时间不更新
定位 timestamp 异常
定位跳变
HDMap 未加载
HDMap 查询失败
cluster 在地图范围外
cluster 位于地图 polygon 边界
```

默认原则：

> **不确定时保留障碍物，而不是删除障碍物。**

因为当前目标是低矮障碍物检测，宁可第一阶段少过滤，也不要因为地图错误造成漏检。

---

# 三十九、第一阶段不要做这些事情

明确禁止：

```text
❌ Grid 级 HDMap Filter
❌ PointCloud 级 HDMap Filter
❌ 修改 Ground Filter 算法
❌ 修改 Cluster 算法
❌ 修改 Tracker 算法
❌ 修改 Kalman
❌ 修改 Hungarian
❌ 修改 UDP 输出协议
❌ 大规模重构项目 B
```

本次只完成：

```text
Localization
+
HDMap
+
Coordinate Transform
+
Cluster Soft Constraint
```

---

# 四十、每完成一个 Phase 必须编译

不要最后一次性编译。

严格执行：

```text
Phase 1
↓
编译
↓
解决问题
↓
运行验证

Phase 2
↓
编译
↓
运行验证

Phase 3
↓
编译
↓
单测

Phase 4
↓
编译
↓
运行验证

Phase 5
↓
最终编译
↓
PCAP 回放
```

---

# 四十一、如果遇到编译错误

不要直接修改到“能编译”为止。

必须分析：

```text
错误来源
↓
A/C 代码接口
↓
B 项目现有接口
↓
最小兼容修改
```

尤其注意：

```text
namespace
typedef
struct
callback
ABI
extern "C"
```

等问题。

---

# 四十二、如果定位库接口与文档不一致

例如发现：

```text
OpenfusionLocMCClient
fusionLoc_set_callback
```

在当前环境中实际 API 不完全一样：

> 不要猜 API。

应该：

1. 搜索头文件；
2. 搜索项目 C 实际调用；
3. 搜索项目 A 实际调用；
4. 确认真实函数签名；
5. 再适配项目 B。

---

# 四十三、最终需要修改的核心文件

根据当前项目实际目录，最终预计涉及：

```text
新增：

localization_manager.h
localization_manager.cpp

hdmap_manager.h
hdmap_manager.cpp

coordinate_transformer.h
coordinate_transformer.cpp

hdmap_filter.h
hdmap_filter.cpp
```

以及修改：

```text
main.cpp

SuTengDriver.h
SuTengDriver.cpp

ReadYamlFile.h
ReadYamlFile.cpp

Type.h
CMakeLists.txt
debug_config.yaml
```

但是：

> **这不是强制文件列表。**

如果当前项目已有对应模块，则优先复用，而不是重复创建。

---

# 四十四、代码质量要求

新增代码必须：

* C++14
* RAII
* 避免裸指针
* 避免全局变量
* 不使用 `using namespace std;`
* 不产生循环依赖
* 头文件使用 include guard 或 `#pragma once`
* 日志风格与项目 B 保持一致
* 错误必须有明确返回值
* 不吞异常/错误
* 不修改无关代码

---

---

# 四十六、最终输出一个修改总结

代码修改完成后，生成：

```text
docs/HDMap_Localization_Implementation.md
```

内容至少包括：

```text
1. 修改了哪些文件

2. 每个文件为什么修改

3. LocalizationManager 如何工作

4. 定位数据从哪里来

5. HDMap 如何加载

6. CoordinateTransformer 如何工作

7. HDMapFilter 如何工作

8. Cluster 过滤发生在哪里

9. 配置项

10. 编译方式

11. 运行方式

12. 日志示例

13. 已验证功能

14. 尚未验证功能

15. 已知问题

16. 后续 Grid/PointCloud 过滤计划
```

---

# 四十七、最终必须给出代码修改前后数据流

最终文档中必须给出：

## 修改前

```text
PointCloud
 ↓
Transform
 ↓
Ground Filter
 ↓
Grid
 ↓
Cluster
 ↓
Tracker
 ↓
UDP
```

## 修改后

```text
Localization
      │
      ↓
STR_FUSIONLOC
      │
      ↓
LocalizationManager
      │
      │
HDMap ──→ HDMapManager
      │
      ↓
CoordinateTransformer
      │
      ↓

PointCloud
 ↓
Transform
 ↓
Ground Filter
 ↓
Grid
 ↓
Cluster
 ↓
HDMapFilter
 │
 ├── map valid → in_road/confidence
 │
 └── map invalid → 保留
 ↓
Tracker
 ↓
UDP
```

---

# 四十八、最终原则

整个实施过程必须遵循：

> **最小修改、逐步验证、可关闭、可降级、不破坏原有低矮检测。**

尤其是：

```text
定位 ≠ 队列
队列 ≠ 定位源
HDMap ≠ 障碍物真值
道路区域 ≠ 没有障碍物
地图查询失败 ≠ 删除障碍物
定位失败 ≠ 低矮检测退出
```

第一阶段最重要的目标不是“地图过滤掉多少障碍物”。

而是先建立一个可靠的数据链：

```text
公司定位源
   ↓
STR_FUSIONLOC
   ↓
LocalizationManager
   ↓
Vehicle Pose
   ↓
CoordinateTransformer
   ↓
HDMap
   ↓
Cluster
   ↓
HDMap Soft Constraint
```

在这条链路经过实际运行验证之前，不要继续实现 Grid/PointCloud 级地图过滤。

完成代码修改后，必须告诉我：

1. 实际修改了哪些文件；
2. 每个文件修改了什么；
3. 编译是否成功；
4. 运行是否成功；
5. 定位是否收到；
6. HDMap 是否成功加载；
7. 坐标转换是否验证；
8. Cluster 是否成功进入 HDMapFilter；
9. 是否存在尚未解决的问题；
10. 给出下一步建议。

不要为了完成任务而修改与本需求无关的代码。
