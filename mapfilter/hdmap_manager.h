#ifndef LOW_DETECTION_HDMAP_MANAGER_H
#define LOW_DETECTION_HDMAP_MANAGER_H

#include <string>
#include <vector>

#include "read_hdmap.h"

namespace Lidar_Low_Detection
{

/**
 * @brief HDMap 管理器：封装项目 A 移植的高精地图解析与查询
 *
 * 职责：
 *   - loadMap(path)   按配置路径加载 hdmap.bin（路径参数化，失败返回 false）
 *   - getMap()        返回 STR_MAP（地图结构体）
 *   - 查询车辆所在 Section / 构建可行驶区域多边形
 *   - isPointInDrivableArea 点面测试（射线法 + 边界外扩 margin）
 *
 * 说明：
 *   - 地图只加载一次（启动时），之后只读共享，无需加锁（无并发写）
 *   - 依赖仅标准库 + RTKLIB ENU（convert.cpp），无 PCL / PROJ 依赖
 *   - 地图加载失败不影响低矮检测主流程（调用方应降级为"不过滤"）
 */
class HDMapManager
{
public:
    HDMapManager();
    ~HDMapManager();

    HDMapManager(const HDMapManager&) = delete;
    HDMapManager& operator=(const HDMapManager&) = delete;

    /// 加载地图：path 为空或加载失败返回 false
    bool loadMap(const std::string& path);

    bool isLoaded() const { return loaded_; }

    /// 返回地图指针（未加载返回 NULL）
    STR_MAP* getMap() const;

    /// 统计地图元素数量（调试打印用）
    void getStats(int& lanes, int& sections, int& roads, int& junctions) const;

    /// 根据地图坐标 (x,y) 返回车辆所在 Section 的索引；失败返回 -1
    int getCurrentSectionIndex(double map_x, double map_y) const;

    /// 根据 Section ID 查找其数组索引；失败返回 -1
    int findSectionIndexById(int section_id) const;

    /**
     * @brief 构建车辆附近的可行驶区域多边形（地图系，单位米）
     *
     * 包含：车辆所在 Section + 其 income/outgo Section 的左右边界线围成的多边形。
     * 每个多边形 = 左边界点列 + 右边界点列反向闭合。
     *
     * @param map_x, map_y   车辆地图坐标（用于定位当前 Section）
     * @param polygons_out   输出多边形列表（地图系坐标）
     * @param section_count  [可选] 输出的有效多边形个数
     * @return true 至少构建出一个有效多边形
     */
    bool buildDrivablePolygons(
        double map_x, double map_y,
        std::vector<std::vector<STR_POINT2F>>& polygons_out,
        int* section_count = nullptr) const;

    /**
     * @brief 判断地图坐标点是否位于可行驶（道路）区域
     * @param margin 边界外扩距离(m)：点在多边形内或距边界 <= margin 都算"在道路"
     * @return 查询失败（无多边形）返回 false；点在道路内返回 true
     */
    bool isPointInDrivableArea(double map_x, double map_y, double margin) const;

    /// 通用点面测试：射线法判断点是否在多边形内
    static bool pointInPolygon(double px, double py,
                               const std::vector<STR_POINT2F>& poly);

    /// 点到多边形边界的最短距离（m）；多边形无效返回很大值
    static double distanceToPolygon(double px, double py,
                                    const std::vector<STR_POINT2F>& poly);

private:
    read_hdmap* rd_map_;
    bool        loaded_;
};

} // namespace Lidar_Low_Detection

#endif // LOW_DETECTION_HDMAP_MANAGER_H
