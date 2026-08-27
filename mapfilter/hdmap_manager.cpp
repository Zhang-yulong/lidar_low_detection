#include "hdmap_manager.h"

#include <cmath>
#include <cfloat>
#include <cstdio>
#include <set>

namespace Lidar_Low_Detection
{

HDMapManager::HDMapManager()
    : rd_map_(new read_hdmap()),
      loaded_(false)
{
}

HDMapManager::~HDMapManager()
{
    delete rd_map_;
    rd_map_ = nullptr;
}

bool HDMapManager::loadMap(const std::string& path)
{
    if (path.empty())
    {
        printf("[HDMap] loadMap: path empty, load failed\n");
        loaded_ = false;
        return false;
    }

    // 先释放旧地图（若已加载）
    if (loaded_)
    {
        delete rd_map_;
        rd_map_ = new read_hdmap();
        loaded_ = false;
    }

    const int ret = rd_map_->load(path.c_str());
    if (ret != 0 || rd_map_->get_hdmap() == nullptr ||
        rd_map_->get_hdmap()->iCountLane <= 0)
    {
        printf("[HDMap] loadMap failed: %s\n", path.c_str());
        loaded_ = false;
        return false;
    }

    loaded_ = true;
    int lanes = 0, sections = 0, roads = 0, junctions = 0;
    getStats(lanes, sections, roads, junctions);
    printf("[HDMap] loaded: %s\n", path.c_str());
    printf("[HDMap] map objects: lanes=%d sections=%d roads=%d junctions=%d\n",
           lanes, sections, roads, junctions);
    return true;
}

STR_MAP* HDMapManager::getMap() const
{
    if (!loaded_ || rd_map_ == nullptr)
    {
        return nullptr;
    }
    return rd_map_->get_hdmap();
}

void HDMapManager::getStats(int& lanes, int& sections, int& roads, int& junctions) const
{
    lanes = sections = roads = junctions = 0;
    if (!loaded_ || rd_map_->get_hdmap() == nullptr)
    {
        return;
    }
    const STR_MAP* m = rd_map_->get_hdmap();
    lanes     = m->iCountLane;
    sections  = m->iCountSection;
    roads     = m->iCountRoad;
    junctions = m->iCountJunction;
}

int HDMapManager::findSectionIndexById(int section_id) const
{
    if (!loaded_ || rd_map_->get_hdmap() == nullptr)
    {
        return -1;
    }
    const STR_MAP* m = rd_map_->get_hdmap();
    for (int i = 0; i < m->iCountSection; ++i)
    {
        if (m->pstrSection[i].iId == section_id)
        {
            return i;
        }
    }
    return -1;
}

int HDMapManager::getCurrentSectionIndex(double map_x, double map_y) const
{
    if (!loaded_ || rd_map_->get_hdmap() == nullptr)
    {
        return -1;
    }

    STR_POINT2F pt;
    pt.fX = static_cast<float>(map_x);
    pt.fY = static_cast<float>(map_y);

    // 1) 定位所在 Lane（最邻近）
    const STR_ID_ARRAY lane_ids = rd_map_->GetLaneIdByPoint(pt);
    if (lane_ids.iCount <= 0)
    {
        return -1;
    }

    // 2) Lane -> Section ID
    int section_id = -1;
    for (int i = 0; i < lane_ids.iCount; ++i)
    {
        const int sid = rd_map_->GetSectionIdByLaneId(lane_ids.piID[i]);
        if (sid > 0)
        {
            section_id = sid;
            break;
        }
    }
    if (section_id <= 0)
    {
        return -1;
    }

    // 3) Section ID -> 数组索引
    return findSectionIndexById(section_id);
}

bool HDMapManager::buildDrivablePolygons(
    double map_x, double map_y,
    std::vector<std::vector<STR_POINT2F>>& polygons_out,
    int* section_count) const
{
    polygons_out.clear();
    if (section_count)
    {
        *section_count = 0;
    }
    if (!loaded_ || rd_map_->get_hdmap() == nullptr)
    {
        return false;
    }

    const STR_MAP* m = rd_map_->get_hdmap();
    const int cur_idx = getCurrentSectionIndex(map_x, map_y);
    if (cur_idx < 0 || cur_idx >= m->iCountSection)
    {
        return false;
    }

    // 收集相关 Section 索引：当前 + income + outgo
    std::set<int> section_indices;
    section_indices.insert(cur_idx);

    const STR_SECTION& cur = m->pstrSection[cur_idx];
    for (int i = 0; i < cur.iCountIncome; ++i)
    {
        const int idx = findSectionIndexById(cur.piIncomeSectionId[i]);
        if (idx >= 0)
        {
            section_indices.insert(idx);
        }
    }
    for (int i = 0; i < cur.iCountOutgo; ++i)
    {
        const int idx = findSectionIndexById(cur.piOutgoSectionId[i]);
        if (idx >= 0)
        {
            section_indices.insert(idx);
        }
    }

    // 为每个 Section 构建闭合多边形：左边界 + 右边界反向
    for (const int idx : section_indices)
    {
        const STR_SECTION& sec = m->pstrSection[idx];
        const int n = sec.iCountLeftRight;
        if (n < 3 || sec.pstrLeft == nullptr || sec.pstrRight == nullptr)
        {
            continue;
        }

        std::vector<STR_POINT2F> poly;
        poly.reserve(static_cast<size_t>(2 * n));
        for (int i = 0; i < n; ++i)
        {
            poly.push_back(sec.pstrLeft[i]);
        }
        for (int i = n - 1; i >= 0; --i)
        {
            poly.push_back(sec.pstrRight[i]);
        }
        polygons_out.push_back(std::move(poly));
    }

    if (section_count)
    {
        *section_count = static_cast<int>(polygons_out.size());
    }
    return !polygons_out.empty();
}

bool HDMapManager::isPointInDrivableArea(double map_x, double map_y, double margin) const
{
    std::vector<std::vector<STR_POINT2F>> polys;
    if (!buildDrivablePolygons(map_x, map_y, polys))
    {
        return false;
    }
    for (const auto& poly : polys)
    {
        if (pointInPolygon(map_x, map_y, poly))
        {
            return true;
        }
        if (margin > 0.0 && distanceToPolygon(map_x, map_y, poly) <= margin)
        {
            return true;
        }
    }
    return false;
}

// ---------------------------------------------------------------------------
// 射线法点面测试
// ---------------------------------------------------------------------------
bool HDMapManager::pointInPolygon(double px, double py,
                                  const std::vector<STR_POINT2F>& poly)
{
    const size_t n = poly.size();
    if (n < 3)
    {
        return false;
    }

    bool inside = false;
    for (size_t i = 0, j = n - 1; i < n; j = i++)
    {
        const double xi = poly[i].fX, yi = poly[i].fY;
        const double xj = poly[j].fX, yj = poly[j].fY;

        const bool intersects = ((yi > py) != (yj > py)) &&
            (px < (xj - xi) * (py - yi) / (yj - yi) + xi);
        if (intersects)
        {
            inside = !inside;
        }
    }
    return inside;
}

double HDMapManager::distanceToPolygon(double px, double py,
                                       const std::vector<STR_POINT2F>& poly)
{
    const size_t n = poly.size();
    if (n < 2)
    {
        return DBL_MAX;
    }

    double min_dist = DBL_MAX;
    for (size_t i = 0, j = n - 1; i < n; j = i++)
    {
        const double ax = poly[j].fX, ay = poly[j].fY;
        const double bx = poly[i].fX, by = poly[i].fY;

        // 点到线段 (a,b) 的最短距离
        const double dx = bx - ax;
        const double dy = by - ay;
        const double len2 = dx * dx + dy * dy;
        double t = 0.0;
        if (len2 > 0.0)
        {
            t = ((px - ax) * dx + (py - ay) * dy) / len2;
            t = (t < 0.0) ? 0.0 : (t > 1.0 ? 1.0 : t);
        }
        const double cx = ax + t * dx;
        const double cy = ay + t * dy;
        const double dist = std::sqrt((px - cx) * (px - cx) + (py - cy) * (py - cy));
        if (dist < min_dist)
        {
            min_dist = dist;
        }
    }
    return min_dist;
}

} // namespace Lidar_Low_Detection
