// ============================================================================
// 坐标转换单元测试：vehicle -> map -> vehicle 互逆性验证
// ============================================================================
// 编译（在项目根目录）：
//   g++ -std=c++14 -I./mapfilter
//       mapfilter/test_coordinate_transform.cpp
//       mapfilter/coordinate_transformer.cpp mapfilter/convert.cpp -o /tmp/test_ct
// 运行：
//   /tmp/test_ct
// 全部通过返回 0，失败返回 1。
// ============================================================================

#include "coordinate_transformer.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <initializer_list>

using namespace Lidar_Low_Detection;

static int g_failures = 0;

#define CHECK(cond, msg)                                                     \
    do {                                                                     \
        if (!(cond)) {                                                       \
            printf("  [FAIL] %s (line %d)\n", msg, __LINE__);                \
            ++g_failures;                                                    \
        } else {                                                             \
            printf("  [PASS] %s\n", msg);                                    \
        }                                                                    \
    } while (0)

static double dist2(double ax, double ay, double bx, double by)
{
    const double dx = ax - bx;
    const double dy = ay - by;
    return std::sqrt(dx * dx + dy * dy);
}

static void testRoundTrip(const VehiclePose& pose, double vx, double vy,
                          const char* label)
{
    double mx = 0, my = 0;
    CoordinateTransformer::vehicleToMap(vx, vy, pose, mx, my);

    double vx2 = 0, vy2 = 0;
    CoordinateTransformer::mapToVehicle(mx, my, pose, vx2, vy2);

    const double err = dist2(vx, vy, vx2, vy2);
    char buf[256];
    std::snprintf(buf, sizeof(buf),
                  "round-trip %s: veh(%.3f,%.3f)->map(%.3f,%.3f)->veh(%.3f,%.3f) err=%.6f",
                  label, vx, vy, mx, my, vx2, vy2, err);
    CHECK(err < 1e-3, buf);
}

static void testHeadingDirections()
{
    printf("== 特殊航向方向验证 ==\n");
    VehiclePose pose;
    pose.x = 0.0; pose.y = 0.0;

    // h=0（车头朝北）：车辆前方 1m -> 地图 (0, 1)（北）
    pose.heading_deg = 0.0;
    {
        double mx, my;
        CoordinateTransformer::vehicleToMap(1.0, 0.0, pose, mx, my);
        CHECK(dist2(mx, my, 0.0, 1.0) < 1e-6, "h=0: front(1,0) -> map(0,1) north");
    }

    // h=90（车头朝东）：车辆前方 1m -> 地图 (1, 0)（东）
    pose.heading_deg = 90.0;
    {
        double mx, my;
        CoordinateTransformer::vehicleToMap(1.0, 0.0, pose, mx, my);
        CHECK(dist2(mx, my, 1.0, 0.0) < 1e-6, "h=90: front(1,0) -> map(1,0) east");
    }

    // h=180（车头朝南）：前方 1m -> 地图 (0,-1)
    pose.heading_deg = 180.0;
    {
        double mx, my;
        CoordinateTransformer::vehicleToMap(1.0, 0.0, pose, mx, my);
        CHECK(dist2(mx, my, 0.0, -1.0) < 1e-6, "h=180: front(1,0) -> map(0,-1) south");
    }

    // h=-90（车头朝西）：前方 1m -> 地图 (-1,0)
    pose.heading_deg = -90.0;
    {
        double mx, my;
        CoordinateTransformer::vehicleToMap(1.0, 0.0, pose, mx, my);
        CHECK(dist2(mx, my, -1.0, 0.0) < 1e-6, "h=-90: front(1,0) -> map(-1,0) west");
    }

    // 车辆左方 1m（y=1）在 h=0（朝北）时应为地图 (-1,0)（西，因为 y+ = 左 = 西）
    pose.heading_deg = 0.0;
    {
        double mx, my;
        CoordinateTransformer::vehicleToMap(0.0, 1.0, pose, mx, my);
        CHECK(dist2(mx, my, -1.0, 0.0) < 1e-6, "h=0: left(0,1) -> map(-1,0) west");
    }
}

static void testLonLat()
{
    printf("== 经纬度转换验证 ==\n");
    // 锚点自身 -> (0,0)
    const double anchor_lon = 114.054429823102;
    const double anchor_lat = 22.6656342253927;

    double mx, my;
    CoordinateTransformer::lonLatToMap(anchor_lon, anchor_lat,
                                       anchor_lon, anchor_lat, mx, my);
    CHECK(dist2(mx, my, 0.0, 0.0) < 1e-3, "lonLatToMap(anchor) -> (0,0)");

    // 锚点向正北移动 0.0001°（约 11.1m，WGS84 椭球纬线弧长）
    const double dlat = 0.0001;
    CoordinateTransformer::lonLatToMap(anchor_lon, anchor_lat + dlat,
                                       anchor_lon, anchor_lat, mx, my);
    CHECK(my > 10.0 && my < 12.0, "lonLatToMap(north 0.0001deg) -> y≈11.1");
    CHECK(std::fabs(mx) < 0.5, "lonLatToMap(north) -> x≈0");

    // 锚点向正东移动 0.0001°（约 10.3m）
    const double dlon = 0.0001;
    CoordinateTransformer::lonLatToMap(anchor_lon + dlon, anchor_lat,
                                       anchor_lon, anchor_lat, mx, my);
    CHECK(mx > 10.0 && mx < 11.0, "lonLatToMap(east 0.0001deg) -> x≈10.3");
    CHECK(std::fabs(my) < 0.5, "lonLatToMap(east) -> y≈0");

    // 正逆变换互逆：lonLatToMap -> mapToLonLat 应还原经纬度
    double lon, lat;
    CoordinateTransformer::mapToLonLat(mx, my, anchor_lon, anchor_lat, lon, lat);
    CHECK(std::fabs(lon - (anchor_lon + dlon)) < 1e-9, "mapToLonLat inverse lon ok");
    CHECK(std::fabs(lat - anchor_lat) < 1e-9, "mapToLonLat inverse lat ok");
}

int main()
{
    printf("=== CoordinateTransformer 单元测试 ===\n");

    printf("== 互逆性验证（多航向多位置）==\n");
    const double headings[] = { 0.0, 45.0, 90.0, 135.0, 180.0, -90.0, -45.0, 273.0 };
    for (double h : headings)
    {
        for (double px : { -2.0, 0.0, 1.5, 8.0 })
        {
            for (double py : { -3.0, -0.5, 0.0, 2.7 })
            {
                VehiclePose pose;
                pose.x = 100.0;
                pose.y = -50.0;
                pose.heading_deg = h;
                char label[64];
                std::snprintf(label, sizeof(label), "h=%.0f veh=(%.1f,%.1f)", h, px, py);
                testRoundTrip(pose, px, py, label);
            }
        }
    }

    // 原点位姿 + 不同车辆位置
    VehiclePose origin;
    origin.x = 0.0; origin.y = 0.0; origin.heading_deg = 30.0;
    testRoundTrip(origin, 3.0, 1.2, "origin veh=(3,1.2)");
    testRoundTrip(origin, -1.0, -2.5, "origin veh=(-1,-2.5)");

    testHeadingDirections();
    testLonLat();

    printf("\n=== 结果: %s (%d failures) ===\n",
           g_failures == 0 ? "ALL PASS" : "FAILED", g_failures);
    return g_failures == 0 ? 0 : 1;
}
