#include "coordinate_transformer.h"
#include "convert.h"   // RTKLIB ENU: PointDeg2Enu / PointEnu2Deg

#include <cmath>

namespace Lidar_Low_Detection
{

//匿名命名空间：限制内部标识符的作用域仅限于当前 .cpp 文件
namespace
{
constexpr double kPi      = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;

// 读取融合项目的fusion_predict.cfg文件
constexpr double ToIMU_R23_Matrix[6] =  {0.0, -1.0, 0.0, 1.0, 0.0, 0.0};
}

// ============================================================================
// 静态成员：主雷达系(Grid) ↔ 融合IMU系 航向补偿（度）
// 默认 0（不补偿）。由 main.cpp 启动时根据 lidar.cfg fLidar2Vehicle_Heading 设置：
//   offset = 90° + fLidar2Vehicle_Heading（见 docs/HDMap可视化.md 第五节）
// 本模块在 mapToVehicle / vehicleToMap 内部统一应用，可视化与过滤共用。
// ============================================================================
double CoordinateTransformer::s_grid_heading_offset_deg = 0.0;

void CoordinateTransformer::setGridHeadingOffsetDeg(double deg)
{
    s_grid_heading_offset_deg = deg;
}

double CoordinateTransformer::gridHeadingOffsetDeg()
{
    return s_grid_heading_offset_deg;
}



// ---------------------------------------------------------------------------
// 车辆系（其实应该是主激光雷达系） → 地图系
// 移植自项目 A lidar_capture.cpp:614 VehicleToWorld
//   fx = -veh_y;  fy = veh_x;
//   fAngle = heading * PI / 180;
//   map_x = cos(fAngle)*fx + sin(fAngle)*fy + car_x;
//   map_y = -sin(fAngle)*fx + cos(fAngle)*fy + car_y;
// ---------------------------------------------------------------------------
void CoordinateTransformer::vehicleToMap(double veh_x, double veh_y,
                                         const VehiclePose& pose,
                                         double& map_x, double& map_y)
{
    // 车辆系绕 Z 旋转 -90°：车辆前向(X) → 地图 +Y(北)，车辆左向(Y) → 地图 +X(东)
    //ToIMU_R23_Matrix的内容
    const double fx = -veh_y;
    const double fy =  veh_x;

    // heading 加主雷达系↔融合IMU系 航向补偿（默认 0，启动时由 lidar.cfg 计算设置）
    const double fAngle = (pose.heading_deg + s_grid_heading_offset_deg) * kDeg2Rad;

    map_x = std::cos(fAngle) * fx + std::sin(fAngle) * fy + pose.x;
    map_y = -std::sin(fAngle) * fx + std::cos(fAngle) * fy + pose.y;
}

// ---------------------------------------------------------------------------
// 地图系 → 车辆系（vehicleToMap 的严格逆变换）
// 移植自项目 A get_roi.cpp:873 PointTransform（互逆对）
//   dx = map_x - car_x;  dy = map_y - car_y;
//   theta = -(90 - heading) * PI / 180;
//   veh_x = cos(theta)*dx - sin(theta)*dy;
//   veh_y = sin(theta)*dx + cos(theta)*dy;
// ---------------------------------------------------------------------------
void CoordinateTransformer::mapToVehicle(double map_x, double map_y,
                                         const VehiclePose& pose,
                                         double& veh_x, double& veh_y)
{
    const double dx = map_x - pose.x;
    const double dy = map_y - pose.y;

    // heading 加主雷达系↔融合IMU系 航向补偿（默认 0，启动时由 lidar.cfg 计算设置）
    const double theta = -(90.0 - (pose.heading_deg + s_grid_heading_offset_deg)) * kDeg2Rad;
    const double c = std::cos(theta);
    const double s = std::sin(theta);

    veh_x = c * dx - s * dy;
    veh_y = s * dx + c * dy;
}

// ---------------------------------------------------------------------------
// GPS 经纬度 → 地图系（RTKLIB ENU，与项目 A read_hdmap::LonLat2XY 一致）
// PointDeg2Enu(point, deg, enu):
//   point[0]=锚点纬度 point[1]=锚点经度；deg[0]=目标纬度 deg[1]=目标经度
//   enu[0]=东 enu[1]=北 enu[2]=上（单位米）
// ---------------------------------------------------------------------------
void CoordinateTransformer::lonLatToMap(double lon, double lat,
                                        double anchor_lon, double anchor_lat,
                                        double& map_x, double& map_y)
{
    double anchorpoint[3] = { anchor_lat, anchor_lon, 0.0 };
    double latlonpoint[3] = { lat,       lon,       0.0 };
    double xypoint[3]     = { 0.0, 0.0, 0.0 };

    PointDeg2Enu(anchorpoint, latlonpoint, xypoint);

    map_x = xypoint[0];
    map_y = xypoint[1];
}

// ---------------------------------------------------------------------------
// 地图系 → GPS 经纬度（RTKLIB ENU 逆变换，调试用）
// ---------------------------------------------------------------------------
void CoordinateTransformer::mapToLonLat(double map_x, double map_y,
                                        double anchor_lon, double anchor_lat,
                                        double& lon, double& lat)
{
    double anchorpoint[3] = { anchor_lat, anchor_lon, 0.0 };
    double xypoint[3]     = { map_x, map_y, 0.0 };
    double latlonpoint[3] = { 0.0, 0.0, 0.0 };

    PointEnu2Deg(anchorpoint, xypoint, latlonpoint);

    lat = latlonpoint[0];
    lon = latlonpoint[1];
}

} // namespace Lidar_Low_Detection
