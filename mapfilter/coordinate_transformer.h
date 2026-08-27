#ifndef LOW_DETECTION_COORDINATE_TRANSFORMER_H
#define LOW_DETECTION_COORDINATE_TRANSFORMER_H

namespace Lidar_Low_Detection
{

/**
 * @brief 车辆在地图中的位姿（由 LocalizationManager 提供）
 *
 * 坐标系约定（与项目 A / C 一致，见迁移设计文档第 13/15 章）：
 *   - 车辆系：x 前向(forward)、y 左向(left)、z 上向(up)
 *   - 地图系：ENU（x=东 E，y=北 N，单位米），原点为地图锚点
 *   - fHeadingInMap：北=0，顺时针为正（Compass/NovAtel 约定），单位度
 */
struct VehiclePose
{
    double x          = 0.0;   ///< 车辆在地图中的 X（东，m）
    double y          = 0.0;   ///< 车辆在地图中的 Y（北，m）
    double heading_deg = 0.0;  ///< 车辆地图航向（度，北=0 顺时针）
};

/**
 * @brief 车辆系 ↔ 地图系 坐标转换器
 *
 * 数学公式移植自项目 A（与项目 C 数学等价，见迁移文档第 13 章）：
 *   - vehicleToMap : P_map = R(-heading)·[-y_veh; x_veh] + P_car
 *                    （项目 A lidar_capture.cpp VehicleToWorld）
 *   - mapToVehicle : P_veh = R(-(90-heading))·(P_map - P_car)
 *                    （项目 A get_roi.cpp PointTransform，互逆）
 *
 * 本模块为纯计算，无任何第三方依赖，便于单元测试。
 */
class CoordinateTransformer
{
public:
    // 删除默认构造函数，禁止实例化
    CoordinateTransformer() = default;
    ~CoordinateTransformer() = default;


    // ---- 车辆系（其实是主激光雷达系） → 地图系 ----
    static void vehicleToMap(double veh_x, double veh_y,
                             const VehiclePose& pose,
                             double& map_x, double& map_y);

    // ---- 地图系 → 车辆系（其实是主激光雷达系）（vehicleToMap 的严格逆变换）----
    static void mapToVehicle(double map_x, double map_y,
                             const VehiclePose& pose,
                             double& veh_x, double& veh_y);

    // ---- GPS 经纬度 → 地图系（RTKLIB ENU，锚点为地图锚点）----
    // lon/lat 单位度；anchor_lon/anchor_lat 为地图锚点经纬度（度）
    static void lonLatToMap(double lon, double lat,
                            double anchor_lon, double anchor_lat,
                            double& map_x, double& map_y);

    // ---- 地图系 → GPS 经纬度（RTKLIB ENU 逆变换，调试用）----
    static void mapToLonLat(double map_x, double map_y,
                            double anchor_lon, double anchor_lat,
                            double& lon, double& lat);

    // ---- 主雷达系(Grid) ↔ 融合IMU系 航向补偿（度）----
    // 融合定位 pose.heading 是 IMU 航向；ElevationMap 点云/Grid 是主雷达系(x前)。
    // 两者前向相差补偿角 = 90° + fLidar2Vehicle_Heading（lidar.cfg，见 docs/HDMap可视化.md 第五节）。
    // 默认 0；仅在低矮检测启动时由 main.cpp 根据 lidar.cfg 设置一次。本模块内部统一应用，
    // 保证可视化(DrawHdMapOverlay)与过滤(HDMapFilter)使用同一补偿，避免调用点遗漏/重复。
    static void setGridHeadingOffsetDeg(double deg);
    static double gridHeadingOffsetDeg();

private:
    static double s_grid_heading_offset_deg;
};

} // namespace Lidar_Low_Detection

#endif // LOW_DETECTION_COORDINATE_TRANSFORMER_H
