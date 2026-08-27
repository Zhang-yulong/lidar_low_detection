#ifndef TYPE_H
#define TYPE_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>  // 各种点云数据类型
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>   // 包含了用于可视化点云的函数和类，用于在3D视窗中现实点云数据

#include <Eigen/Core>

namespace Lidar_Low_Detection
{  

typedef pcl::PointCloud<pcl::PointXYZI> PointCloud2Intensity;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud2RGB;



struct STR_LIDAR_CONFIG_INFO{
	Eigen::Matrix4f transformMartix; //自动初始化全零矩阵
    float reviseGroudHeight = 0.0f;
    float fLidar2Vehicle_Y = 0.0f;
    float fLidar2Vehicle_X = 0.0f;
    float rad = 0.0f;

    int rsairy_iInstallType = -99;
};


struct STR_ALL_LIDAR_CONFIG_INFO{
	STR_LIDAR_CONFIG_INFO toMainLidarInfo;
	STR_LIDAR_CONFIG_INFO toCarInfo;
	float fLidar2GridHeadingCorrectionDeg = 0.0f;  // 主雷达系(Grid) vs 融合IMU系 航向补偿(度)，由 fLidar2Vehicle_Heading 计算
};

#pragma pack(push, 1)
// ****************  udp发送用的               ****************//
struct Point2D {
    float x;
    float y;
};

typedef struct
{
	int id;
    float depth;          	// X轴长度
    float width;           	// Y轴长度
    float height;          	// Z轴高度
    float pos_x;           	// 中心点 X
    float pos_y;           	// 中心点 Y
    float pos_z;           // 中心点 Z
    Point2D corners[4]; 	//二维平面上的4个顶点

}S2obstacleBox;

typedef struct {
    unsigned long long timestamp;
    int return_val;
    int obs_num;
    S2obstacleBox obstacle[20];
    // int extents[8192];

} newS2AviodObject;

#pragma pack(pop)

}


#endif