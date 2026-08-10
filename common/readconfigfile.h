#pragma once
#ifndef _CONFIG_H_
#define _CONFIG_H_


#define MAX_COMBINE_PARA_NUM 12

#include <netinet/in.h>
#include <string>

// 配置文件读取错误码枚举
// 层级: application(-1) > lidar/rsairy子块(-2) > float/int等参数(-3)
// 全部读取成功返回 CFG_READ_SUCCESS(1)
enum ConfigReadError {
    CFG_READ_SUCCESS           =  1,  // 全部读取成功
    CFG_ERR_FILE_READ          = -1,  // application级别：config_read_file失败，配置文件本身无法读取
    CFG_ERR_SECTION_NOT_FOUND  = -2,  // 子块级别：application.lidar 或 application.rsairy 段未找到
    CFG_ERR_PARAM_READ         = -3,  // 参数级别：具体float/int等参数读取失败
};

typedef struct _STR_LIDAR_CONIFG
{
    int type;   // lidar_type
	int port;  // lidar_port
	int lidar_difop;
	int iPacketNum;
	int  isTransmit;
    double fGroundHeight;
    double fLeftCurb[6];
    double fRightCurb[6];
	double fComBinePara[MAX_COMBINE_PARA_NUM];
	double fLidar2Vehicle_Roll;
	double fLidar2Vehicle_Pitch;
    double fLidar2Vehicle_Heading;
    double fLidar2Vehicle_X;
    double fLidar2Vehicle_Y;
	char model[128];
	int isMulticast = 1;
	std::string sDestIP;
	in_addr_t sTransmitDestIP;

	double z_max_threshold;
	double fMinIntensity = 1.0;
	double car_x_min_threshold;
	double car_x_max_threshold;
	double car_y_min_threshold;
	double car_y_max_threshold;
	
	double fMaxDetectDistance;
	int iImageWidth;
	int iImageHeight;
	int iImageCenterX;
	int iImageCenterY;
	double fPixelScale;
	int iMinContourSize;
	int iMaxObjNum;
	double fNearRoi[4];
	double fFarRatio;
	double fNearMinHeightDiff;
	double fFarMinHeightDiff;
	double fAroundCarRoi[4];
	int iAroundCarPointsTh;
	double fCloudDensityTh;
	int iMinTrackNum;
	int iUseBackground;
	int iSaveObjects;

	int iInstallType;
	double fMaxFilterDis;
	double fMinFilterDis;
	int isFilterGround;
	double fFilterGroundRange;

	int isCreateGround;
	double fPitchError;
	double fHeightError;

	unsigned int uiPointNum;
	int iPacketCount;


	double flidar2imu_angle; //老的接口
	
}STR_LIDAR_CONFIG;


ConfigReadError readConfigFile(STR_LIDAR_CONFIG *strLidarConfig, const char* strConfigPath);

#endif
