#pragma once
#ifndef LEI_SHEN_DRIVER_H
#define LEI_SHEN_DRIVER_H

#include <iostream>
#include <stdlib.h>
#include <cmath>
#include <atomic>
#include <thread>
#include <pcl/filters/radius_outlier_removal.h>

#include "BaseDriver.h"
#include "CommonGroundDetection.h"
#include "LidarCurbDetection.h"



#include "GetLidarData_CH64w.h"
#include "GetLidarData_LS128.h"
#include "readconfigfile.h"
#include "ReadYamlFile.h"


// enum eLeishen_LidarType = {s_LS_CH64w, s_LS_128}


# define pcl_isfinite(x) std::isfinite(x)


namespace Lidar_Low_Detection
{

const int N_EDGE_SCAN = 64;     // 边缘部分64线雷达
const int N_MIDDLE_SCAN = 128;  // 中间部分128线雷达 (说明书写着10°)
const int Horizon_SCAN = 1500;  // 180° / 0.12 = 1500
const float ang_res_x = 0.12;   // 水平分辨率
const float ang_res_y = 0.6349; // 垂直分辨率 40 / (N_EDGE_SCAN - 1)
const float ang_bottom = -25;
//用于限制地面分割的雷达线范围，通常只使用靠近地面的几条线进行地面拟合。
const int groundScanInd[2] = {0,15};     //deepseek给出的建议：表示使用雷达最下方的 16 条线


class LeiShenDrive : public BaseDriver
{
public:

    LeiShenDrive(const SELF_DEBUG_CONFIG &config, const STR_ALL_LIDAR_CONFIG_INFO &allLidarTransInfo);
    ~LeiShenDrive();

    void Start();
    
    int Init();
    
    void Stop();

    void Free(); 

    void CallBackFunction(std::vector<MuchLidarData> LidarDataValue, int);	//回调函数，获取每一帧的数据
    void GetLidarType(const std::string LidarType);

    static void startGetDevSock(void *pth );  
    static void startGetDataSock(void *pth );

    unsigned long long GetPointCloudFromOnline(PointCloud2Intensity::Ptr &pOutputCloud);

private: 
    void InitSocket(); //把获取设备包的端口号getDevSock()和获取数据包的端口号getDataSock() 包装在一起 (没写)

    void GetDevSock();
    void GetDataSock();

    void SavePcd(const PointCloud2Intensity::Ptr &InputCloud, const unsigned long long &ullTime);

    void PointCloudTransform(std::vector<MuchLidarData> vTmpLidarData, PointCloud2Intensity::Ptr &pOutputCloud);

    void GroundRemoval(PointCloud2Intensity::Ptr &fullCloud);

    unsigned long long GetPointCloudFromPcd(PointCloud2Intensity::Ptr &pOutputCloud);

    void VoxelGridProcess(PointCloud2Intensity::Ptr &pOutputCloud);

    void ApplyRadiusOutlierFilter( PointCloud2Intensity::Ptr &InputCloud);

private: 
    unsigned long long m_ullCatchTimeStamp;
    unsigned long long m_ullUseTimeStamp;

    std::atomic<bool> m_running{false};
    std::thread m_SutengDriveThread;
    std::thread m_dataSockThread;
    std::thread m_devSockThread;

    int m_iDataSockFd;
    int m_iDevSockFd;
    int m_Sock;
    std::string m_sComputerIP;
    int m_iMsopPort; //数据端口
    int m_iDifopPort; //设备端口
    std::string m_sLeiShenType;

    Fun fun;
    Viewer *m_pViewer;
    // GetLidarData *m_pGetLidarData;  //没有命名空间时使用
    GetLidarData_CH64w *m_pGetLidarData;    //使用了命名空间时使用
    LidarCurbDetection *m_pLidarCurbDetection;
    CommonGroundDetection *m_pCommonGroundDetection;

    const SELF_DEBUG_CONFIG &m_stLSConfig;
    const STR_ALL_LIDAR_CONFIG_INFO &m_strAllLidarTransfromInfo;
    
    std::vector<MuchLidarData> m_vLidarData;
    std::mutex m_DataMutex;

    std::shared_ptr<pcl::visualization::PCLVisualizer> pcl_viewer = nullptr;

    std::shared_ptr<pcl::visualization::PCLVisualizer> ground_viewer = nullptr;
    
};
}

#endif