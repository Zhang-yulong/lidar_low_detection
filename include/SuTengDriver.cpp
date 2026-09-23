#include "SuTengDriver.h"
#include "static_obb_refinement.h"   // Phase 3-B: STATIC OBB geometry refinement
#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <unordered_map>
#include <unordered_set>

#include <atomic>
namespace Lidar_Low_Detection
{

std::atomic<unsigned long long> g_previousTimestamp(0);

bool test_EMX = true; //true false

SutengDriver::SutengDriver(){
    m_pElevationMapGroundFilter = nullptr;
}

SutengDriver::SutengDriver(const SELF_DEBUG_CONFIG &config, const STR_ALL_LIDAR_CONFIG_INFO &allLidarTransInfo)
    :m_ST_Config(&config), m_ST_AllLidarTransfromInfo(&allLidarTransInfo)
{

    m_pCommonGroundDetection = new CommonGroundDetection(*m_ST_Config);
    
}


SutengDriver::~SutengDriver(){
	Stop();
	Free();
}


int SutengDriver::Init(){


    send_fd = socket(AF_INET, SOCK_DGRAM, 0);
   	if (send_fd < 0) {
   		perror("create socket error");
        LOG_RAW(" ！！！[ERROR] Failed to create (发送UDP) socket. Please check the network configuration. \n");
   		return -1;
   	}

   	// 这行代码允许绑定处于 TIME_WAIT 状态的端口
   	int opt = 1;
   	if (setsockopt(send_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
   		perror("setsockopt SO_REUSEADDR failed");
        LOG_RAW(" ！！！[ERROR] Failed to set socket options (SO_REUSEADDR). Please check the network configuration. \n");

   		close(send_fd);
   		return -1;
   	}


    // 初始化 Elevation Map Ground Filter（使用默认参数，后续可从 YAML 加载）
    ElevationGridConfig grid_config;
    grid_config.roi_x_min = m_ST_Config->roi_x_min;
    // grid_config.roi_x_min = m_ST_Config->car_half_x + m_ST_Config->body_filter_x_threshold;
    grid_config.roi_x_max = m_ST_Config->roi_x_max;
    grid_config.roi_y_min = m_ST_Config->roi_y_min;
    grid_config.roi_y_max = m_ST_Config->roi_y_max;
    // Body 参数: 用于 BuildGrid 内部计算 effective ROI 边界，排除车身区域
    grid_config.car_half_x              = m_ST_Config->car_half_x;
    grid_config.car_half_y              = m_ST_Config->car_half_y;
    grid_config.body_filter_x_threshold = m_ST_Config->body_filter_x_threshold;
    grid_config.body_filter_y_threshold = m_ST_Config->body_filter_y_threshold;
    //判断地面参数
    grid_config.grid_resolution         = m_ST_Config->grid_resolution;
    grid_config.slope_threshold         = m_ST_Config->slope_threshold;
    grid_config.height_diff_threshold   = m_ST_Config->height_diff_threshold;
    grid_config.min_points_per_cell     = m_ST_Config->min_points_per_cell;
    //传感器高度
    grid_config.sensor_height = m_ST_AllLidarTransfromInfo->toCarInfo.reviseGroudHeight;
    grid_config.sensor_height_nominal = m_ST_AllLidarTransfromInfo->toCarInfo.reviseGroudHeight;
    //低矮障碍物范围
    grid_config.obstacle_top_height_min_near = m_ST_Config->obstacle_top_height_min_near;
    grid_config.obstacle_top_height_max_near = m_ST_Config->obstacle_top_height_max_near;
    grid_config.obstacle_top_height_min_far  = m_ST_Config->obstacle_top_height_min_far;
    grid_config.obstacle_top_height_max_far  = m_ST_Config->obstacle_top_height_max_far;
    grid_config.near_range_boundary          = m_ST_Config->near_range_boundary;
    


    /*// 在GetAllTransformConfigInfo函数中修正过高度值
    float mainLidarHeight = m_ST_AllLidarTransfromInfo->toCarInfo.GroudHeight;
    float airyLidarHeight = m_ST_AllLidarTransfromInfo->toMainLidarInfo; 
    if(mainLidarHeight < 0.0f && airyLidarHeight < 0.0f){
        grid_config.sensor_height = mainLidarHeight + airyLidarHeight;
    }
    else{
        LOG_RAW(" ！！！[ERROR] Lidar height is not set correctly in the configuration. Please check the YAML file. \n");
        return -1;
    }*/

    m_pElevationMapGroundFilter = std::make_unique<ElevationMapGroundFilter>(grid_config);

    // 创建 DebugViewer 并注入到 GroundFilter
    m_debugViewer = std::make_unique<DebugViewer>(*m_ST_Config);
    m_pElevationMapGroundFilter->SetDebugViewer(m_debugViewer.get());

    // ========================================================================
    // HDMap + 定位（迁移设计文档 Phase 1：Cluster 级软约束）
    //   启用条件（缺一不可，均可配置关闭，见迁移文档第 27/30 章）：
    //     mapFilterModel != 0   （HdmapFilter.mapFilterModel）
    //     localizationEnable != 0（Localization.enable）
    //     !pcdRunningModel       （pcd 单帧调试不启用）
    // ========================================================================
    m_hdmapEnabled = (m_ST_Config->mapFilterModel != 0) &&
                     (m_ST_Config->localizationEnable != 0) &&
                     !m_ST_Config->pcdRunningModel;

    if (m_hdmapEnabled)
    {
        // 启动时加载一次地图
        m_hdmapManager.loadMap(m_ST_Config->mapPath);

        HDMapFilter::Config fcfg;
        fcfg.enabled        = true;
        fcfg.filterMode     = m_ST_Config->hdmapFilterMode;
        fcfg.expandDistance = m_ST_Config->hdmapExpandDistance;
        fcfg.logEveryN      = m_ST_Config->hdmapLogEveryN;
        m_hdmapFilter.configure(fcfg);

        LOG_RAW("[HDMap] enabled (mapPath=%s)\n", m_ST_Config->mapPath.c_str());
    }
    else
    {
        m_hdmapFilter.configure(HDMapFilter::Config());
        LOG_RAW("[HDMap] disabled (mapFilterModel=%d localizationEnable=%d pcd=%d)\n",
               m_ST_Config->mapFilterModel, m_ST_Config->localizationEnable,
               m_ST_Config->pcdRunningModel);
    }

    if(m_ST_Config->pcapRunningModel == 1){
        m_rs_param.input_type = InputType::PCAP_FILE;
        m_rs_param.input_param.pcap_path = m_ST_Config->rs_pcapPath;  ///< Set the pcap file directory
        m_rs_param.input_param.pcap_rate = 1.0;
    }
    else{
        m_rs_param.input_type = InputType::ONLINE_LIDAR;
    }

    if(m_ST_Config->onlineModel){
        // m_rs_param.input_param.host_address = m_ST_Config->selfComputerIP;
        m_rs_param.input_param.group_address = m_ST_Config->groupIP;
    }
    
    m_rs_param.input_param.msop_port = m_ST_Config->msopPort;                          ///< Set the lidar msop port number, the default is 6699
    m_rs_param.input_param.difop_port = m_ST_Config->difopPort;
    // m_rs_param.lidar_type = LidarType::RSAIRY;
    m_rs_param.lidar_type= m_ST_Config->STlidarType;

    m_driver.regPointCloudCallback(
        [this](){return this->driverGetPointCloudFromCallerCallback();}, 
        [this](std::shared_ptr<PointCloudMsg> msg){this->driverReturnPointCloudToCallerCallback(msg);}
    ); ///< Register the point cloud callback functions
    
    m_driver.regExceptionCallback(
        [this](const robosense::lidar::Error& code){this->exceptionCallback(code);}
    );  ///< Register the exception callback function

    return 1;
}



int send_fd = -1;
unsigned int ip = inet_addr("224.0.0.1");
unsigned short port = 8192;


int SutengDriver::sendUdpMsg(int sk, unsigned char *pstrMsg, unsigned short usLen, unsigned int IpAddr, unsigned short usPort)
{
    ssize_t c;
    socklen_t addr_len;
    //stDBG_INFO dbg;
    struct sockaddr_in addr;

    // 将套接字设置为非阻塞模式，这样即使目标IP不可达，sendto调用也会立即返回
        int flags = fcntl(sk, F_GETFL, 0);
        fcntl(sk, F_SETFL, flags | O_NONBLOCK);

    // 忽略ICMP错误消息（仅适用于Linux），以防止内核处理“目的地不可达”等消息导致的延迟
        //int optval = 0;
        //setsockopt(sk, IPPROTO_IP, IP_RECVERR, &optval, sizeof(optval));

        memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = IpAddr;
    addr.sin_port = htons(usPort);
    addr_len = sizeof(addr);
    //printf("sendto start\n");
    struct timeval tv1, tv2;
    static int s_iCnt = 0;
    gettimeofday(&tv1, NULL);  //MSG_DONTWAIT
    c = sendto(sk, (void *)pstrMsg, (size_t)usLen, MSG_DONTWAIT, (struct sockaddr *)(&addr), addr_len);
    gettimeofday(&tv2, NULL);
    s_iCnt++;
    int ullNowTime = (tv2.tv_sec - tv1.tv_sec) * 1000 + (tv2.tv_usec - tv1.tv_usec) / 1000;
    if(ullNowTime > 5)
    {
        // LOG_RAW("sendUdpMsg ch64w now time = %d, s_iCnt = %d, c = %d, usPort = %d, errno = %d\n", ullNowTime, s_iCnt, c, usPort, errno);
        s_iCnt = 0;
    }
    else
    {
        //LOG_RAW("sendUdpMsg ch64w now time = %d, s_iCnt = %d, c = %d, usPort = %d, errno = %d\n", ullNowTime, s_iCnt, c, usPort, errno);
    }

    #if 1
    if (-1 == c)
    {
        static int s_icnt = 0;
        s_icnt++;
        if(s_icnt % 1000 == 0)
        {
            s_icnt = 0;
            // LOG_RAW("sendto udp port %u fail, sys reason %s\r\n", usPort, strerror(errno));
        }
        return c;
    // 		//continue;
    // 		//exit (0);
    // 		FILE *fp;
    // 		char error_str[] = "sendto udp port fail, sys reason";
    // 		fp = fopen("lidar_error_file.txt", "w");
    // 		fwrite(error_str, sizeof(error_str), 1, fp);
    // 		fclose(fp);
        //return 0;
        return -1;
    }
#endif
//API_LOG(LOG_INFO, "sendto udp port %u ok, sys reason %s\r\n", usPort, strerror(errno));
return c;
}




std::shared_ptr<PointCloudMsg> SutengDriver::driverGetPointCloudFromCallerCallback(void)
{
  // Note: This callback function runs in the packet-parsing/point-cloud-constructing thread of the driver, 
  //       so please DO NOT do time-consuming task here.
  std::shared_ptr<PointCloudMsg> msg = free_cloud_queue.pop();
  if (msg.get() != NULL)
  {
    return msg;
  }

  return std::make_shared<PointCloudMsg>();
}


void SutengDriver::driverReturnPointCloudToCallerCallback(std::shared_ptr<PointCloudMsg> msg)
{
  // Note: This callback function runs in the packet-parsing/point-cloud-constructing thread of the driver, 
  //       so please DO NOT do time-consuming task here. Instead, process it in caller's own thread. (see processCloud() below)
  stuffed_cloud_queue.push(msg);
}


void SutengDriver::exceptionCallback(const robosense::lidar::Error & code)
{
  // Note: This callback function runs in the packet-receving and packet-parsing/point-cloud_constructing thread of the driver, 
  //       so please DO NOT do time-consuming task here.
  RS_WARNING << code.toString() << RS_REND;
}


bool SutengDriver::loadPointCloud(const std::string& file_path, PointCloud2Intensity::Ptr& cloud) {
    // 使用 loadPCDFile 读取文件
    // 注意：传入的是 *cloud，即解引用后的对象
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(file_path, *cloud) == -1) {
        printf("Error reading PCD file: %s\n", file_path.c_str());
        return false;
    }
    return true;
}

void SutengDriver::PointCloudTransform(PointCloud2Intensity::Ptr &pInputCloud, const Eigen::Matrix4f &R_Combined, PointCloud2Intensity::Ptr &pOutputCloud){

#if 1
    if (!pOutputCloud){
        pOutputCloud.reset(new PointCloud2Intensity);}

    pOutputCloud->clear();
    pOutputCloud->reserve(pInputCloud->size());

    // ======= 提前把矩阵元素取出来，避免每个点访问 Eigen =======
    const float r00 = R_Combined(0,0), r01 = R_Combined(0,1), r02 = R_Combined(0,2), tx = R_Combined(0,3);
    const float r10 = R_Combined(1,0), r11 = R_Combined(1,1), r12 = R_Combined(1,2), ty = R_Combined(1,3);
    const float r20 = R_Combined(2,0), r21 = R_Combined(2,1), r22 = R_Combined(2,2), tz = R_Combined(2,3);


    // ======= Body Filter 参数 =======
    // const float expand_x =
    //     m_ST_Config->body_filter_x_threshold +
    //     m_ST_Config->car_half_x;

    // const float expand_y =
    //     -(m_ST_Config->body_filter_y_threshold +
    //       m_ST_Config->car_half_y);
    const float expand_x =
        m_ST_Config->body_filter_x_threshold +
        m_ST_Config->car_half_x;

    const float left_limit =
        m_ST_Config->car_half_y +
        m_ST_Config->body_filter_y_threshold;

    const float right_limit =
        -(m_ST_Config->car_half_y +
        m_ST_Config->body_filter_y_threshold);

    // ======= ROI 参数 =======
    const float roi_x_max = m_ST_Config->roi_x_max;
    const float roi_y_min = m_ST_Config->roi_y_min;
    const float roi_y_max = m_ST_Config->roi_y_max;

    // auto t0 = std::chrono::steady_clock::now();

    // std::cout
    // << "expand_x = " << expand_x
    // << std::endl;

    // std::cout
    // << "expand_y = " << expand_y
    // << std::endl;


    for(const auto &src : pInputCloud->points)
    {
        pcl::PointXYZI dst;

        //-------------------------
        // 1. 坐标变换
        //-------------------------
        if(m_ST_Config->pcapRunningModel || m_ST_Config->onlineModel){
            dst.x = r00*src.x + r01*src.y + r02*src.z + tx;
            dst.y = r10*src.x + r11*src.y + r12*src.z + ty;
            dst.z = r20*src.x + r21*src.y + r22*src.z + tz;

            dst.intensity = src.intensity;
        
        
            // X方向
            if (dst.x <= expand_x){
                continue;
            }

            if (dst.x >= roi_x_max){
                continue;
            }
                
            // Y方向
            if (dst.y <= roi_y_min){
                continue;
            }

            if (dst.y >= roi_y_max){
                continue;
            }
            
        }
        else{
            dst.x = src.x;
            dst.y = src.y;
            dst.z = src.z;
            // if (test_EMX)
            // {
            
            //     if(src.z > -1.265){
            //         dst.z = src.z;
            //     }
            //     else{
            //         dst.z = -1.265;
            //     }
            // }
            dst.intensity = src.intensity;
        }    
       

        pOutputCloud->push_back(dst);
    }

    // auto t1 = std::chrono::steady_clock::now();

    // std::cout
    //     << "Transform + Filter : "
    //     << std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0).count()
    //     << " ms\n";

    pOutputCloud->width = pOutputCloud->size();
    pOutputCloud->height = 1;
    pOutputCloud->is_dense = false;

#endif



//----------下面也是正确的，换了种写法，在上面

#if 0
    // 确保输出点云指针已分配内存（防止空指针异常）
    if (!pOutputCloud) {
        pOutputCloud = PointCloud2Intensity::Ptr(new PointCloud2Intensity);
    }

    auto t0 = std::chrono::steady_clock::now();
    // 调用 PCL 内置函数执行点云变换，该函数会将 pInputCloud 中的每个点左乘 R 矩阵
    // Eigen::Matrix4f R_ToMainLidar = m_ST_AllLidarTransfromInfo->toMainLidarInfo.transformMartix;
    PointCloud2Intensity::Ptr pBeforefilterPointCloud(new PointCloud2Intensity);
    if(m_ST_Config->pcapRunningModel || m_ST_Config->onlineModel){
        // pcl::transformPointCloud(*pInputCloud, *pBeforefilterPointCloud, R_ToMainLidar);
        pcl::transformPointCloud(*pInputCloud, *pBeforefilterPointCloud, R_Combined);
        // pcl::transformPointCloud(*pInputCloud, *pOutputCloud, R_Combined);
    }
    auto t1 = std::chrono::steady_clock::now();

    // pBeforefilterPointCloud->clear();
    // pBeforefilterPointCloud->resize(pInputCloud->size());
    // const auto& T = R_Combined;
    // for(size_t i=0;i<pInputCloud->size();++i)
    // {
    //     const auto& s = pInputCloud->points[i];
    //     auto& d = pBeforefilterPointCloud->points[i];

    //     d.x = T(0,0)*s.x + T(0,1)*s.y + T(0,2)*s.z + T(0,3);
    //     d.y = T(1,0)*s.x + T(1,1)*s.y + T(1,2)*s.z + T(1,3);
    //     d.z = T(2,0)*s.x + T(2,1)*s.y + T(2,2)*s.z + T(2,3);

    //     d.intensity = s.intensity;
    // }
    // auto t4 = std::chrono::steady_clock::now();
    //  std::cout
    // << "自己写转换矩阵       : "
    // << std::chrono::duration_cast<std::chrono::milliseconds>(t4-t1).count()
    // << " ms\n";


/* 都是配置文件里body_filter_y_threshold 之前写的外扩时填负值。内缩时填正值
    // ========== Step 1: 剔除车身区域（用可配置参数） ==========
    // 保留逻辑：x > body_filter_x_threshold OR y < body_filter_y_threshold
    // 雷达装在车右侧，旋转变换到车体后：x+→车头，y+→车左侧
    // {
    //     float body_x = m_ST_Config->body_filter_x_threshold;
    //     float body_y = m_ST_Config->body_filter_y_threshold;

    //     pcl::ConditionOr<PointCloud2Intensity::PointType>::Ptr body_cond(
    //         new pcl::ConditionOr<PointCloud2Intensity::PointType>());

    //     body_cond->addComparison(pcl::FieldComparison<PointCloud2Intensity::PointType>::ConstPtr(
    //         new pcl::FieldComparison<PointCloud2Intensity::PointType>("x", pcl::ComparisonOps::GT, body_x)));

    //     body_cond->addComparison(pcl::FieldComparison<PointCloud2Intensity::PointType>::ConstPtr(
    //         new pcl::FieldComparison<PointCloud2Intensity::PointType>("y", pcl::ComparisonOps::LT, body_y)));

    //     pcl::ConditionalRemoval<PointCloud2Intensity::PointType> body_removal;
    //     body_removal.setCondition(body_cond);
    //     body_removal.setKeepOrganized(false);

    //     if(m_ST_Config->pcapRunningModel || m_ST_Config->onlineModel)
    //         body_removal.setInputCloud(pBeforefilterPointCloud);
    //     else if(m_ST_Config->pcdRunningModel)
    //         body_removal.setInputCloud(pInputCloud);

    //     body_removal.filter(*pOutputCloud);  // 中间结果写入 pOutputCloud
    // }

    // {
    //     float expand_x = m_ST_Config->body_filter_x_threshold + m_ST_Config->car_half_x;
    //     //y正方向→车左侧
    //     float expand_y;
    //     if(m_ST_Config->body_filter_y_threshold < 0.0)
    //         expand_y = m_ST_Config->body_filter_y_threshold - m_ST_Config->car_half_y;
    //     else if(m_ST_Config->body_filter_y_threshold > 0.0)
    //         expand_y = m_ST_Config->car_half_y - m_ST_Config->body_filter_y_threshold;
    //     else
    //         expand_y = m_ST_Config->car_half_y;

    //     pcl::ConditionOr<PointCloud2Intensity::PointType>::Ptr body_cond(
    //         new pcl::ConditionOr<PointCloud2Intensity::PointType>());

    //     body_cond->addComparison(pcl::FieldComparison<PointCloud2Intensity::PointType>::ConstPtr(
    //         new pcl::FieldComparison<PointCloud2Intensity::PointType>("x", pcl::ComparisonOps::GT, expand_x)));

    //     body_cond->addComparison(pcl::FieldComparison<PointCloud2Intensity::PointType>::ConstPtr(
    //         // new pcl::FieldComparison<PointCloud2Intensity::PointType>("y", pcl::ComparisonOps::LT, body_y)));
    //         new pcl::FieldComparison<PointCloud2Intensity::PointType>("y", pcl::ComparisonOps::LT, expand_y)));
    //     pcl::ConditionalRemoval<PointCloud2Intensity::PointType> body_removal;
    //     body_removal.setCondition(body_cond);
    //     body_removal.setKeepOrganized(false);

    //     if(m_ST_Config->pcapRunningModel || m_ST_Config->onlineModel)
    //         body_removal.setInputCloud(pBeforefilterPointCloud);
    //     else if(m_ST_Config->pcdRunningModel)
    //         body_removal.setInputCloud(pInputCloud);

    //     body_removal.filter(*pOutputCloud);  // 中间结果写入 pOutputCloud
    // }
*/

    {
        
        float expand_x = m_ST_Config->body_filter_x_threshold + m_ST_Config->car_half_x;
        //y正方向→车左侧,因为使用右边airy，要剔除右边部分的点云，加了负号
        float expand_y = -(m_ST_Config->body_filter_y_threshold + m_ST_Config->car_half_y);
       
        pcl::ConditionOr<PointCloud2Intensity::PointType>::Ptr body_cond(
            new pcl::ConditionOr<PointCloud2Intensity::PointType>());

        body_cond->addComparison(pcl::FieldComparison<PointCloud2Intensity::PointType>::ConstPtr(
            new pcl::FieldComparison<PointCloud2Intensity::PointType>("x", pcl::ComparisonOps::GT, expand_x)));

        body_cond->addComparison(pcl::FieldComparison<PointCloud2Intensity::PointType>::ConstPtr(
            // new pcl::FieldComparison<PointCloud2Intensity::PointType>("y", pcl::ComparisonOps::LT, body_y)));
            new pcl::FieldComparison<PointCloud2Intensity::PointType>("y", pcl::ComparisonOps::LT, expand_y)));
        pcl::ConditionalRemoval<PointCloud2Intensity::PointType> body_removal;
        body_removal.setCondition(body_cond);
        body_removal.setKeepOrganized(false);

        if(m_ST_Config->pcapRunningModel || m_ST_Config->onlineModel)
            body_removal.setInputCloud(pBeforefilterPointCloud);
        else if(m_ST_Config->pcdRunningModel)
            body_removal.setInputCloud(pInputCloud);

        body_removal.filter(*pOutputCloud);  // 中间结果写入 pOutputCloud
    }
    auto t2 = std::chrono::steady_clock::now();


    // ========== Step 2: ROI 区域裁剪 ==========
    // 保留：x < roi_x_max AND y > roi_y_min AND y < roi_y_max
    {
        pcl::ConditionAnd<PointCloud2Intensity::PointType>::Ptr roi_cond(
            new pcl::ConditionAnd<PointCloud2Intensity::PointType>());

        roi_cond->addComparison(pcl::FieldComparison<PointCloud2Intensity::PointType>::ConstPtr(
            new pcl::FieldComparison<PointCloud2Intensity::PointType>("x", pcl::ComparisonOps::LT,
                m_ST_Config->roi_x_max)));

        roi_cond->addComparison(pcl::FieldComparison<PointCloud2Intensity::PointType>::ConstPtr(
            new pcl::FieldComparison<PointCloud2Intensity::PointType>("y", pcl::ComparisonOps::GT,
                m_ST_Config->roi_y_min)));

        roi_cond->addComparison(pcl::FieldComparison<PointCloud2Intensity::PointType>::ConstPtr(
            new pcl::FieldComparison<PointCloud2Intensity::PointType>("y", pcl::ComparisonOps::LT,
                m_ST_Config->roi_y_max)));

        PointCloud2Intensity::Ptr temp(new PointCloud2Intensity);
        pcl::ConditionalRemoval<PointCloud2Intensity::PointType> roi_removal;
        roi_removal.setCondition(roi_cond);
        roi_removal.setInputCloud(pOutputCloud);  // 以 body removal 结果为输入
        roi_removal.setKeepOrganized(false);
        roi_removal.filter(*temp);
        pOutputCloud->swap(*temp);
    }
    auto t3 = std::chrono::steady_clock::now();

        std::cout
    << "transform : "
    << std::chrono::duration_cast<std::chrono::milliseconds>(t1-t0).count()
    << " ms\n";

    std::cout
    << "body      : "
    << std::chrono::duration_cast<std::chrono::milliseconds>(t2-t1).count()
    << " ms\n";

    std::cout
    << "roi       : "
    << std::chrono::duration_cast<std::chrono::milliseconds>(t3-t2).count()
    << " ms\n";

   
    // ========== 两步滤波结束 ==========

    // LOG_RAW(" @@@后的点云数量: %zu\n", pOutputCloud->points.size());
#endif
}


void SutengDriver::SavePcd(const PointCloud2Intensity::Ptr &InputCloud)
{

//	PointCloud::Ptr filteredCloud(new PointCloud());
	// 过滤点云，只保留强度大于给定阈值的点
//    for (const auto& point : InputCloud->points)
//    {
//        if (point.intensity > m_stLSConfig.savePcdReflection && point.z < 0.0)  // 只保留强度大于阈值的点
//        {
//            pcl::PointXYZI filteredPoint;
//            filteredPoint.x = point.x;
//            filteredPoint.y = point.y;
//            filteredPoint.z = point.z;
//            filteredPoint.intensity = point.intensity;
//
//            // 将过滤后的点添加到新的点云中
//            filteredCloud->points.push_back(filteredPoint);
//        }
//    }

    // 如果过滤后的点云为空，则不保存
    if (InputCloud->empty())
    {
        std::cout << "No points with intensity above threshold. Skipping saving." << std::endl;
        return;
    }

//	// 设置新的点云的高度和宽度
//    filteredCloud->height = filteredCloud->points.size();
//    filteredCloud->width = 1;
//    filteredCloud->is_dense = false;

//  	InputCloud->height = InputCloud->points.size();
//  	InputCloud->width = 1;
//  	InputCloud->is_dense = false;
	static int i = 0;
	i++;
	// if(i % 3 != 0)
	// {
	// 	return;
	// }

	// struct timeval tNow;
	// gettimeofday(&tNow, NULL);
	// unsigned long long ullTimestamp = ((unsigned long long)tNow.tv_sec)*1000 + ((unsigned long long)tNow.tv_usec)/1000;
	// printf("pop ullTimestamp:%ld\n", ullTimestamp);
	// // gettimeofday(&tStart, NULL);
	char pchFileName[128];
	bzero(pchFileName, sizeof (pchFileName));
//	sprintf(pchFileName, "/home/zyl/echiev_lidar_curb_detection/log/pcd/%lld.pcd", ullTime);
	sprintf(pchFileName, "/home/zyl/echiev_low_lidar_detection/log/init.pcd");
//	pcl::io::savePCDFileASCII (pchFileName, *InputCloud);
	pcl::io::savePCDFileBinary(pchFileName, *InputCloud);
	// pcl::io::savePCDFileASCII (pchFileName, *filteredCloud);
}



#if 0 // ---- 可视化已迁移至 main.cpp ----
// ============================================================
// 绘制车身矩形轮廓（z=0 平面，只绘制一次）
// ============================================================
void SutengDriver::DrawCarBodyOutlines()
{
    if (m_carBodyDrawn || !ground_viewer) 
        return;

    m_carBodyDrawn = true;

    float orig_hx = m_ST_Config->car_half_x;
    float orig_hy = m_ST_Config->car_half_y;
    float hz = m_ST_AllLidarTransfromInfo->toCarInfo.reviseGroudHeight;
    float exp_hx  = m_ST_Config->body_filter_x_threshold + m_ST_Config->car_half_x;
    float exp_hy = m_ST_Config->body_filter_y_threshold + m_ST_Config->car_half_y;
    /*都是配置文件里body_filter_y_threshold 之前写的外扩时填负值。内缩时填正值
    if(m_ST_Config->body_filter_y_threshold < 0.0f)
        exp_hy = std::abs(m_ST_Config->body_filter_y_threshold) + m_ST_Config->car_half_y;
    else if(m_ST_Config->body_filter_y_threshold > 0.0f)
        exp_hy = -m_ST_Config->body_filter_y_threshold + m_ST_Config->car_half_y;
    else
        exp_hy = m_ST_Config->car_half_y;
    */
    // 辅助 lambda：画一个水平矩形框（z=GroudHeight）
    auto drawRect = [&](float hx, float hy, float hz, double r, double g, double b, const std::string& prefix)
    {
        pcl::PointXYZ p0(-hx, -hy, hz);
        pcl::PointXYZ p1( hx, -hy, hz);
        pcl::PointXYZ p2( hx,  hy, hz);
        pcl::PointXYZ p3(-hx,  hy, hz);

        ground_viewer->addLine(p0, p1, r, g, b, prefix + "_L0");
        ground_viewer->addLine(p1, p2, r, g, b, prefix + "_L1");
        ground_viewer->addLine(p2, p3, r, g, b, prefix + "_L2");
        ground_viewer->addLine(p3, p0, r, g, b, prefix + "_L3");
    };

    // 原始车身 —— 红色
    drawRect(orig_hx, orig_hy, hz, 1.0, 0.0, 0.0, "car_orig");
    // 外扩车身 —— 白色
    drawRect(exp_hx, exp_hy, hz, 1.0, 1.0, 1.0, "car_exp");
}
#endif // ---- 可视化已迁移至 main.cpp

#if 0 // ---- 可视化已迁移至 main.cpp
// ============================================================
// 共用可视化：初始化 ground_viewer
// ============================================================
void SutengDriver::InitGroundViewer()
{
    if (ground_viewer) return;  // 已初始化

    ground_viewer = std::make_shared<pcl::visualization::PCLVisualizer>("GroundFilterViewer");
    ground_viewer->setBackgroundColor(0.05, 0.05, 0.05);
    ground_viewer->addCoordinateSystem(2.0);
}
#endif // ----

#if 0 // ---- 可视化已迁移至 main.cpp
// ============================================================
// 共用可视化：更新地面/障碍物点云显示
// ============================================================
void SutengDriver::UpdateGroundViewer(const PointCloud2Intensity::Ptr &pGroundCloud,
                                      const PointCloud2Intensity::Ptr &pObstacleCloud,
                                      const std::vector<GridCluster> &clusters)
{
    if (!ground_viewer) return;

    DrawCarBodyOutlines();  // 车身矩形（只绘制一次）

    // ---- 地面点云 (绿色) ----
    {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> ground_color(
            pGroundCloud, 0, 255, 0);
        if (!ground_viewer->updatePointCloud<pcl::PointXYZI>(pGroundCloud, ground_color, "ground"))
        {
            ground_viewer->addPointCloud<pcl::PointXYZI>(pGroundCloud, ground_color, "ground");
            ground_viewer->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "ground");
        }
    }

    // ---- 障碍物点云 (红色) ----
    {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> obstacle_color(
            pObstacleCloud, 255, 0, 0);
        if (!ground_viewer->updatePointCloud<pcl::PointXYZI>(pObstacleCloud, obstacle_color, "obstacle"))
        {
            ground_viewer->addPointCloud<pcl::PointXYZI>(pObstacleCloud, obstacle_color, "obstacle");
            ground_viewer->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "obstacle");
        }
    }

    // ---- Cluster 包围盒可视化 ----
    // 先清理上一帧残留的 shape（按计数移除）
    for (int i = 0; i < m_lastClusterCount; ++i)
    {
        ground_viewer->removeShape("cluster_box_" + std::to_string(i));
        ground_viewer->removeShape("cluster_center_" + std::to_string(i));
    }

    // 绘制当前帧的 cluster
    for (size_t i = 0; i < clusters.size(); ++i)
    {
        const auto& c = clusters[i];
        std::string box_id    = "cluster_box_"    + std::to_string(i);
        std::string center_id = "cluster_center_" + std::to_string(i);

        // 白色线框包围盒
        ground_viewer->addCube(
            c.min_x, c.max_x, c.min_y, c.max_y, c.min_z, c.max_z,
            1.0, 1.0, 1.0,  // RGB: 白色
            box_id);
        ground_viewer->setShapeRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
            pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
            box_id);

        // 黄色球标记中心
        ground_viewer->addSphere(
            pcl::PointXYZ(c.center_x, c.center_y, c.center_z),
            0.15,  // 半径
            1.0, 1.0, 0.0,  // RGB: 黄色
            center_id);
    }

    m_lastClusterCount = static_cast<int>(clusters.size());
}

// ============================================================
// 共用可视化重载：TrackedObstacle 版本（显示 tracker 稳定 ID）
// ============================================================
void SutengDriver::UpdateGroundViewer(const PointCloud2Intensity::Ptr &pGroundCloud,
                                      const PointCloud2Intensity::Ptr &pObstacleCloud,
                                      const std::vector<TrackedObstacle> &tracks)
{
    if (!ground_viewer) return;

    DrawCarBodyOutlines();  // 车身矩形（只绘制一次）

    // ---- 地面点云 (绿色) ----
    {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> ground_color(
            pGroundCloud, 0, 255, 0);
        if (!ground_viewer->updatePointCloud<pcl::PointXYZI>(pGroundCloud, ground_color, "ground"))
        {
            ground_viewer->addPointCloud<pcl::PointXYZI>(pGroundCloud, ground_color, "ground");
            ground_viewer->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "ground");
        }
    }

    // ---- 障碍物点云 (红色) ----
    {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> obstacle_color(
            pObstacleCloud, 255, 0, 0);
        if (!ground_viewer->updatePointCloud<pcl::PointXYZI>(pObstacleCloud, obstacle_color, "obstacle"))
        {
            ground_viewer->addPointCloud<pcl::PointXYZI>(pObstacleCloud, obstacle_color, "obstacle");
            ground_viewer->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "obstacle");
        }
    }

    // ---- Tracked Obstacle 包围盒 + 稳定 ID 可视化 ----
    // 先清理上一帧残留的 shape
    for (int i = 0; i < m_lastTrackCount; ++i)
    {
        ground_viewer->removeShape("track_box_" + std::to_string(i));
        ground_viewer->removeShape("track_center_" + std::to_string(i));
        ground_viewer->removeText3D("track_id_" + std::to_string(i));
    }

    // 绘制当前帧的 tracked obstacles
    for (size_t i = 0; i < tracks.size(); ++i)
    {
        const auto& t = tracks[i];

        // 计算 Z 范围（pos_z 为中心，上下各 half height）
        float half_h = t.height * 0.5f;
        float min_z = t.pos_z - half_h;
        float max_z = t.pos_z + half_h;

        // 从 corners[4] 计算 X/Y AABB
        float min_x = t.corners[0].x, max_x = t.corners[0].x;
        float min_y = t.corners[0].y, max_y = t.corners[0].y;
        for (int c = 1; c < 4; ++c)
        {
            if (t.corners[c].x < min_x) min_x = t.corners[c].x;
            if (t.corners[c].x > max_x) max_x = t.corners[c].x;
            if (t.corners[c].y < min_y) min_y = t.corners[c].y;
            if (t.corners[c].y > max_y) max_y = t.corners[c].y;
        }

        std::string box_id    = "track_box_"    + std::to_string(i);
        std::string center_id = "track_center_" + std::to_string(i);
        std::string text_id   = "track_id_"     + std::to_string(i);

        // 白色线框包围盒（与 cluster 白色区分）
        ground_viewer->addCube(
            min_x, max_x, min_y, max_y, min_z, max_z,
            1.0, 1.0, 1.0,  // RGB: 青色
            box_id);
        ground_viewer->setShapeRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
            pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
            box_id);

        // 橙色球标记中心
        ground_viewer->addSphere(
            pcl::PointXYZ(t.pos_x, t.pos_y, t.pos_z),
            0.2,  // 半径（稍大以区分 cluster）
            1.0, 0.5, 0.0,  // RGB: 橙色
            center_id);

        // // 白色文本显示稳定 ID
        // ground_viewer->addText3D(
        //     std::to_string(t.id),
        //     pcl::PointXYZ(t.pos_x, t.pos_y, max_z + 0.3),
        //     0.3,  // 文字高度
        //     1.0, 1.0, 1.0,  // RGB: 白色
        //     text_id);
    }

    m_lastTrackCount = static_cast<int>(tracks.size());
}

// ============================================================
// 共用可视化：非阻塞 spin 一次，返回窗口是否仍然打开
// ============================================================
bool SutengDriver::SpinGroundViewerOnce()
{
    if (!ground_viewer || ground_viewer->wasStopped())
        return false;
    ground_viewer->spinOnce(10);  // 10ms，与 while 循环的帧率配合
    return !ground_viewer->wasStopped();
}
#endif // ---- 可视化已迁移至 main.cpp



void SutengDriver::ProcessPcapCloud(){

    PointCloud2Intensity::Ptr pInputCloud(new PointCloud2Intensity);
    PointCloud2Intensity::Ptr pFilteredPointCloud(new PointCloud2Intensity);
    PointCloud2Intensity::Ptr pGroundCloud(new PointCloud2Intensity);
    PointCloud2Intensity::Ptr pObstacleCloud(new PointCloud2Intensity);

    //定义 的旋转矩阵 ，因为转到车体后，是y朝前，x朝右。（！！！！周洋说还需要x，y互换
    Eigen::Matrix4f R_X_Exchange_Y = Eigen::Matrix4f::Identity();
    R_X_Exchange_Y(0, 0) =  0.0f;
    R_X_Exchange_Y(0, 1) = 1.0f;
    R_X_Exchange_Y(1, 0) =  1.0f;
    R_X_Exchange_Y(1, 1) =  0.0f;

    Eigen::Matrix4f R_CombinedTransMatrix = R_X_Exchange_Y * (m_ST_AllLidarTransfromInfo->toCarInfo.transformMartix 
                            * m_ST_AllLidarTransfromInfo->toMainLidarInfo.transformMartix);
    // Eigen::Matrix4f R_CombinedTransMatrix = (m_ST_AllLidarTransfromInfo->toCarInfo.transformMartix 
    //                         * m_ST_AllLidarTransfromInfo->toMainLidarInfo.transformMartix);

    if(test_EMX){
        // Eigen::Matrix4f R_CombinedTransMatrix = Eigen::Matrix4f::Identity();
    }

    //定位+地图容器
    bool have_pose = false;
    LocalizationManager::Pose loc_pose;
    std::vector<std::vector<STR_POINT2F>> hdmap_polygons;

    while(m_running)
    {  
        unsigned long long titleTime_ms = GetCurrentTimestamp();
        std::string titleTime_string = timestampToTimeString(titleTime_ms);
 
        auto t_start = std::chrono::steady_clock::now();

        //── DebugViewer: 每帧递增计数 ──
        if (m_debugViewer)
        {
            m_debugViewer->NewFrame();
            test_Frame_count++;
        }


        std::shared_ptr<PointCloudMsg> msg = stuffed_cloud_queue.popWait();
        if (msg.get() == NULL)
        {
            continue;
        }

        auto t_end_1_ = std::chrono::steady_clock::now();
        auto duration_1 = std::chrono::duration_cast<std::chrono::milliseconds>(t_end_1_ - t_start);
         
      
        // std::cout <<" ----------------------" << std::endl;
        // RS_MSG << "msg: " << msg->seq << " time: " << std::to_string(msg->timestamp) << " point cloud size: " << msg->points.size() << RS_REND;
        
        unsigned long long rec_timestamp_ms = static_cast<unsigned long long>(msg->timestamp * 1000.0);

        
        LOG_RAW("--------------------------------------------\n");
        LOG_RAW("[%s][%lld]Loop Start\n", titleTime_string.c_str(), titleTime_ms);

        if(m_ST_Config->pcapRunningModel){
            LOG_RAW("[pcap rec time]: %lld \n" ,rec_timestamp_ms);
        }
        
        if(m_debugViewer->GetFrameCountValue() ==1){
            g_previousTimestamp.store(rec_timestamp_ms, std::memory_order_relaxed);
            LOG_RAW("第一次循环，跳过！！！！\n");
            continue;
        }

        pInputCloud->clear();
        pFilteredPointCloud->clear();
        pGroundCloud->clear();
        pObstacleCloud->clear();

        for(size_t i = 0; i < msg->points.size(); i++)
        {
            pcl::PointXYZI temp;
            
            //zy偷懒，这里仅判断大于0不行
            if( (m_ST_Config->STlidarType == robosense::lidar::LidarType::RSAIRY) && 
                (m_ST_AllLidarTransfromInfo->toMainLidarInfo.rsairy_iInstallType > 0)){
                //海南T05右边airy
                temp.x = msg->points[i].x;
                temp.y = -msg->points[i].y;
                temp.z = -msg->points[i].z;
            }
            else if( (m_ST_Config->STlidarType == robosense::lidar::LidarType::RSAIRY) &&
                    (m_ST_AllLidarTransfromInfo->toMainLidarInfo.rsairy_iInstallType == -3)){
                //阜阳T1后面airy
                temp.z = -msg->points[i].x;
                temp.y = msg->points[i].y;
                temp.x = msg->points[i].z;
            }
            else{
                temp.x = msg->points[i].x;
                temp.y = msg->points[i].y;
                temp.z = msg->points[i].z;
            }
            temp.intensity = msg->points[i].intensity;
            pInputCloud->push_back(temp);
        }
            

        PointCloudTransform(pInputCloud, R_CombinedTransMatrix, pFilteredPointCloud);

        auto t_end_2_ = std::chrono::steady_clock::now();
        auto duration_2 = std::chrono::duration_cast<std::chrono::milliseconds>(t_end_2_ - t_end_1_);



        //一些测试发现，上坡时，会出现把低矮的障碍物当作地面，考虑废弃
        // std::pair<PointCloud2Intensity::Ptr, PointCloud2Intensity::Ptr> ground_obstacle_clouds = m_pCommonGroundDetection->CommonGroundSegmentStart(pFilteredPointCloud, 0);
        // PointCloud2Intensity::Ptr p_comm_no_Ground = ground_obstacle_clouds.first;
        // PointCloud2Intensity::Ptr p_comm_ground = ground_obstacle_clouds.second;

        auto t_end_3_ = std::chrono::steady_clock::now();
        auto duration_3 = std::chrono::duration_cast<std::chrono::milliseconds>(t_end_3_ - t_end_2_);

        // bool success = m_pElevationMapGroundFilter->Process(p_comm_no_Ground, pGroundCloud, pObstacleCloud);


        std::vector<GridCluster> outputClusters;
        // bool success = m_pElevationMapGroundFilter->ProcessWithObstacleDetection(p_comm_no_Ground, pGroundCloud, pObstacleCloud, outputClusters);
        bool success = m_pElevationMapGroundFilter->ProcessWithObstacleDetection(pFilteredPointCloud, pGroundCloud, pObstacleCloud, outputClusters);

        auto t_end_4_ = std::chrono::steady_clock::now();
        auto duration_4 = std::chrono::duration_cast<std::chrono::milliseconds>(t_end_4_ - t_end_3_);


        // ── 定位读取：每帧统一读取一次（独立于 HDMap 开关，Historical Feedback 也需要）──
        // getPose() 返回 has_data（是否收到过定位），loc_pose.valid 才是本帧新鲜有效标志。
        {
            const bool got_pose = LocalizationManager::instance().getPose(loc_pose);
            have_pose = got_pose && loc_pose.valid;
        }

        // ── HDMap 过滤：Cluster 级软约束（Cluster 生成后、Tracker 之前）──
        // 迁移设计文档 Phase 1：只打标签（in_road/map_valid/map_confidence），不删除 Cluster。
        // 定位无效 / 地图未加载 / 查询失败 → HDMapFilter 内部自动降级为"保留全部"。
        if (m_hdmapEnabled)
        {
            if (have_pose)
            {
                m_hdmapManager.buildDrivablePolygons(loc_pose.x, loc_pose.y, hdmap_polygons);
            }

            // 每帧定位诊断：点云帧墙钟时间戳 vs 定位新鲜度（墙钟）与定位模块时间戳
            // loc_age_ms = 当前墙钟 - 最近一次收到定位的墙钟；定位100Hz时一般 ≤10ms。
            // 注意：loc_module_ts(ullTimestampModule) 与 cloud_ts_ms 不是同一时间基准，
            // 要看"拿到一帧与拿到一帧定位的时间差"应以 loc_age_ms 为准。
            LOG_RAW("[LocDiag] cloud_ts_ms=%llu loc_valid=%d loc_module_ts=%llu loc_age_ms=%llu loc_n=%llu\n",
                    rec_timestamp_ms,
                    (int)loc_pose.valid,
                    (unsigned long long)loc_pose.timestamp_ms,
                    (unsigned long long)LocalizationManager::instance().lastUpdateAgeMs(),
                    (unsigned long long)LocalizationManager::instance().receivedCount());

            m_hdmapFilter.filterClusters(outputClusters, loc_pose, m_hdmapManager, rec_timestamp_ms);
        }

        m_debugViewer->DrawClusterOverlay(*pGroundCloud, *pObstacleCloud,outputClusters,m_pElevationMapGroundFilter->GetConfig(),hdmap_polygons,loc_pose);

    

        // ── 跟踪器：为障碍物分配稳定的跨帧 ID（从 9999 起始）──
        {
            std::vector<TrackedObstacle> detections;
            ConvertClustersToTrackedObstacles(outputClusters, detections);
            m_tracker.update(detections, rec_timestamp_ms);
            // m_tracker.update_V2(detections, rec_timestamp_ms);

            for (const auto& track : m_tracker.vtrackings)
            {
                // std::cout << "Track id: " << track.id << ": "
                //           << "Center(" << track.pos_x << ", "
                //           << track.pos_y << ", "
                //           << track.pos_z << "), "
                //           << "age=" << track.age
                //           << std::endl;
                
                // LOG_RAW("Track id: %d: Center(%.2f, %.2f, %.2f), age=%d\n", track.id, track.pos_x, track.pos_y, track.pos_z, track.age);
            }
        

            // ── Phase 3-A: Motion State（map frame, UNKNOWN/STATIC/MOVING）──
            // 在 m_tracker.update() 之后：维护 map position history + 运动证据 + 状态机，
            // 结果写回 m_tracker.vtrackings（供 DebugViewer / 日志读取）。
            // 不修改 tracker 匹配/删除，不修改 Phase 2 关联。
            UpdateTrackMotionStates(loc_pose, have_pose, rec_timestamp_ms);
            LOG_RAW("\n");
            
            // ── Map Anchor 维护（Phase 2 侧表；必须使用 RAW 几何）────────────────────
            // 时序要求：锚点必须在 geometry refinement 之前刷新，使 map anchor / 历史关联始终基于
            // RAW 检测几何，不形成 "Refined → Anchor → Association → Refined" 的反馈回路。
            // UpdateMapAnchors() 只维护 m_mapTracks（map 位置/角点，取 RAW vtrackings），
            // 不读不写 tracker 几何状态；其中的 Phase 3-B yaw 记忆字段由 RefineStaticObbGeometry 维护。
            UpdateMapAnchors(loc_pose, have_pose);
        }

        // ── Phase 3-B (v2): STATIC Track OBB Geometry Refinement ─────────────────
        // 只读 m_tracker.vtrackings（RAW）；结果写入 outTracks（RAW 快照副本），供 Debug/可视化/UDP 使用。
        // ⚠ 不写回 m_tracker：Tracker State 永远保持 RAW（association / 速度 / age / lastSeen / motion_state 不受影响）。
        std::vector<TrackedObstacle> outTracks;
        RefineStaticObbGeometry(outputClusters, loc_pose, have_pose, outTracks);

        // ── Phase 3-B': 1cm Raw Point Image OBB（实验，仅 Debug 可视化）─────────────
        // 把本帧匹配到的 Cluster 的 1cm Raw Point Image minAreaRect 结果写入 outTracks 的
        // 实验字段 new_corners；不改变任何现有输出（corners/depth/width/tracker/UDP 全不动）。
        AttachRawImageObbs(outputClusters, outTracks);

        // ── Historical Feedback（Phase 2/3 Quick Validation，实验性）──────────────────
        // 只产生 Debug 证据（投影到当前 Grid 的空间先验 + Fused 统计），
        // 不修改 detections / tracker 输入 / 最终 UDP 输出。
        // {
        //     m_historicalFeedback.clear();
        //     ComputeHistoricalFeedback(outputClusters, loc_pose, have_pose, m_historicalFeedback);

        //     if (m_debugViewer)
        //     {
        //         m_debugViewer->DrawHistoricalFeedbackOverlay(
        //             outputClusters, m_historicalFeedback,
        //             m_pElevationMapGroundFilter->GetConfig(),hdmap_polygons, loc_pose);
        //     }

        //     // 每帧结束：用当前 Track + 当前位姿维护地图锚点侧表（下一帧反馈使用）
        //     UpdateMapAnchors(loc_pose, have_pose);
        // }

        // ── Phase 3-B: STATIC Historical Geometry（cell 补充；fused OBB 默认关闭）──
        // 复用 Phase 2 的 m_historicalFeedback（association + projected cells）。
        // ⚠ 必须在 UpdateMapAnchors() 之后：确保 Phase 2 锚点基于原始检测几何，不被 fused 几何污染。
        // ApplyHistoricalGeometry(outputClusters);

        // ── Debug: 第九层 跟踪结果点云叠加图 (TrackedObstacle 版本) ──
        // 需要 DebugViewer 配置 TrackerOverlay.enable=1 才会绘制
        if (m_debugViewer)
        {
            // std::vector<std::vector<STR_POINT2F>> hdmap_polygons;
            // LocalizationManager::Pose loc_pose;
            // bool have_pose = false;

            // if (m_hdmapEnabled)
            // {
            //     have_pose = LocalizationManager::instance().getPose(loc_pose);
            //     if (have_pose && loc_pose.valid)
            //     {
            //         m_hdmapManager.buildDrivablePolygons(loc_pose.x, loc_pose.y, hdmap_polygons);
            //     }
            // }

            // if (have_pose && loc_pose.valid && !hdmap_polygons.empty())
            // {
                m_debugViewer->DrawMapAndAllOverlay(*pGroundCloud, *pObstacleCloud,
                                           outTracks,   // Phase 3-B: RAW + refined 几何（仅输出层）
                                           m_pElevationMapGroundFilter->GetConfig(),
                                           hdmap_polygons,
                                           loc_pose);

                // ── Phase 3-B': 1cm Raw Point Image OBB A/B 对比图（独立窗口 RawImageObb）──
                // 白 = Grid PCA OBB, 青 = 1cm Raw Point Image OBB, 橙 = 1cm ROI,
                // 红 = 原始占用像素, 黄 = 膨胀新增像素, 暗红 = 障碍物点云（空间参考）
                m_debugViewer->DrawRawPointImageObbDebug(*pGroundCloud, *pObstacleCloud,
                                                       m_pElevationMapGroundFilter->GetRawPointImage(),
                                                       m_pElevationMapGroundFilter->GetRawImageWidth(),
                                                       m_pElevationMapGroundFilter->GetRawImageHeight(),
                                                       outTracks, outputClusters,
                                                       m_pElevationMapGroundFilter->GetConfig(),
                                                       hdmap_polygons,
                                                       loc_pose);
            // }
            // else
            // {
            //     m_debugViewer->DrawTrackOverlay(*pGroundCloud, *pObstacleCloud,
            //                                m_tracker.vtrackings,
            //                                m_pElevationMapGroundFilter->GetConfig());
            // }
        }

        auto t_end_5_ = std::chrono::steady_clock::now();
        auto duration_5 = std::chrono::duration_cast<std::chrono::milliseconds>(t_end_5_ - t_end_4_);


        if(m_ST_Config->pcapRunningModel){

            // SavePcd(pInputCloud);
            // SavePcd(pFilteredPointCloud);
            // SavePcd(pObstacleCloud);
            // SavePcd(p_comm_no_Ground);

            /* ---- 可视化已迁移至 main.cpp ----
            // ============ 共用可视化：逐帧更新 ============
            InitGroundViewer();
            // ============ 共用可视化：逐帧更新 (tracker 稳定 ID) ============
            // UpdateGroundViewer(pFilteredPointCloud, pObstacleCloud, outputClusters);
            // UpdateGroundViewer(pGroundCloud, pObstacleCloud, outputClusters);
            UpdateGroundViewer(pGroundCloud, pObstacleCloud, m_tracker.vtrackings);
            // UpdateGroundViewer(p_comm_ground, p_comm_no_Ground, m_tracker.vtrackings); //
            SpinGroundViewerOnce();  // 非阻塞，给 UI 刷新机会
            ---- */

            // ============ 帧缓冲：发布数据给主线程渲染（Phase 3-B: 使用 refined 输出副本）============
            m_visBuffer.Publish(pGroundCloud, pObstacleCloud, outTracks);
        }
        auto t_end_6_ = std::chrono::steady_clock::now();
        auto duration_6 = std::chrono::duration_cast<std::chrono::milliseconds>(t_end_6_ - t_end_5_);


        if(m_ST_Config->onlineModel){
            newS2AviodObject outputObject;
            memset(&outputObject, 0, sizeof(outputObject));
            // Phase 3-B: 使用 refined 输出副本（outTracks）；Tracker 内部仍为 RAW。
            m_pElevationMapGroundFilter->ConvertTrackToS2ObstacleBox(outTracks, rec_timestamp_ms, outputObject);

            
            int send_ret = sendUdpMsg(send_fd, (unsigned char*)&outputObject, sizeof(outputObject), ip, port);
            
            LOG_RAW("send timestamp=%llu return_val=%d obs_num=%d sizeof=%zu\n",
                outputObject.timestamp,
                outputObject.return_val,
                outputObject.obs_num,
                sizeof(newS2AviodObject));
        }

        auto t_end = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start);

        g_previousTimestamp.store(rec_timestamp_ms, std::memory_order_relaxed);

        LOG_RAW("接收数据 %lld ms, 坐标转换 %lld ms, APMF: %lld ms, ProcessWithObstacleDetection: %lld ms, Track: %lld ms, 可视化: %lld ms, Total completed in **(%lld)** ms\n",
                    duration_1.count(), duration_2.count(), duration_3.count(), duration_4.count(), duration_5.count(), duration_6.count(), duration.count(),
                    pGroundCloud->size(), pObstacleCloud->size(), outputClusters.size());

        //── DebugViewer: 刷新 OpenCV 窗口 ──
        if (m_debugViewer)
        {
            m_debugViewer->SpinOnce();
        }
    }

}


// ============================================================================
// Historical Feedback (Phase 2/3 Quick Validation)：地图锚点维护 + 当前 Grid 投影
// ============================================================================

// 每帧结束：把本帧匹配成功(lastSeen==0)的 Track 用当前位姿锚定到地图系；
// 匹配失败(miss)的 Track 冻结其地图锚点；被 tracker 删除的 Track 同步删除锚点。
void SutengDriver::UpdateMapAnchors(const LocalizationManager::Pose& pose, bool pose_valid)
{
    if (!pose_valid)
    {
        // 定位无效：冻结所有锚点（不更新），保留历史最后一次地图位置
        return;
    }

    VehiclePose vpose;
    vpose.x = pose.x;
    vpose.y = pose.y;
    vpose.heading_deg = pose.heading_deg;

    std::unordered_set<int> alive_ids;
    for (const auto& t : m_tracker.vtrackings)
    {
        alive_ids.insert(t.id);

        MapAnchoredTrack* anchor = nullptr;
        for (auto& a : m_mapTracks)
        {
            if (a.id == t.id) { anchor = &a; break; }
        }
        if (anchor == nullptr)
        {
            m_mapTracks.push_back(MapAnchoredTrack());
            anchor = &m_mapTracks.back();
            anchor->id = t.id;
        }

        if (t.lastSeen == 0)
        {
            // 本帧匹配成功：用当前位姿更新地图锚点
            CoordinateTransformer::vehicleToMap(t.pos_x, t.pos_y, vpose,
                                                anchor->map_x, anchor->map_y);
            anchor->has_map = true;
            anchor->depth = t.depth;
            anchor->width = t.width;
            for (int j = 0; j < 4; ++j)
            {
                double mx = 0.0, my = 0.0;
                CoordinateTransformer::vehicleToMap(t.corners[j].x, t.corners[j].y,
                                                    vpose, mx, my);
                anchor->map_corners[j].x = static_cast<float>(mx);
                anchor->map_corners[j].y = static_cast<float>(my);
            }
            anchor->age = t.age;
            anchor->lastSeen = t.lastSeen;
        }
        else
        {
            // miss：冻结锚点，仅同步 lastSeen（供调试观察生命周期）
            // 为什么这里【不需要】像雷达系那样做 miss 重投影：
            //   - 本函数存的是【地图系】anchor(map_x/map_y/map_corners)，地图系不随自车运动，
            //     静止目标在地图系里本来就不动 → “不更新”就是正确值，freeze = 正确语义。
            //     （雷达系之所以要重投影，是因为雷达系本身跟着自车动，
            //       沿用上一帧的雷达系坐标 = 目标跟着车跑，这是两码事。）
            //   - 调用顺序：UpdateTrackMotionStates()（雷达系 miss 重投影，只改 vtrackings 的
            //     pos/corners/vx/vy）→ UpdateMapAnchors()（本函数）。miss 帧这里只写 lastSeen、
            //     不读 vtrackings 的几何，所以两者互不干扰，锚点不会被重投影结果污染。
            //   - 运动目标的地图系 coast（anchor += map_v*dt）属 Phase 3-C 范围，本阶段不做：
            //     宁可让历史先验滞后，也不伪造运动（ComputeHistoricalFeedback 只做投影，不做外推）。
            anchor->lastSeen = t.lastSeen;
        }
    }

    // 删除 tracker 已移除的 Track 的锚点
    m_mapTracks.erase(
        std::remove_if(m_mapTracks.begin(), m_mapTracks.end(),
                       [&alive_ids](const MapAnchoredTrack& a) {
                           return alive_ids.find(a.id) == alive_ids.end();
                       }),
        m_mapTracks.end());
}

// ============================================================================
// Phase 3-A: Motion State (map frame, UNKNOWN / STATIC / MOVING)
// ============================================================================
// 在 m_tracker.update() 之后调用：把本帧【匹配成功】的 Track 位置经 vehicleToMap 转入地图系，
// 维护最近 map position history，计算运动证据（step / net / 方向一致性 / 速度），
// 用带迟滞的状态机更新 motion_state，并写回 m_tracker.vtrackings（供可视化/日志读取）。
//
// 原则：
//   - 只在 matched(lastSeen==0) 且 pose.valid 时产生新的 map 观测；miss 帧不追加、不伪造位置。
//   - age / lastSeen / vx / vy 语义不变；map 速度写入独立的 map_vx/map_vy。
//   - 不改 SimpleTracker::update() 匹配/删除，不改 Phase 2 关联。

// 前置声明：定义在本文件 Phase 3-B 段（RefineStaticObbGeometry 之前），
// 供 UpdateTrackMotionStates 记录 map_yaw_deg / 漏检帧重投影复用（同一 TU 内的 static 函数）。
static float RadarYawRadToMapYawRad(float yaw_rad, const VehiclePose& vpose);
static float MapYawRadToRadarYawRad(float yaw_map_rad, const VehiclePose& vpose);

void SutengDriver::UpdateTrackMotionStates(const LocalizationManager::Pose& pose,
                                           bool pose_valid,
                                           unsigned long long rec_timestamp_ms)
{
    constexpr float kPi = 3.14159265358979323846f;

    // dt 与 SimpleTracker::update() 同源（g_previousTimestamp 在帧末才更新）
    const unsigned long long prev_ts = g_previousTimestamp.load(std::memory_order_relaxed);
    const double dt = (rec_timestamp_ms > prev_ts)
                      ? static_cast<double>(rec_timestamp_ms - prev_ts) / 1000.0
                      : 0.0;

    VehiclePose vpose;
    vpose.x = pose.x;
    vpose.y = pose.y;
    vpose.heading_deg = pose.heading_deg;

    for (auto& t : m_tracker.vtrackings)
    {
        bool new_obs = false;
        bool reanchored = false;   // 本帧是否被“地图系 anchored 重投影”修正（仅供日志）

        // ---- 只有 matched + pose valid 才产生新的 map 观测 ----
        if (t.lastSeen == 0 && pose_valid)
        {
            double mx = 0.0, my = 0.0;
            CoordinateTransformer::vehicleToMap(t.pos_x, t.pos_y, vpose, mx, my);
            new_obs = true;

            // 1) 帧间位移 / 地图系速度（EMA）
            if (t.has_map_pos)
            {
                const float dx = static_cast<float>(mx) - t.map_x;
                const float dy = static_cast<float>(my) - t.map_y;
                t.motion_step    = std::sqrt(dx * dx + dy * dy);
                t.motion_dir_deg = std::atan2(dy, dx) * 180.0f / kPi;
                if (dt > 1e-3)
                {
                    const float nvx = dx / static_cast<float>(dt);
                    const float nvy = dy / static_cast<float>(dt);
                    t.map_vx = kMotionVEmaAlpha * nvx + (1.0f - kMotionVEmaAlpha) * t.map_vx;
                    t.map_vy = kMotionVEmaAlpha * nvy + (1.0f - kMotionVEmaAlpha) * t.map_vy;
                }
            }
            else
            {
                t.motion_step = 0.0f;
                t.motion_dir_deg = 0.0f;
            }

            // 2) 更新当前 map position + history（漏检帧不追加）
            t.map_x = static_cast<float>(mx);
            t.map_y = static_cast<float>(my);
            t.has_map_pos = true;

            // 2.1) 记录 OBB 长轴朝向（地图系）：
            //      长轴方向由雷达系角点 corner0→corner1 得到（与 BuildObbCorners 约定一致），
            //      再用两点法转成地图系 yaw，落盘为 deg。
            //      用途：miss 帧可视化时 雷达系 yaw = f(map_yaw_deg, 当前 pose)，
            //            自车转向时重建的框方向依然正确（消除“已知残余误差”）。
            {
                const float dx_c = t.corners[1].x - t.corners[0].x;
                const float dy_c = t.corners[1].y - t.corners[0].y;
                if (dx_c * dx_c + dy_c * dy_c > 1e-6f)
                {
                    const float yaw_radar_rad = std::atan2(dy_c, dx_c);
                    const float yaw_map_rad   = RadarYawRadToMapYawRad(yaw_radar_rad, vpose);
                    t.map_yaw_deg = yaw_map_rad * 180.0f / kPi;
                    t.has_map_yaw = true;
                }
                else
                {
                    // 角点退化（尺寸≈0）：不伪造方向，可视化侧回退旧雷达系 yaw
                    t.has_map_yaw = false;
                }
            }

            t.recent_map_positions.push_back(Point2D{ static_cast<float>(mx), static_cast<float>(my) });
            if (static_cast<int>(t.recent_map_positions.size()) > kMotionHistoryCap)
            {
                t.recent_map_positions.erase(t.recent_map_positions.begin());
            }

            // 3) 窗口证据：净位移 + 方向一致性
            const int n = static_cast<int>(t.recent_map_positions.size());
            float net = 0.0f; //净位移
            bool  dir_ok = false;
            if (n >= 2)
            {
                int win = kMotionMoveWin;
                if (win > n - 1) win = n - 1;
                const Point2D& lastp  = t.recent_map_positions[n - 1];
                const Point2D& firstp = t.recent_map_positions[n - 1 - win];
                const float ndx = lastp.x - firstp.x;
                const float ndy = lastp.y - firstp.y;
                net = std::sqrt(ndx * ndx + ndy * ndy);
                const float net_dir = std::atan2(ndy, ndx) * 180.0f / kPi;

                // 需要至少 kMotionMMove 步，且每一步方向与窗口净方向夹角 <= DIR_TOL
                if (n >= kMotionMMove + 1)
                {
                    dir_ok = true;
                    for (int i = n - kMotionMMove; i < n; ++i)
                    {
                        const float sdx = t.recent_map_positions[i].x - t.recent_map_positions[i - 1].x;
                        const float sdy = t.recent_map_positions[i].y - t.recent_map_positions[i - 1].y;
                        const float step_len = std::sqrt(sdx * sdx + sdy * sdy);
                        if (step_len < 1e-6f) { dir_ok = false; break; }
                        float d = std::fabs(std::atan2(sdy, sdx) * 180.0f / kPi - net_dir);
                        if (d > 180.0f) d = 360.0f - d;
                        if (d > kMotionDirTolDeg) { dir_ok = false; break; }
                    }
                }
            }
            t.motion_net = net;

            // 4) 计数器（死区内两者都清零 → 状态保持，形成迟滞）
            const bool moving_ok = (t.motion_step >= kMotionMoveStepMin) && dir_ok &&
                                   (net >= kMotionMoveNetMin);
            const bool static_ok = (t.motion_step <= kMotionStaticStepMax);

            t.motion_moving_run = moving_ok ? (t.motion_moving_run + 1) : 0;
            t.motion_static_run = static_ok ? (t.motion_static_run + 1) : 0;

            // 5) 状态迁移（迟滞：进入快、退出慢；单帧不足以切换）
            switch (t.motion_state)
            {
                case MotionState::UNKNOWN:
                    if (t.motion_moving_run >= kMotionMMove)
                        t.motion_state = MotionState::MOVING;
                    else if (t.motion_static_run >= kMotionUnknownToStatic)
                        t.motion_state = MotionState::STATIC;
                    break;
                case MotionState::STATIC:
                    if (t.motion_moving_run >= kMotionMMove)
                        t.motion_state = MotionState::MOVING;
                    break;
                case MotionState::MOVING:
                    if (t.motion_static_run >= kMotionMovingToStatic)
                        t.motion_state = MotionState::STATIC;
                    break;
            }

            // ---- 日志：matched 都输出，便于判断“为什么”是该状态 ----
            LOG_RAW("[MotionState] Track=%d {age=%d lastSeen=%d} state=%s Center(%.2f,%.2f,%.2f) map=(%.2f,%.2f) step=%.3f 净位移=%.3f dir=%.1f mapV=(%.2f,%.2f) obs=%d anchor=%d hist=%zu srun=%d mrun=%d\n",
                    t.id, t.age, t.lastSeen, MotionStateName(t.motion_state),
                    t.pos_x, t.pos_y, t.pos_z,
                    t.map_x, t.map_y, t.motion_step, t.motion_net, t.motion_dir_deg,
                    t.map_vx, t.map_vy, (int)new_obs, (int)reanchored,
                    t.recent_map_positions.size(),
                    t.motion_static_run, t.motion_moving_run);
        
        }
        else if (pose_valid && t.has_map_pos)
        {
            // ---- 漏检帧（含 detections 全空的帧）：地图系 anchored 重投影 ----
            // 背景（与 track.cpp 漏检分支的注释同源）：
            //   SimpleTracker::update() 的漏检分支拿不到位姿，只能做“雷达系 v*dt 外推”或
            //   “位置不动”，自车一动，目标就等效于跟着车跑（静止目标在地图系里会漂）。
            //   因此这里在【定位有效 + 已有地图锚】时，把地图系位置按当前 pose 重投影回雷达系，
            //   中心与 OBB 角点一并重建（与 DebugViewer::DrawMapAndAllOverlay 完全同源）。
            // 设计取舍（重要）：
            //   - 不 coast（不按 map_v 外推 map_x/map_y）：否则下一帧 matched 时
            //     motion_step=|观测-预测|≈0 → Phase 3-A 会把运动目标误判为 STATIC。
            //     静止目标的地图系位置本来就不动，不 coast 对主场景（低矮静态障碍）无损。
            //   - 不追加 recent_map_positions：重投影不是新观测，不污染运动证据/状态机。


            // 1) 地图系 → 本帧雷达系（位置）
            //    ⚠ 覆盖前先存下 pos/v：
            //      pos_pre = tracker 本帧漏检分支已做过一次 pos += v_old*dt 外推后的值
            //      v_old   = 上一帧写回（或上次 matched 计算）的雷达系速度
            //    两者用于第 3) 步把速度写回成“与本次重投影自洽”的值。
            const float pos_pre_x = t.pos_x;
            const float pos_pre_y = t.pos_y;
            const float vx_old    = t.vx;
            const float vy_old    = t.vy;

            double veh_x = 0.0, veh_y = 0.0;
            CoordinateTransformer::mapToVehicle(static_cast<double>(t.map_x),
                                                static_cast<double>(t.map_y),
                                                vpose, veh_x, veh_y);
            t.pos_x = static_cast<float>(veh_x);
            t.pos_y = static_cast<float>(veh_y);

            // 2) OBB 角点重建（顺序与 BuildObbCorners 一致: 左下→右下→右上→左上）
            //    yaw 优先用“地图系朝向 → 本帧雷达系”（自车转向后框方向仍正确），
            //    无地图 yaw 时退化为旧角点反算的雷达系 yaw（等价假设自车朝向未变）。
            const float dx_c = t.corners[1].x - t.corners[0].x;
            const float dy_c = t.corners[1].y - t.corners[0].y;

            float yaw_radar = std::atan2(dy_c, dx_c);
            if (t.has_map_yaw)
            {
                yaw_radar = MapYawRadToRadarYawRad(t.map_yaw_deg * kPi / 180.0f, vpose);
            }

            float len = t.depth;
            float wid = t.width;
            if (len <= 1e-3f)
            {
                len = std::sqrt(dx_c * dx_c + dy_c * dy_c);
            }
            if (wid <= 1e-3f)
            {
                const float dx_w = t.corners[3].x - t.corners[0].x;
                const float dy_w = t.corners[3].y - t.corners[0].y;
                wid = std::sqrt(dx_w * dx_w + dy_w * dy_w);
            }
            len = std::max(len, 0.05f);   // 退化保护：避免框退化为一个点
            wid = std::max(wid, 0.05f);

            BuildObbCorners(t.pos_x, t.pos_y, yaw_radar, len, wid, t.corners);

            // 3) 写回雷达系速度 vx/vy（供下一帧 SimpleTracker::update 的代价矩阵预测）
            //    为什么必须写回：
            //      下一帧关联用的是 predicted_pos = pos + v*dt（见 track.cpp 的 predicted_positions），
            //      其中 pos 是【本帧重投影后的新位置】。若 v 仍是上次 matched 的旧雷达系值，
            //      预测会二次计入运动中已经包含的部分，偏差变大 → 超过 match_threshold(0.5m)
            //      → 关联失败 → 新 ID（断链）。
            //    取值：与 tracker 的 matched 分支同一约定——
            //      “雷达系两点差分”（含自车运动造成的视在运动）。
            //      推导：tracker 本帧已做过 pos += v_old*dt，先撤销它再差分：
            //        v_new = (p_reproj - (pos_pre - v_old*dt)) / dt = (p_reproj - pos_pre)/dt + v_old
            //      对静止目标：p_reproj 随自车反向移动 v_ego*dt，得到 v_new = -v_ego(雷达系) ✓
            //      对运动目标：得到 1 帧滞后的视在速度（与不 coast 的取舍一致）✓
            //    注：只在 dt 有效时更新，避免除零；dt 与 tracker 的 delta_time 同源
            //        （同一 g_previousTimestamp），所以这里的“撤销”是精确的。
            if (dt > 1e-3)
            {
                const float inv_dt = static_cast<float>(1.0 / dt);
                t.vx = (t.pos_x - pos_pre_x) * inv_dt + vx_old;
                t.vy = (t.pos_y - pos_pre_y) * inv_dt + vy_old;
            }

            reanchored = true;
            
            // ---- 日志：miss 都输出，便于判断“为什么”是该状态 ----
            LOG_RAW("[MotionState] Track=%d miss{age=%d lastSeen=%d} state=%s Center(%.2f,%.2f,%.2f) map=(%.2f,%.2f) step=%.3f 净位移=%.3f dir=%.1f mapV=(%.2f,%.2f) obs=%d anchor=%d hist=%zu srun=%d mrun=%d\n",
                    t.id, t.age, t.lastSeen, MotionStateName(t.motion_state),
                    t.pos_x, t.pos_y, t.pos_z,
                    t.map_x, t.map_y, t.motion_step, t.motion_net, t.motion_dir_deg,
                    t.map_vx, t.map_vy, (int)new_obs, (int)reanchored,
                    t.recent_map_positions.size(),
                    t.motion_static_run, t.motion_moving_run);
        
        }

        
    }
}

// ============================================================================
// Phase 3-B (v2): STATIC Track OBB Geometry Refinement
// ============================================================================
// 设计/阈值/决策表见 include/static_obb_refinement.h。
//
// 决策（只对 motion_state == STATIC + lastSeen == 0 + pose 有效的 Track）：
//   yaw    = PCA（当前 cells 的 PCA 主方向可观测，且与历史 yaw 连续）
//          | HISTORY（PCA 不可观测，或可观测但与历史明显冲突 → 保守）
//          | RAW（不可观测且无历史 yaw，不凭空创造方向）
//   L/W    = 用最终 yaw 在当前帧 cells 上重新投影（不使用历史 L/W）
//   center = 当前 RAW center（不做任何位置平滑）
//
// 隔离（架构硬约束）：
//   - 只读 m_tracker.vtrackings（RAW）；refined 几何只写入 outTracks（调用方持有的副本）
//   - 不修改 association / vx,vy / age / lastSeen / motion_state / 删除策略
//   - 历史 yaw 记忆保存在 m_mapTracks（MapAnchoredTrack::static_yaw_map_rad，地图系，弧度），
//     只保存【被接受】的稳定 yaw；UpdateMapAnchors() 不修改该字段（锚点始终基于 RAW 几何）

// ---- Phase 3-B 局部工具：雷达系 yaw ↔ 地图系 yaw（无向长轴）----
// 用"两点法"：对同一方向上的两个点分别做坐标变换后取差，
// 自动包含 CoordinateTransformer 内部的 heading + gridHeadingOffsetDeg，避免手推符号出错。
static float RadarYawRadToMapYawRad(float yaw_rad, const VehiclePose& vpose)
{
    double x0 = 0.0, y0 = 0.0, x1 = 0.0, y1 = 0.0;
    CoordinateTransformer::vehicleToMap(0.0, 0.0, vpose, x0, y0);
    CoordinateTransformer::vehicleToMap(static_cast<double>(std::cos(yaw_rad)),
                                        static_cast<double>(std::sin(yaw_rad)), vpose, x1, y1);
    return NormalizeYaw180(static_cast<float>(std::atan2(y1 - y0, x1 - x0)));
}

static float MapYawRadToRadarYawRad(float yaw_map_rad, const VehiclePose& vpose)
{
    double x0 = 0.0, y0 = 0.0, x1 = 0.0, y1 = 0.0;
    CoordinateTransformer::mapToVehicle(0.0, 0.0, vpose, x0, y0);
    CoordinateTransformer::mapToVehicle(static_cast<double>(std::cos(yaw_map_rad)),
                                        static_cast<double>(std::sin(yaw_map_rad)), vpose, x1, y1);
    return NormalizeYaw180(static_cast<float>(std::atan2(y1 - y0, x1 - x0)));
}

void SutengDriver::RefineStaticObbGeometry(const std::vector<GridCluster>& clusters,
                                           const LocalizationManager::Pose& pose,
                                           bool pose_valid,
                                           std::vector<TrackedObstacle>& outTracks)
{
    // ── 输出默认 = RAW 快照（无论是否 refinement，Debug/UDP 都消费 outTracks）──
    outTracks = m_tracker.vtrackings;

    if (!kObbRefineEnable) return;

    const ElevationGridConfig& cfg = m_pElevationMapGroundFilter->GetConfig();
    const int   cols  = m_pElevationMapGroundFilter->GetGridCols();
    const float eff_x = m_pElevationMapGroundFilter->GetEffectiveRoiXMin();
    const float res   = cfg.grid_resolution;
    if (res <= 0.0f || cols <= 0) return;

    // 本帧 cluster id → GridCluster（cluster.id 为帧内唯一编号）
    std::unordered_map<int, const GridCluster*> cluster_by_id;
    cluster_by_id.reserve(clusters.size());
    for (const auto& cl : clusters)
    {
        cluster_by_id[cl.id] = &cl;
    }

    VehiclePose vpose;
    vpose.x = pose.x;
    vpose.y = pose.y;
    vpose.heading_deg = pose.heading_deg;

    int n_static = 0, n_refined = 0, n_pca = 0, n_hist = 0;
    int n_not_static = 0, n_no_match = 0, n_no_cluster = 0, n_no_obb = 0;
    int n_no_pose = 0, n_no_yaw = 0, n_axis_mismatch = 0;

    std::vector<Point2D> centers;
    centers.reserve(16);

    for (size_t i = 0; i < m_tracker.vtrackings.size(); ++i)
    {
        const TrackedObstacle& t = m_tracker.vtrackings[i];   // RAW Track（只读）

        // 历史 yaw 记忆（先查找，便于在 MOVING 时清理陈旧记忆）
        MapAnchoredTrack* anchor = nullptr;
        for (auto& a : m_mapTracks)
        {
            if (a.id == t.id) { anchor = &a; break; }
        }

        // ---- 只处理 STATIC；UNKNOWN / MOVING 一律 RAW ----
        if (t.motion_state != MotionState::STATIC)
        {
            n_not_static++;
            // MOVING：目标姿态是时变量，此前积累的 “STATIC 稳定 yaw” 不再代表当前姿态 → 清除，
            // 避免 MOVING → STATIC 后误用陈旧历史方向；UNKNOWN 不动（保持“单帧不可靠不失效”语义）。
            if (t.motion_state == MotionState::MOVING && anchor != nullptr)
            {
                anchor->has_static_yaw     = false;
                anchor->static_yaw_map_rad = 0.0f;
            }
            continue;
        }
        n_static++;

        // ---- 必须本帧匹配到检测（STATIC + miss 的预测属 Phase 3-C，本阶段不做）----
        if (t.lastSeen != 0)
        {
            n_no_match++;
            LOG_RAW("[GeomRefine] Track=%d frame=%d state=STATIC skip=NO_MATCH lastSeen=%d\n", t.id, test_Frame_count, t.lastSeen);
            continue;
        }

        // ---- 本帧匹配到的 Cluster（cell 几何来源）----
        auto it_cl = cluster_by_id.find(t.cluster_id);
        if (it_cl == cluster_by_id.end())
        {
            n_no_cluster++;
            LOG_RAW("[GeomRefine] Track=%d frame=%d state=STATIC skip=NO_CLUSTER cluster_id=%d\n",
                    t.id, test_Frame_count, t.cluster_id);
            continue;
        }
        const GridCluster& cl = *it_cl->second;
        const int n_cells = static_cast<int>(cl.cell_indices.size());

        if (!cl.has_obb)
        {
            n_no_obb++;
            LOG_RAW("[GeomRefine] Track=%d frame=%d state=STATIC skip=NO_OBB cells=%d\n", t.id, test_Frame_count, n_cells);
            continue;
        }

        // ---- pose：仅用于 map↔radar 的 yaw 投影（不引入新的定位逻辑）----
        if (!pose_valid)
        {
            n_no_pose++;
            LOG_RAW("[GeomRefine] Track=%d frame=%d state=STATIC skip=NO_POSE (yaw prior 无法投影)\n", t.id, test_Frame_count);
            continue;
        }

        // ---- 历史 yaw（此前积累的、被接受的稳定 yaw；先读后写，避免被本帧 PCA 覆盖）----
        const bool  has_hist = (anchor != nullptr) && anchor->has_static_yaw;
        const float hist_yaw = has_hist ? MapYawRadToRadarYawRad(anchor->static_yaw_map_rad, vpose)
                                        : 0.0f;

        // ---- 决策：PCA / HISTORY / RAW ----
        StaticObbDecision dec = DecideStaticObbYaw(cl.obb_angle,
                                                   cl.obb_lambda_max,
                                                   cl.obb_lambda_min,
                                                   n_cells,
                                                   has_hist,
                                                   hist_yaw);

        if (!dec.refined)
        {
            n_no_yaw++;
            LOG_RAW("[GeomRefine] Track=%d frame=%d state=STATIC cells=%d lambdaMax=%.6f lambdaMin=%.6f "
                    "eigenRatio=%.3f observable=0 skip=NO_USEFUL_YAW (无历史 yaw，不凭空创造方向)\n",
                    t.id, test_Frame_count, n_cells, cl.obb_lambda_max, cl.obb_lambda_min, dec.eigen_ratio);
            continue;
        }

        // ---- 用 final yaw 在当前帧 cells 上重新投影得到 L/W（不使用历史 L/W）----
        centers.clear();
        for (int idx : cl.cell_indices)
        {
            float cx = 0.0f, cy = 0.0f;
            HistCellCenter(idx, cols, eff_x, cfg.roi_y_min, res, cx, cy);
            centers.push_back(Point2D{ cx, cy });
        }

        float L = 0.0f, W = 0.0f;
        ComputeYawAlignedExtents(centers, dec.yaw, L, W);
        const bool axis_mismatch = (L < W);   // historical yaw 与当前 cells 长轴不一致时的诊断
        if (axis_mismatch) n_axis_mismatch++;

        // ---- center = 当前 RAW center；据此重建 corners（历史 corners 一律不用）----
        Point2D corners[4];
        BuildObbCorners(t.pos_x, t.pos_y, dec.yaw, L, W, corners);

        outTracks[i].depth = L;
        outTracks[i].width = W;
        for (int c = 0; c < 4; ++c) outTracks[i].corners[c] = corners[c];
        // outTracks[i].pos_x / pos_y / pos_z / height 保持 RAW，不修改

        // ---- 历史 yaw 更新：仅当本次采用了可靠的当前 PCA 方向 ----
        if (dec.update_history && anchor != nullptr)
        {
            anchor->static_yaw_map_rad = RadarYawRadToMapYawRad(dec.yaw, vpose);
            anchor->has_static_yaw = true;
        }

        n_refined++;
        if (dec.yaw_source == ObbYawSource::PCA) n_pca++; else n_hist++;

        if (kObbRefineVerbose)
        {
            LOG_RAW("[GeomRefine] Track=%d frame=%d state=STATIC cells=%d lambdaMax=%.6f lambdaMin=%.6f "
                    "eigenRatio=%.3f pcaYaw=%.1f histYaw=%.1f yawDelta180=%.1f observable=%d "
                    "yawSource=%s finalYaw=%.1f rawL=%.2f rawW=%.2f refinedL=%.2f refinedW=%.2f "
                    "axisMismatch=%d center=(%.2f,%.2f)\n", 
                    t.id, test_Frame_count, n_cells, cl.obb_lambda_max, cl.obb_lambda_min, dec.eigen_ratio,
                    ObbRadToDeg(dec.pca_yaw), has_hist ? ObbRadToDeg(dec.hist_yaw) : -999.0f,
                    ObbRadToDeg(dec.yaw_delta), static_cast<int>(dec.observable),
                    ObbYawSourceName(dec.yaw_source), ObbRadToDeg(dec.yaw),
                    t.depth, t.width, L, W, static_cast<int>(axis_mismatch), t.pos_x, t.pos_y);
        }
    }

    LOG_RAW("[GeomRefineSummary] tracks=%zu static=%d refined=%d yawPca=%d yawHistory=%d raw=%d "
            "notStatic=%d noMatch=%d noCluster=%d noObb=%d noPose=%d noUsefulYaw=%d axisMismatch=%d\n",
            m_tracker.vtrackings.size(), n_static, n_refined, n_pca, n_hist,
            (n_static - n_refined), n_not_static, n_no_match, n_no_cluster, n_no_obb,
            n_no_pose, n_no_yaw, n_axis_mismatch);
}

// 本帧：把历史 Map Track（地图锚点）投影到当前雷达系得到预测位置 A
//      → A 转当前 Grid Base Cell → 3×3 邻居搜索 → 候选当前 Cluster → 关联
// 关联：优先 SAME_TRACK_ID（当前 tracker 同 ID 且本帧匹配），否则几何距离（带门控）。
// 上一阶段的 OBB rasterize / overlap / fused_cells 统计保留为 secondary。
// 本帧结果只进日志 + DebugViewer（Debug / Validation only）。
void SutengDriver::ComputeHistoricalFeedback(
    const std::vector<GridCluster>& clusters,
    const LocalizationManager::Pose& pose,
    bool pose_valid,
    std::vector<HistoricalFeedbackRegion>& out)
{
    out.clear();

    // 定位无效：Hard gate —— Historical Feedback 禁用（锚点在 UpdateMapAnchors 中冻结）
    if (!pose_valid)
    {
        LOG_RAW("[HistoricalAssociation] localization invalid -> disabled (map_tracks=%zu)\n",
                m_mapTracks.size());
        return;
    }

    const ElevationGridConfig& cfg = m_pElevationMapGroundFilter->GetConfig();
    const int rows = m_pElevationMapGroundFilter->GetGridRows();
    const int cols = m_pElevationMapGroundFilter->GetGridCols();
    const float eff_x_min = m_pElevationMapGroundFilter->GetEffectiveRoiXMin();
    const float res = cfg.grid_resolution;
    if (res <= 0.0f || rows <= 0 || cols <= 0) return;
    const float inv_res = 1.0f / res;

    VehiclePose vpose;
    vpose.x = pose.x;
    vpose.y = pose.y;
    vpose.heading_deg = pose.heading_deg;

    // ---- 当前帧索引：cell → cluster；cluster id → clusters 下标 ----
    // 参考中心 = has_obb ? obb_center : center（与 ConvertClustersToTrackedObstacles 一致）
    std::unordered_map<int, int> cell_to_cluster;
    std::unordered_map<int, int> cluster_pos;
    for (size_t i = 0; i < clusters.size(); ++i)
    {
        const GridCluster& cl = clusters[i];
        cluster_pos[cl.id] = static_cast<int>(i);
        for (int idx : cl.cell_indices)
        {
            if (cell_to_cluster.find(idx) == cell_to_cluster.end())
            {
                cell_to_cluster[idx] = cl.id;
            }
        }
    }

    auto refCenterOf = [](const GridCluster& c) -> std::pair<float, float>
    {
        if (c.has_obb) return std::make_pair(c.obb_center_x, c.obb_center_y);
        return std::make_pair(c.center_x, c.center_y);
    };

    const auto findClusterCenter = [&](int cid, float& cx, float& cy) -> bool
    {
        auto it = cluster_pos.find(cid);
        if (it == cluster_pos.end()) return false;
        const GridCluster& c = clusters[it->second];
        const std::pair<float, float> cc = refCenterOf(c);
        cx = cc.first;
        cy = cc.second;
        return true;
    };

    // 逐 Track 关联统计
    int same_id_match    = 0;
    int geometric_match  = 0;
    int no_detection_cnt = 0;

    for (const auto& anchor : m_mapTracks)
    {
        if (!anchor.has_map) continue;
        // 临时年龄门槛：仍在用 anchor.age，但其语义是"最后一次匹配成功时的累计匹配帧数"，
        // 不是"连续观测帧数"。因此日志中同时打印 age / lastSeen / live 实时状态。
        if (anchor.age < kMinTrackAgeForFeedback) continue;

        HistoricalFeedbackRegion r;
        r.track_id = anchor.id;
        r.has_map_anchor = true;
        r.length = anchor.depth;
        r.width  = anchor.width;

        // ---- Map Anchor → 当前 Radar：预测位置 A（本轮最重要的量） ----
        double ax = 0.0, ay = 0.0;
        CoordinateTransformer::mapToVehicle(anchor.map_x, anchor.map_y, vpose, ax, ay);
        r.projected_x = static_cast<float>(ax);
        r.projected_y = static_cast<float>(ay);
        r.center_x = r.projected_x;   // 与旧字段对齐：center = A
        r.center_y = r.projected_y;

        // 4 角点投影（旧 OBB 区域层，secondary 可视化用）
        for (int j = 0; j < 4; ++j)
        {
            double vx = 0.0, vy = 0.0;
            CoordinateTransformer::mapToVehicle(
                static_cast<double>(anchor.map_corners[j].x),
                static_cast<double>(anchor.map_corners[j].y), vpose, vx, vy);
            r.corners[j].x = static_cast<float>(vx);
            r.corners[j].y = static_cast<float>(vy);
        }
        RasterizeQuadToGrid(r.corners, cfg, eff_x_min, rows, cols,
                            r.projected_cell_indices);
        r.valid = true;   // A 投影始终存在；即使完全在 ROI 外也记录（NO_CURRENT_DETECTION）

        // ---- 当前 tracker 中同 ID 的 live track（实时 age / lastSeen / matched）----
        const TrackedObstacle* live = nullptr;
        for (const auto& t : m_tracker.vtrackings)
        {
            if (t.id == anchor.id) { live = &t; break; }
        }
        if (live)
        {
            r.live_track_age      = live->age;
            r.live_track_lastSeen = live->lastSeen;
            r.live_track_matched  = (live->lastSeen == 0);
        }

        // ---- A → 当前 Grid Base Cell ----
        const int raw_row = static_cast<int>(std::floor((ay - cfg.roi_y_min) * inv_res));
        const int raw_col = static_cast<int>(std::floor((ax - eff_x_min) * inv_res));
        r.a_inside_grid = (raw_row >= 0 && raw_row < rows &&
                           raw_col >= 0 && raw_col < cols);
        // 距 ROI 边界 ≤1 cell 也可搜索（刚超出盲区/最远端时夹取到最近边界 cell）
        const bool searchable =
            (raw_row >= -1 && raw_row <= rows &&
             raw_col >= -1 && raw_col <= cols);
        r.a_searchable = searchable;

        int br = raw_row, bc = raw_col;
        if (br < 0) br = 0; else if (br > rows - 1) br = rows - 1;
        if (bc < 0) bc = 0; else if (bc > cols - 1) bc = cols - 1;
        r.base_row = br;
        r.base_col = bc;
        r.base_linear_index = br * cols + bc;

        // ---- 3×3 Search Window：Base Cell + 8 邻居（边界 cell 做合法性检查）----
        //     (-1,-1)(-1,0)(-1,+1) / (0,-1)(0,0)(0,+1) / (+1,-1)(+1,0)(+1,+1)
        if (searchable)
        {
            std::unordered_set<int> cand_set;
            for (int dr = -1; dr <= 1; ++dr)
            {
                for (int dc = -1; dc <= 1; ++dc)
                {
                    const int rr = br + dr;
                    const int cc = bc + dc;
                    if (rr < 0 || rr >= rows || cc < 0 || cc >= cols) continue;
                    const int idx = rr * cols + cc;
                    r.search_window_indices.push_back(idx);
                    auto it = cell_to_cluster.find(idx);
                    if (it != cell_to_cluster.end()) cand_set.insert(it->second);
                }
            }
            r.candidate_cluster_ids.assign(cand_set.begin(), cand_set.end());
            std::sort(r.candidate_cluster_ids.begin(), r.candidate_cluster_ids.end());
        }

        // ---- 候选 Cluster 几何信息（日志 + 几何 fallback）----
        struct CandidateGeom { int id; float cx; float cy; float dist; };
        std::vector<CandidateGeom> cands;
        for (int cid : r.candidate_cluster_ids)
        {
            float ccx = 0.0f, ccy = 0.0f;
            if (!findClusterCenter(cid, ccx, ccy)) continue;
            const float dx = r.projected_x - ccx;
            const float dy = r.projected_y - ccy;
            CandidateGeom g;
            g.id = cid; g.cx = ccx; g.cy = ccy;
            g.dist = std::sqrt(dx * dx + dy * dy);
            cands.push_back(g);
        }

        // ---- Association ----
        int    matched   = -1;
        int    reason    = kHistMatchNone;
        bool   in_window = false;
        float  mdist     = -1.0f;
        float  mcx = 0.0f, mcy = 0.0f;

        if (live && live->lastSeen == 0)
        {
            // 优先：SAME_TRACK_ID —— 同 ID track 本帧被 tracker 匹配成功。
            // tracker 把检测中心拷给了 track.pos，故按"参考中心 ≈ live->pos"反查其 Cluster。
            int cid_by_track = -1;
            for (size_t i = 0; i < clusters.size(); ++i)
            {
                const GridCluster& cl = clusters[i];
                const std::pair<float, float> cc = refCenterOf(cl);
                const float dx = cc.first  - live->pos_x;
                const float dy = cc.second - live->pos_y;
                if (std::sqrt(dx * dx + dy * dy) < 0.05f)
                {
                    cid_by_track = cl.id;
                    break;
                }
            }
            if (cid_by_track >= 0)
            {
                matched   = cid_by_track;
                reason    = kHistMatchSameTrackId;
                in_window = (std::find(r.candidate_cluster_ids.begin(),
                                       r.candidate_cluster_ids.end(), matched)
                             != r.candidate_cluster_ids.end());
                if (findClusterCenter(matched, mcx, mcy))
                {
                    const float dx = r.projected_x - mcx;
                    const float dy = r.projected_y - mcy;
                    mdist = std::sqrt(dx * dx + dy * dy);
                }
            }
        }
        else if (live == nullptr && !cands.empty())
        {
            // 当前 tracker 已无同 ID 目标（可能被删后以新 ID 重建 / 或确实不同目标）。
            // 几何 fallback：3×3 窗口内最近候选 + 距离门控（不无限制取最近）。
            const CandidateGeom* best = nullptr;
            for (const auto& g : cands)
            {
                if (!best || g.dist < best->dist) best = &g;
            }
            if (best && best->dist <= kHistAssociationMaxDistM)
            {
                matched   = best->id;
                reason    = kHistMatchGeometric;
                in_window = true;
                mdist     = best->dist;
                mcx       = best->cx;
                mcy       = best->cy;
            }
        }
        // 其余情况（live 存在但本帧 miss；或 live==null 且窗口无候选；或 A 完全 ROI 外）：
        //   保持 matched=-1 → NO_CURRENT_DETECTION。禁止凭历史目标自动制造当前检测。

        r.association_cluster_id = matched;
        r.association_reason     = reason;
        r.association_in_window  = in_window;
        r.association_distance   = mdist;
        r.matched_center_x = mcx;
        r.matched_center_y = mcy;
        r.no_current_detection  = (matched < 0);

        // ---- secondary：旧 overlap/fused 统计（当前仅 debug 参考，不作主指标）----
        r.historical_cells = static_cast<int>(r.projected_cell_indices.size());
        r.current_cells    = 0;
        r.overlap_cells    = 0;
        r.matched_cluster_id = matched;
        if (matched >= 0)
        {
            auto it = cluster_pos.find(matched);
            if (it != cluster_pos.end())
            {
                const GridCluster& mc = clusters[it->second];
                r.current_cells = static_cast<int>(mc.cell_indices.size());
                for (int widx : r.search_window_indices)
                {
                    auto cit = cell_to_cluster.find(widx);
                    if (cit != cell_to_cluster.end() && cit->second == matched)
                    {
                        r.overlap_cells++;
                    }
                }
            }
        }
        r.fused_cells = r.current_cells + r.historical_cells - r.overlap_cells;
        if (r.fused_cells < 0) r.fused_cells = 0;

        if (r.no_current_detection) no_detection_cnt++;
        else if (reason == kHistMatchSameTrackId) same_id_match++;
        else if (reason == kHistMatchGeometric)   geometric_match++;

        out.push_back(r);

        // ==================== 日志（重点体现 3×3 搜索） ====================
        const char* reason_str = (reason == kHistMatchSameTrackId) ? "SAME_TRACK_ID"
                               : (reason == kHistMatchGeometric)   ? "GEOMETRIC_DISTANCE"
                               : "NO_CURRENT_DETECTION";

        LOG_RAW("[HistoricalAssociation] Track=%d age=%d lastSeen=%d live_age=%d live_lastSeen=%d live_matched=%d mapAnchor=(%.2f,%.2f)\n",
                r.track_id, anchor.age, anchor.lastSeen,
                r.live_track_age, r.live_track_lastSeen, (int)r.live_track_matched,
                anchor.map_x, anchor.map_y);
        LOG_RAW("[HistoricalAssociation]   ProjectedCurrent=(%.2f,%.2f) insideGrid=%d baseCell=(%d,%d) windowCells=%zu\n",
                r.projected_x, r.projected_y, (int)r.a_inside_grid,
                r.base_row, r.base_col, r.search_window_indices.size());

        // 3×3 布局：'-'=窗口内无 cluster, 'X'=越界(未搜索), C{id}=该 cell 所属当前 cluster
        // 修复：每格使用独立缓冲，避免相邻格写满 5 字符后覆盖前格 null 终止符，
        //       导致 %s 把整行 15 字符重复打印（现象：同一行 C8 被打印 3 遍）。
        if (searchable)
        {
            char celltag[3][3][8];
            for (int dr = -1; dr <= 1; ++dr)
            {
                for (int dc = -1; dc <= 1; ++dc)
                {
                    const int rr = br + dr;
                    const int cc = bc + dc;
                    char* seg = celltag[dr + 1][dc + 1];
                    if (rr < 0 || rr >= rows || cc < 0 || cc >= cols)
                    {
                        std::snprintf(seg, 8, "  X ");
                        continue;
                    }
                    auto it = cell_to_cluster.find(rr * cols + cc);
                    if (it != cell_to_cluster.end())
                        std::snprintf(seg, 8, "C%-2d", it->second);
                    else
                        std::snprintf(seg, 8, "  - ");
                }
            }
            LOG_RAW("[HistoricalAssociation]   Search3x3 row-1: [%s][%s][%s]\n",
                    celltag[0][0], celltag[0][1], celltag[0][2]);
            LOG_RAW("[HistoricalAssociation]   Search3x3 row  : [%s][%s][%s]\n",
                    celltag[1][0], celltag[1][1], celltag[1][2]);
            LOG_RAW("[HistoricalAssociation]   Search3x3 row+1: [%s][%s][%s]\n",
                    celltag[2][0], celltag[2][1], celltag[2][2]);
        }
        else
        {
            LOG_RAW("[HistoricalAssociation]   Search3x3: skipped (A outside ROI >1 cell)\n");
        }

        std::string cand_str;
        for (const auto& g : cands)
        {
            char tmp[128];
            std::snprintf(tmp, sizeof(tmp),
                          "Cluster=%d Center=(%.2f,%.2f) Dist=%.3f | ",
                          g.id, g.cx, g.cy, g.dist);
            cand_str += tmp;
        }
        if (cand_str.empty()) cand_str = "none";
        LOG_RAW("[HistoricalAssociation]   Candidates: %s\n", cand_str.c_str());

        if (matched >= 0)
        {
            LOG_RAW("[HistoricalAssociation]   MatchedCluster=%d Center=(%.2f,%.2f) Distance=%.3f inWindow=%d reason=%s\n",
                    matched, mcx, mcy, mdist, (int)in_window, reason_str);
        }
        else
        {
            LOG_RAW("[HistoricalAssociation]   MatchedCluster=-1 reason=%s (Historical A exists, no current detection)\n",
                    reason_str);
        }
    }

    // 帧摘要
    LOG_RAW("[HistoricalAssociation] frame summary: hist_tracks=%zu sameTrackIdMatches=%d geomMatches=%d noCurrentDetection=%d\n",
            out.size(), same_id_match, geometric_match, no_detection_cnt);
}

// ============================================================================
// Phase 3-B: STATIC Historical Geometry
// ============================================================================
// 复用 Phase 2 的 m_historicalFeedback（association + projected historical cells），
// 对 STATIC + 已关联的 Track 做 cell 补充（见 include/historical_geometry.h）。
// 仅在 kEnableHistoricalFusedOBB=true 且存在 accepted supplement 时写回 vtrackings 几何。
void SutengDriver::ApplyHistoricalGeometry(const std::vector<GridCluster>& clusters)
{
    m_historicalGeometry.clear();

    FuseHistoricalGeometry(clusters, m_historicalFeedback, m_tracker.vtrackings,
                           m_pElevationMapGroundFilter->GetConfig(),
                           m_pElevationMapGroundFilter->GetGridRows(),
                           m_pElevationMapGroundFilter->GetGridCols(),
                           m_pElevationMapGroundFilter->GetEffectiveRoiXMin(),
                           m_historicalGeometry);

    for (const auto& g : m_historicalGeometry)
    {
        const char* st = MotionStateName(static_cast<MotionState>(g.motion_state));
        const char* ar = (g.association_reason == kHistMatchSameTrackId) ? "SAME_TRACK_ID"
                       : (g.association_reason == kHistMatchGeometric)   ? "GEOMETRIC"
                                                                        : "NONE";
        if (!g.fused)
        {
            LOG_RAW("[HistoricalGeometry] Track=%d state=%s cluster=%d assoc=%s cur=%d hist=%d missing=%d accepted=0 skip=%s\n",
                    g.track_id, st, g.cluster_id, ar,
                    g.current_cells, g.historical_cells, g.missing_cells,
                    (g.skip_reason != nullptr && g.skip_reason[0] != '\0') ? g.skip_reason
                                                                           : "NO_SUPPLEMENT");
            continue;
        }

        LOG_RAW("[HistoricalGeometry] Track=%d state=%s cluster=%d assoc=%s cur=%d hist=%d missing=%d accepted=%d rej(oog=%d other=%d dist=%d conn=%d area=%d) area_ratio=%.2f max_supp=%d fusion=ACCEPT obb=%d\n",
                g.track_id, st, g.cluster_id, ar,
                g.current_cells, g.historical_cells, g.missing_cells, g.accepted_cells,
                g.rejected_out_of_grid, g.rejected_other_cluster, g.rejected_distance,
                g.rejected_connectivity, g.rejected_area,
                g.area_ratio, g.max_supplement, (int)g.obb_updated);

        if (g.obb_updated)
        {
            LOG_RAW("[HistoricalGeometry]   fusedOBB center=(%.2f,%.2f) L=%.2f W=%.2f yaw=%.1f conf=%.2f\n",
                    g.center_x, g.center_y, g.length, g.width,
                    g.yaw_deg, g.orientation_confidence);
        }
    }
}


void SutengDriver::Free(){

    delete m_pCommonGroundDetection;
    // m_pCommonGroundDetection = nullptr;
}
void SutengDriver::Stop()
{
    m_running = false;

    m_driver.stop();

    // stuffed_cloud_queue.clear();
    // free_cloud_queue.clear();


    if(m_processThread.joinable()){
        m_processThread.join();
    }


    if (m_SutengDriverThread.joinable()){
        m_SutengDriverThread.join();
    }
}


// ============================================================================
// GridCluster → TrackedObstacle 转换（供 tracker 使用）
// ============================================================================
// ============================================================================
// Phase 3-B': 1cm Raw Point Image OBB → outTracks 实验字段（仅 Debug 可视化）
// ============================================================================
//
// 数据流：
//   GridCluster.img_corners（本帧 1cm Raw Point Image ROI → minAreaRect）
//        │  经 cluster_id 关联（SimpleTracker matched 分支已写回 cluster_id）
//        ▼
//   outTracks[i].new_corners / has_new_corners
//        │
//        ▼
//   DebugViewer::DrawRawPointImageObbDebug（青框）
//
// 严格约束：
//   - 只写 new_corners / has_new_corners
//   - 不写 corners / depth / width / pos_*（现有输出与 UDP 语义完全不变）
//   - 不写 m_tracker.vtrackings（Tracker State 永远不含实验字段）
//   - 不写 detections（tracker 关联 / 速度 / ID 与 baseline 完全一致）
void SutengDriver::AttachRawImageObbs(const std::vector<GridCluster>& clusters,
                                      std::vector<TrackedObstacle>& outTracks) const
{
    // cluster_id → GridCluster（cluster.id 为帧内唯一编号）
    std::unordered_map<int, const GridCluster*> cluster_by_id;
    cluster_by_id.reserve(clusters.size());
    for (const auto& cl : clusters)
    {
        cluster_by_id[cl.id] = &cl;
    }

    int n_ok = 0, n_miss = 0, n_no_cluster = 0, n_no_img_obb = 0;

    for (auto& t : outTracks)
    {
        t.has_new_corners = false;

        // 只处理本帧【匹配成功】的 Track：miss 帧的 cluster_id 指向历史帧的 cluster，
        // 不在本帧 clusters 集合里 → 自然跳过（不做漏检帧预测，属 Phase 3-C）
        if (t.lastSeen != 0)
        {
            n_miss++;
            continue;
        }

        auto it = cluster_by_id.find(t.cluster_id);
        if (it == cluster_by_id.end())
        {
            n_no_cluster++;
            continue;
        }

        const GridCluster& cl = *it->second;
        if (!cl.has_img_obb)
        {
            n_no_img_obb++;
            continue;
        }

        for (int j = 0; j < 4; ++j)
        {
            t.new_corners[j] = cl.img_corners[j];
        }
        t.has_new_corners = true;
        ++n_ok;

        // ── A/B 逐 Track 日志（yaw 均为 [0,180)；dYaw 为无向轴夹角 mod 180）──
        float pca_yaw_deg = cl.obb_angle * 180.0f / static_cast<float>(M_PI);
        if (pca_yaw_deg < 0.0f) pca_yaw_deg += 180.0f;

        float img_yaw_deg = cl.img_yaw_rad * 180.0f / static_cast<float>(M_PI);
        if (img_yaw_deg < 0.0f) img_yaw_deg += 180.0f;

        float d = img_yaw_deg - pca_yaw_deg;
        while (d <= -90.0f) d += 180.0f;
        while (d >    90.0f) d -= 180.0f;

        LOG_RAW("[RawImgOBB-Track] track=%d cluster=%d cells=%zu RawPts=%d Px=%d PxD=%d "
                "| PCA: c=(%.2f,%.2f) L=%.2f W=%.2f yaw=%.1f "
                "| IMG: c=(%.2f,%.2f) L=%.2f W=%.2f yaw=%.1f | dYaw=%.1f dL=%.2f dW=%.2f\n",
                t.id, t.cluster_id, cl.cell_indices.size(), cl.img_raw_pt_count, cl.img_pixel_count,
                cl.img_pixel_count_dilated,
                cl.obb_center_x, cl.obb_center_y, cl.obb_length, cl.obb_width, pca_yaw_deg,
                cl.img_center_x, cl.img_center_y, cl.img_length, cl.img_width, img_yaw_deg,
                std::fabs(d), cl.img_length - cl.obb_length, cl.img_width - cl.obb_width);
    }

    LOG_RAW("[RawImgOBB-Track-Sum] outTracks=%zu ok=%d skip=miss(%d)/no_cluster(%d)/no_img_obb(%d)\n",
            outTracks.size(), n_ok, n_miss, n_no_cluster, n_no_img_obb);
}


void SutengDriver::ConvertClustersToTrackedObstacles(
    const std::vector<GridCluster>& clusters,
    std::vector<TrackedObstacle>& out) const
{
    out.clear();
    out.reserve(clusters.size());

    for (const auto& cluster : clusters)
    {
        if(cluster.in_road || !m_hdmapEnabled){
            TrackedObstacle obs;

            obs.cluster_id = cluster.id;

            // 注意: 这里不设置 id，id 由 SimpleTracker::update() 统一分配（从 9999 起始）
            obs.id = -1;

            if (cluster.has_obb)
            {
                obs.pos_x = cluster.obb_center_x;
                obs.pos_y = cluster.obb_center_y;
                obs.depth  = cluster.obb_length;
                obs.width  = cluster.obb_width;

                obs.corners[0] = cluster.obb_corners[0];
                obs.corners[1] = cluster.obb_corners[1];
                obs.corners[2] = cluster.obb_corners[2];
                obs.corners[3] = cluster.obb_corners[3];
            }
            else
            {
                obs.pos_x = cluster.center_x;
                obs.pos_y = cluster.center_y;
                obs.depth  = cluster.length;
                obs.width  = cluster.width;

                obs.corners[0] = { cluster.min_x, cluster.min_y };
                obs.corners[1] = { cluster.max_x, cluster.min_y };
                obs.corners[2] = { cluster.max_x, cluster.max_y };
                obs.corners[3] = { cluster.min_x, cluster.max_y };
            }

            obs.pos_z  = cluster.center_z;
            obs.height = cluster.height;
            obs.age    = 0;
            obs.lastSeen = 0;

            obs.vx = 0.0;
            obs.vy = 0.0;


            // Translation/Rotation 暂不填充
            memset(obs.Translation, 0, sizeof(obs.Translation));
            memset(obs.Rotation, 0, sizeof(obs.Rotation));

            out.push_back(obs);
        }
    }
}



void SutengDriver::Start(){

    m_running = true;

    m_SutengDriverThread = std::thread( [this]() {

        
        PointCloud2Intensity::Ptr pInitPointCloud(new PointCloud2Intensity);
        // PointCloud2Intensity::Ptr pPointCloud(new PointCloud2Intensity);
        PointCloud2Intensity::Ptr pFilteredPointCloud(new PointCloud2Intensity);
        PointCloud2Intensity::Ptr pGroundCloud(new PointCloud2Intensity);
        PointCloud2Intensity::Ptr pObstacleCloud(new PointCloud2Intensity);

        //pcap包模式
        if(m_ST_Config->pcapRunningModel){
            m_driver.init(m_rs_param);
            m_processThread = std::thread(&SutengDriver::ProcessPcapCloud, this);
            m_driver.start();
        }

        if(m_ST_Config->onlineModel){
            m_driver.init(m_rs_param);
            m_processThread = std::thread(&SutengDriver::ProcessPcapCloud, this);
            m_driver.start();
        }


        //pcd模式
        if(m_ST_Config->pcdRunningModel)
        {
            std::string path = m_ST_Config->pcdPath; // 替换为你的实际文件路径
            std::cout<< "Loading PCD file from: " << path << std::endl;
            if ( m_pElevationMapGroundFilter && loadPointCloud(path, pInitPointCloud)) {
                std::cout << "Successfully loaded " << pInitPointCloud->points.size() << " points." << std::endl;

                // =====================================================
                // 使用 坐标转换 + 过滤
                // =====================================================

                // std::cout<<"test ToMain \n" << m_ST_AllLidarTransfromInfo->toMainLidarInfo.transformMartix << std::endl;
                // std::cout<<"test ToCar \n" << m_ST_AllLidarTransfromInfo->toCarInfo.transformMartix << std::endl;

                Eigen::Matrix4f R_CombinedTransMatrix = (m_ST_AllLidarTransfromInfo->toCarInfo.transformMartix 
                        * m_ST_AllLidarTransfromInfo->toMainLidarInfo.transformMartix);
                PointCloudTransform(pInitPointCloud,R_CombinedTransMatrix ,pFilteredPointCloud);
               
                auto t_start = std::chrono::steady_clock::now();

                // //统计滤波 // 太耗时
                // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);
                // pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
                // sor.setInputCloud(pFilteredPointCloud);
                // sor.setMeanK(50);            // 设置在进行统计分析时考虑的每个点的近邻数量
                // sor.setStddevMulThresh(1.0); // 设置标准差倍数阈值，距离超过该倍数的点将被视为离群点
                // sor.filter(*cloud_filtered); // 执行滤波，将结果存入 cloud_filtered

                // =====================================================
                // 使用 APMF 进行地面分割
                // =====================================================
                //一些测试发现，上坡时，会出现把低矮的障碍物当作地面，考虑废弃
                // std::pair<PointCloud2Intensity::Ptr, PointCloud2Intensity::Ptr> ground_obstacle_clouds = m_pCommonGroundDetection->CommonGroundSegmentStart(pInitPointCloud, 0);
                // PointCloud2Intensity::Ptr p_comm_no_Ground = ground_obstacle_clouds.first;
                // PointCloud2Intensity::Ptr p_comm_ground = ground_obstacle_clouds.second;
                auto t_end_1_ = std::chrono::steady_clock::now();
                auto duration_1 = std::chrono::duration_cast<std::chrono::milliseconds>(t_end_1_ - t_start);


                // // bool success = true;
                // bool success = m_pElevationMapGroundFilter->Process(p_no_Ground, pGroundCloud, pObstacleCloud);
                // // bool success = m_pElevationMapGroundFilter->Process(pFilteredPointCloud, pGroundCloud, pObstacleCloud);
                // // std::pair<PointCloud2Intensity::Ptr, PointCloud2Intensity::Ptr> ground_obstacle_clouds = m_pCommonGroundDetection->CommonGroundSegmentStart(cloud_filtered, 0);
                
                // =====================================================
                // 使用 Elevation Map Ground Filter 进行地面分割
                // =====================================================
                std::vector<GridCluster> outputClusters;
                // bool success = m_pElevationMapGroundFilter->ProcessWithObstacleDetection(p_comm_no_Ground, pGroundCloud, pObstacleCloud, outputClusters);
                bool success = m_pElevationMapGroundFilter->ProcessWithObstacleDetection(pFilteredPointCloud, pGroundCloud, pObstacleCloud, outputClusters);

                auto t_end = std::chrono::steady_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start);

                if (success)
                {
                    std::cout << " APMF 耗时 "<< duration_1.count() << " ms\n"
                        << " Total Ground Filter completed in " << duration.count() << " ms\n"
                        << "  Ground points:    " << pGroundCloud->size() << "\n"
                        << "  Obstacle points:  " << pObstacleCloud->size() << "\n"
                        << "*****Obstacle clusters: " << outputClusters.size() << std::endl;
                    
                    // SavePcd(pFilteredPointCloud);
                    // SavePcd(pFilteredPointCloud);
                    // for(size_t i = 0; i < outputClusters.size(); i++)
                    // {
                    //     std::cout <<" ------------------"<<std::endl;
                    //     std::cout << "Cluster id: " << outputClusters[i].id << ": "
                    //               << "Center(" << outputClusters[i].center_x << ", "
                    //               << outputClusters[i].center_y << ", "
                    //               << outputClusters[i].center_z << "), "
                                  
                    //               << std::endl;
                    // }

                    /* ---- 可视化已迁移至 main.cpp ----
                    // ============ 共用可视化 ============
                    InitGroundViewer();
                    UpdateGroundViewer(pGroundCloud, pObstacleCloud, outputClusters);
                    // UpdateGroundViewer(p_comm_ground, p_comm_no_Ground, outputClusters);
                    while (SpinGroundViewerOnce())
                    {
                        // 单帧模式下仅保持窗口打开，不做更新
                    }
                    ---- */

                    // ============ 帧缓冲：PCD单帧发布给主线程 ============
                    {
                        std::vector<TrackedObstacle> detections;
                        ConvertClustersToTrackedObstacles(outputClusters, detections);
                        m_visBuffer.Publish(pGroundCloud, pObstacleCloud, detections);
                    }
                }
                else
                {
                    std::cerr << "Ground Filter failed!" << std::endl;
                }
               
        
            }
        }


      
        
    
    });
}




}
