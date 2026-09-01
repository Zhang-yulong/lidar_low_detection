#include "SuTengDriver.h"
#include "debug_frame.h"
#include "debug_frame_queue.h"
#include <chrono>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

#include <atomic>
namespace Lidar_Low_Detection
{

std::atomic<unsigned long long> g_previousTimestamp(0);

bool test_EMX = true; 

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

        
        LOG_RAW("----------------------\n");
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


        // ── HDMap 过滤：Cluster 级软约束（Cluster 生成后、Tracker 之前）──
        // 迁移设计文档 Phase 1：只打标签（in_road/map_valid/map_confidence），不删除 Cluster。
        // 定位无效 / 地图未加载 / 查询失败 → HDMapFilter 内部自动降级为"保留全部"。
        if (m_hdmapEnabled)
        {
            // LocalizationManager::Pose loc_pose;
            // LocalizationManager::instance().getPose(loc_pose);

            have_pose = LocalizationManager::instance().getPose(loc_pose);

            if (have_pose && loc_pose.valid)
            {
                m_hdmapManager.buildDrivablePolygons(loc_pose.x, loc_pose.y, hdmap_polygons);
            }

            m_debugViewer->DrawClusterOverlay(*pGroundCloud, *pObstacleCloud,outputClusters,m_pElevationMapGroundFilter->GetConfig(),hdmap_polygons,loc_pose);


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
        else{
            m_debugViewer->DrawClusterOverlay(*pGroundCloud, *pObstacleCloud,outputClusters,m_pElevationMapGroundFilter->GetConfig(),hdmap_polygons,loc_pose);
        }






        // ── 跟踪器：为障碍物分配稳定的跨帧 ID（从 9999 起始）──
        {
            std::vector<TrackedObstacle> detections;
            ConvertClustersToTrackedObstacles(outputClusters, detections);
            m_tracker.update(detections, rec_timestamp_ms);
            // m_tracker.update_V2(detections, rec_timestamp_ms);

            // std::cout <<" ----[Tracker] stable IDs (from 9999)----"<<std::endl;
            for (const auto& track : m_tracker.vtrackings)
            {
                // std::cout << "Track id: " << track.id << ": "
                //           << "Center(" << track.pos_x << ", "
                //           << track.pos_y << ", "
                //           << track.pos_z << "), "
                //           << "age=" << track.age
                //           << std::endl;
                
                LOG_RAW("Track id: %d: Center(%.2f, %.2f, %.2f), age=%d\n", track.id, track.pos_x, track.pos_y, track.pos_z, track.age);
            }
        }

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

            if (have_pose && loc_pose.valid && !hdmap_polygons.empty())
            {
                m_debugViewer->DrawAllOverlay(*pGroundCloud, *pObstacleCloud,
                                           m_tracker.vtrackings,
                                           m_pElevationMapGroundFilter->GetConfig(),
                                           hdmap_polygons,
                                           loc_pose);
            }
            else
            {
                m_debugViewer->DrawTrackOverlay(*pGroundCloud, *pObstacleCloud,
                                           m_tracker.vtrackings,
                                           m_pElevationMapGroundFilter->GetConfig());
            }
        }

        auto t_end_5_ = std::chrono::steady_clock::now();
        auto duration_5 = std::chrono::duration_cast<std::chrono::milliseconds>(t_end_5_ - t_end_4_);


        if(m_ST_Config->pcapRunningModel){

            // SavePcd(pInputCloud);
            SavePcd(pFilteredPointCloud);
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

            // ============ 帧缓冲：发布数据给主线程渲染 ============
            m_visBuffer.Publish(pGroundCloud, pObstacleCloud, m_tracker.vtrackings);
        }
        auto t_end_6_ = std::chrono::steady_clock::now();
        auto duration_6 = std::chrono::duration_cast<std::chrono::milliseconds>(t_end_6_ - t_end_5_);


        if(m_ST_Config->onlineModel){
            newS2AviodObject outputObject;
            memset(&outputObject, 0, sizeof(outputObject));
            m_pElevationMapGroundFilter->ConvertTrackToS2ObstacleBox(m_tracker.vtrackings, rec_timestamp_ms, outputObject);

            
            int send_ret = sendUdpMsg(send_fd, (unsigned char*)&outputObject, sizeof(outputObject), ip, port);
            
            LOG_RAW("send timestamp=%llu return_val=%d obs_num=%d sizeof=%zu\n",
                outputObject.timestamp,
                outputObject.return_val,
                outputObject.obs_num,
                sizeof(newS2AviodObject));
        }

        // ── DebugFrame：一帧算法结果快照（Phase 2，纯新增，UDP 之后 push）──
        // 数据均为最终状态；不改变任何算法逻辑 / 执行时序 / UDP。
        {
            auto debugFrame = std::make_shared<DebugFrame>();
            debugFrame->timestamp_ms = rec_timestamp_ms;
            // 方案B：对复用的 pFilteredPointCloud 做一次独立深拷贝。
            // 注意：PCL 1.8 下 PointCloud2Intensity::Ptr = boost::shared_ptr，
            //       故用 Ptr(new ...) 构造（不能用 std::make_shared）。
            debugFrame->pointcloud = PointCloud2Intensity::Ptr(new PointCloud2Intensity(*pFilteredPointCloud));
            debugFrame->clusters   = outputClusters;        // 按值（含 HDMap 标签）
            debugFrame->trackings  = m_tracker.vtrackings;  // 按值（update 后最终结果）
            PushDebugFrame(std::move(debugFrame));
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
