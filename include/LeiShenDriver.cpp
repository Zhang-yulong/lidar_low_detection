#include "LeiShenDriver.h"
#include "Viewer.h"
#include "pcl/filters/radius_outlier_removal.h"
#include <boost/smart_ptr/make_shared_array.hpp>
#include <cstddef>

namespace Lidar_Low_Detection
{
std::string CH64w; 
std::string LS128;




LeiShenDrive::LeiShenDrive(const SELF_DEBUG_CONFIG &config, const STR_ALL_LIDAR_CONFIG_INFO &allLidarTransInfo)
	: m_stLSConfig(config), m_strAllLidarTransfromInfo(allLidarTransInfo)
{
	m_pViewer = nullptr;
    m_pGetLidarData = nullptr;
	m_pLidarCurbDetection = nullptr;
	m_pCommonGroundDetection = nullptr;
	m_iDataSockFd = -1;
	m_iDevSockFd = -1;
}


LeiShenDrive::~LeiShenDrive(){
	Stop();
	Free();

	delete(m_pViewer);
	delete(m_pGetLidarData);
	delete(m_pLidarCurbDetection);
	delete(m_pCommonGroundDetection);
}


void LeiShenDrive::Free(){
	std::cout<<" LS free "<<std::endl;
    m_pGetLidarData->LidarStop();
}


int LeiShenDrive::Init(){

	m_sComputerIP = m_stLSConfig.selfComputerIP;
    m_iMsopPort = m_stLSConfig.msopPort;
    m_iDifopPort = m_stLSConfig.difopPort;
    m_sLeiShenType = m_stLSConfig.LSlidarType;


   	if(m_sLeiShenType == "CH64w"){

		m_pViewer = new Viewer(m_stLSConfig);
		m_pGetLidarData = new GetLidarData_CH64w();
		// m_pGetLidarData = GetLidarData_CH64w*;
		m_pLidarCurbDetection = new LidarCurbDetection(m_stLSConfig);
		m_pCommonGroundDetection = new CommonGroundDetection(m_stLSConfig);
	}
        
    
    if(m_pGetLidarData == nullptr){
        printf("m_pGetLidarData = nullptr \r\n");
        return -1;
    }

// Fun fun = std::bind(&LeiShenDrive::callBackFunction, this, std::placeholders::_1, std::placeholders::_2);  //这种错误
    fun = std::bind(&LeiShenDrive::CallBackFunction, this, std::placeholders::_1, std::placeholders::_2);

    m_pGetLidarData->setCallbackFunction(&fun);

    m_pGetLidarData->LidarStar();

    m_running = true;

    m_dataSockThread = std::thread(&LeiShenDrive::GetDataSock, this);
    m_devSockThread = std::thread(&LeiShenDrive::GetDevSock, this);

	return 1;
}

void LeiShenDrive::CallBackFunction(std::vector<MuchLidarData> LidarDataValue, int)
{
	struct timeval tNow;
	gettimeofday(&tNow, NULL);

	// {
	// 	std::lock_guard<std::mutex> lock(m_DataMutex);
	// 	m_vLidarData.clear();
    // 	m_vLidarData = LidarDataValue;
   	// 	m_ullCatchTimeStamp = ((unsigned long long)tNow.tv_sec)*1000 + ((unsigned long long)tNow.tv_usec)/1000;
	// }

	m_DataMutex.lock();
	m_vLidarData.clear();
    m_vLidarData = LidarDataValue;
   	m_ullCatchTimeStamp = ((unsigned long long)tNow.tv_sec)*1000 + ((unsigned long long)tNow.tv_usec)/1000;
    m_DataMutex.unlock();
	// printf("(In leishen callback function) time = %lld, size = %d\n", m_ullCatchTimeStamp, LidarDataValue.size());
}


void LeiShenDrive::SavePcd(const PointCloud2Intensity::Ptr &InputCloud, const unsigned long long &ullTime)
{
	
	PointCloud2Intensity::Ptr filteredCloud(new PointCloud2Intensity());
	// 过滤点云，只保留强度大于给定阈值的点
    for (const auto& point : InputCloud->points)
    {
        if (point.intensity > m_stLSConfig.savePcdReflection && point.z < 0.0)  // 只保留强度大于阈值的点
        {
            pcl::PointXYZI filteredPoint;
            filteredPoint.x = point.x;
            filteredPoint.y = point.y;
            filteredPoint.z = point.z;
            filteredPoint.intensity = point.intensity;

            // 将过滤后的点添加到新的点云中
            filteredCloud->points.push_back(filteredPoint);
        }
    }

    // 如果过滤后的点云为空，则不保存
    if (filteredCloud->empty())
    {
        std::cout << "No points with intensity above threshold. Skipping saving." << std::endl;
        return;
    }

	// 设置新的点云的高度和宽度
    filteredCloud->height = filteredCloud->points.size();
    filteredCloud->width = 1;
    filteredCloud->is_dense = false;

  	InputCloud->height = InputCloud->points.size();
  	InputCloud->width = 1;
  	InputCloud->is_dense = false;
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
	sprintf(pchFileName, "/home/zyl/echiev_lidar_curb_detection/log/pcd/%lld.pcd", ullTime);
	pcl::io::savePCDFileASCII (pchFileName, *InputCloud);
	// pcl::io::savePCDFileASCII (pchFileName, *filteredCloud);
}



void LeiShenDrive::GetLidarType(const std::string LidarType){

    return ;
}


void LeiShenDrive::startGetDevSock(void *pth )
{
	((LeiShenDrive*)pth)->GetDevSock();
}

void LeiShenDrive::startGetDataSock(void *pth )
{
	((LeiShenDrive*)pth)->GetDataSock();
}

//获取数据包的端口号
void LeiShenDrive::GetDataSock()
{
	//******************UDP通信初始化**************************//
	//创建socket
	m_iDataSockFd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if( m_iDataSockFd < 0 ){
		LOG_RAW("create getData socket error\n");
		return;
	}
	else{
		LOG_RAW("create getData socket successful\n");
    }


	//端口复用
	int reuse = 1;

	if( setsockopt(m_iDataSockFd, SOL_SOCKET, SO_REUSEADDR, (const char *)&reuse, sizeof(reuse)) < 0){
		LOG_RAW("set getData socket error : reuse address\n");
		close(m_iDataSockFd);
		return;
	}
	else{
		LOG_RAW("set getData socket successful : reuse address\n");
	}

	if( setsockopt(m_iDataSockFd, SOL_SOCKET, SO_REUSEPORT, &reuse, sizeof(reuse)) < 0){
		LOG_RAW("set getData socket error : reuse port\n");
		close(m_iDataSockFd);
		return;
	}
	else{
		LOG_RAW("set getData socket successful : reuse port\n");
	}

	//定义地址
	struct sockaddr_in sockAddr;
	sockAddr.sin_family = AF_INET;
	sockAddr.sin_port = htons(m_iMsopPort);
	sockAddr.sin_addr.s_addr = inet_addr(m_sComputerIP.c_str());

	//绑定套接字
	if( bind(m_iDataSockFd, (struct sockaddr *)&sockAddr, sizeof(sockAddr)) < 0){
 		LOG_RAW("bind getData socket error\n");
 		close(m_iDataSockFd);
		return;
    }
    else{
    	LOG_RAW("bind getData socket successful\n");
    }

	struct sockaddr_in addrFrom;

    socklen_t len = sizeof(sockaddr_in);

	//接收数据
	char recvBuf[1212] = { 0 };
	int recvLen;

	while (m_running)
	{
        //获取套接字接收内容
        recvLen = recvfrom(m_iDataSockFd, recvBuf, sizeof(recvBuf), 0, (sockaddr*)&addrFrom, &len);
		// printf("recvLen: %d \n", recvLen);

		if (recvLen > 0)
		{
			u_char data[1212] = { 0 };
			memcpy(data, recvBuf, recvLen);
			m_pGetLidarData->CollectionDataArrive(data, recvLen);	//把数据传入到类内
			// printf("数据放入data\n");
		}
	}
	close(m_iDataSockFd);
	m_iDataSockFd = -1;
}

//获取设备包的端口号
void LeiShenDrive::GetDevSock()
{
	//******************UDP通信初始化**************************//
	//创建socket
	m_iDevSockFd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	if( m_iDevSockFd < 0 ){
		LOG_RAW("create Dev socket error\n");
		return;
	}
	else{
		LOG_RAW("create Dev socket successful\n");
    }


	//端口复用
	int reuse = 1;

	if( setsockopt(m_iDevSockFd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0)
	{
		LOG_RAW("set Dev socket error : reuse address\n");
		close(m_iDevSockFd);
		return;
	}
	else{
		LOG_RAW("set Dev socket successful : reuse address\n");
	}

	if( setsockopt(m_iDevSockFd, SOL_SOCKET, SO_REUSEPORT, (const char *)&reuse, sizeof(reuse)) < 0)
	{
		LOG_RAW("set Dev socket error : reuse port\n");
		close(m_iDevSockFd);
		return;
	}
	else{
		LOG_RAW("set Dev socket successful : reuse port\n");
	}

	//定义地址
	struct sockaddr_in sockAddr;
	sockAddr.sin_family = AF_INET;
	sockAddr.sin_port = htons(m_iDifopPort);		//获取设备包的端口号
	sockAddr.sin_addr.s_addr = inet_addr(m_sComputerIP.c_str());

	//绑定套接字
	if( bind(m_iDevSockFd, (struct sockaddr *)&sockAddr, sizeof(sockAddr)) < 0)
	{
		LOG_RAW("bind Dev socket error\n");
		close(m_iDevSockFd);
		return;
	}
	else{
		LOG_RAW("bind Dev socket successful\n");
	}
	
	struct sockaddr_in addrFrom;

    socklen_t len = sizeof(sockaddr_in);

	//接收数据
	char recvBuf[1212] = { 0 };
	int recvLen;

	while (m_running)
	{
        //获取套接字接收内容
        recvLen = recvfrom(m_iDevSockFd, recvBuf, sizeof(recvBuf), 0, (sockaddr*)&addrFrom, &len);

		if (recvLen > 0)
		{
			u_char data[1212] = { 0 };
			memcpy(data, recvBuf, recvLen);
			m_pGetLidarData->CollectionDataArrive(data, recvLen);//把数据传入到类内
		}
	}
	close(m_iDevSockFd);
	m_iDevSockFd = -1;
}

void LeiShenDrive::VoxelGridProcess(PointCloud2Intensity::Ptr &pOutputCloud){

	pcl::VoxelGrid<pcl::PointXYZI> sor;           	//创建滤波对象
	sor.setInputCloud (pOutputCloud);    			//设置需要过滤的点云给滤波对象
	sor.setLeafSize (0.05, 0.05, 0.05);             //设置滤波时创建的体素体积，单位m，因为保留点云大体形状的方式是保留每个体素的中心点，所以体素体积越大那么过滤掉的点云就越多
	sor.filter (*pOutputCloud);          			//执行滤波处理
	// std::cout<<"下采样后点云数量："<<pOutputCloud->points.size()<<std::endl;
}


void LeiShenDrive::GroundRemoval(PointCloud2Intensity::Ptr &fullCloud){

	size_t lowerInd, upperInd;

}


void LeiShenDrive::PointCloudTransform(std::vector<MuchLidarData> vTmpLidarData, PointCloud2Intensity::Ptr &pOutputCloud)
{
	Eigen::Matrix4f R_ToMainLidar = m_strAllLidarTransfromInfo.toMainLidarInfo.transformMartix;
	float fRad_ToMainLidar = m_strAllLidarTransfromInfo.toMainLidarInfo.rad;
	float fcos_ToMainLidar = cos(fRad_ToMainLidar);
	float fsin_ToMainLidar = sin(fRad_ToMainLidar);

	Eigen::Matrix4f R_ToCar = m_strAllLidarTransfromInfo.toCarInfo.transformMartix; 
	float fRad_ToCar = m_strAllLidarTransfromInfo.toCarInfo.rad;

	pcl::PointXYZI thisPoint;
	PointCloud2Intensity::Ptr pFullCloud, pFullInfoCloud;
	cv::Mat rangeMat;	//存储的是雷达点的欧几里得距离信息(x^2 + y^2 + z^2 )
	float verticalAngle, horizonAngle, range;
	size_t rowIdn, columnIdn, index;
	

	for(size_t i = 0; i < vTmpLidarData.size(); i++)
	{
		// std::cout<<"-------------"<<std::endl;
		Eigen::Matrix<float,4,1> x1_Init, x2_ToMainLidar, x3_ToCar, x4_Final;
		x1_Init << vTmpLidarData[i].X, vTmpLidarData[i].Y, vTmpLidarData[i].Z, 1;
		x2_ToMainLidar = R_ToMainLidar * x1_Init;
		pcl::PointXYZI tmppoint_ToMainLidar;
		tmppoint_ToMainLidar.x =  cos(fRad_ToMainLidar) * x2_ToMainLidar(0) + sin(fRad_ToMainLidar) * x2_ToMainLidar(1) ;
		tmppoint_ToMainLidar.y = -sin(fRad_ToMainLidar) * x2_ToMainLidar(0) + cos(fRad_ToMainLidar) * x2_ToMainLidar(1) ;
		tmppoint_ToMainLidar.z = x2_ToMainLidar(2) ;
		tmppoint_ToMainLidar.intensity = vTmpLidarData[i].Intensity;

		// pcl::PointXYZI tmppoint_ToMainLidar;
		// tmppoint_ToMainLidar.x =  cos(fRad_ToMainLidar) * x1_Init(0) + sin(fRad_ToMainLidar) * x1_Init(1) +  R_ToMainLidar();
		// tmppoint_ToMainLidar.y = -sin(fRad_ToMainLidar) * x1_Init(0) + cos(fRad_ToMainLidar) * x1_Init(1) +  ;
		// tmppoint_ToMainLidar.z = x1_Init(2) ;
		// tmppoint_ToMainLidar.intensity = vTmpLidarData[i].Intensity;




		x3_ToCar << tmppoint_ToMainLidar.x, tmppoint_ToMainLidar.y, tmppoint_ToMainLidar.z, 1;
		x4_Final = R_ToCar * x3_ToCar;


		// x4_Final = R_ToCar * R_ToMainLidar * x1_Init;
		// x4_Final = R_ToMainLidar * x1_Init;
		
		pcl::PointXYZI tmppoint_ToCar;
		tmppoint_ToCar.x = x4_Final(0);
		tmppoint_ToCar.y = x4_Final(1);
		tmppoint_ToCar.z = x4_Final(2);
		tmppoint_ToCar.intensity = vTmpLidarData[i].Intensity;


		/*在这里做一下Lego-Loam中的雷达线束id分类*/

		thisPoint.x = x4_Final(0);
		thisPoint.y = x4_Final(1);
		thisPoint.z = x4_Final(2);

		// rowIdn计算出该点激光雷达是竖直方向上第几线的（行索引）
		verticalAngle = vTmpLidarData[i].V_angle;
		rowIdn = vTmpLidarData[i].ID;
		
		// columnIdn计算水平方向(列索引)
		horizonAngle = vTmpLidarData[i].H_angle;
		columnIdn = static_cast<size_t>(horizonAngle / ang_res_x);
		if(columnIdn >= Horizon_SCAN){
			columnIdn = Horizon_SCAN - 1 ;	//防止越界
		}
		
		//存储到点云矩阵
		range = sqrt(tmppoint_ToCar.x * tmppoint_ToCar.x + tmppoint_ToCar.y * tmppoint_ToCar.y + tmppoint_ToCar.y + tmppoint_ToCar.z * tmppoint_ToCar.z);
		// rangeMat.at<float>(rowIdn, columnIdn) = range; //这里报错！！！会段错误
		//在范围图上保留点的位置关系，以便后续特征提取或数据对齐
		thisPoint.intensity = (float)rowIdn + (float)columnIdn / 10000.0;	//人为制造强度值; rowIdn 贡献整数部分，表示该点属于 哪一条激光线（哪一行）; columnIdn / 10000.0区分同一行内的不同点（防止数据丢失）
		index = columnIdn + rowIdn * Horizon_SCAN;	//将 2D 图像坐标（rowIdn, columnIdn）转换为 1D 数组索引

		// pFullCloud->points[index] = thisPoint;
		// pFullInfoCloud->points[index].intensity = range;

		pOutputCloud->points.push_back(tmppoint_ToCar);
	}

	pOutputCloud->width    	= pOutputCloud->points.size();
	pOutputCloud->height   	= 1;
	pOutputCloud->is_dense 	= false;

	// pFullCloud->width 		= pOutputCloud->points.size();
	// pFullCloud->height 		= 1;
	// pFullCloud->is_dense		= false;

	// pFullInfoCloud->width 	= pOutputCloud->points.size();
	// pFullInfoCloud->height	= 1;
	// pFullInfoCloud->is_dense = false;

	// GroundRemoval(pFullCloud);
}


unsigned long long LeiShenDrive::GetPointCloudFromOnline(PointCloud2Intensity::Ptr &pOutputCloud)
{
	unsigned long long ullTimestamp = 0;
	std::vector<MuchLidarData> vTmpLidarData;

	// {
	// 	std::lock_guard<std::mutex> lock(m_DataMutex);
	// 	vTmpLidarData = m_vLidarData;
	// 	ullTimestamp = m_ullCatchTimeStamp;
	// }
	m_DataMutex.lock();
	vTmpLidarData = m_vLidarData;
	ullTimestamp = m_ullCatchTimeStamp;
	m_DataMutex.unlock();
	PointCloudTransform(vTmpLidarData, pOutputCloud);
	
	return ullTimestamp;
}


unsigned long long LeiShenDrive::GetPointCloudFromPcd(PointCloud2Intensity::Ptr &pOutputCloud){

	PointCloud2Intensity::Ptr cloud(new PointCloud2Intensity);
	std::string sPcdPath = "/home/zyl/echiev_lidar_curb_detection/log/pcd/" + m_stLSConfig.pcdPathTime + ".pcd";

	unsigned long long ullTimestamp = stoull(m_stLSConfig.pcdPathTime);

	if(pcl::io::loadPCDFile(sPcdPath, *cloud) == -1){
		std::cerr << "Couldn't read the PCD file: " << sPcdPath << std::endl;
        return -1;
	}
	else{
		std::cout<<"read the PCD file: "<<sPcdPath<<std::endl;
	}

	for (int i = 0; i < cloud->points.size(); ++i) {
		pcl::PointXYZI temp;
		temp.x = cloud->points[i].x;
		temp.y = cloud->points[i].y;
		temp.z = cloud->points[i].z;
		temp.intensity = cloud->points[i].intensity;

        // cout << "Point " << i << ": "
        //      << cloud->points[i].x << " "
        //      << cloud->points[i].y << " "
        //      << cloud->points[i].z << endl;

		pOutputCloud->points.push_back(temp);
    }

	// ApplyRadiusOutlierFilter(pOutputCloud);

	return ullTimestamp;

}


void LeiShenDrive::ApplyRadiusOutlierFilter(PointCloud2Intensity::Ptr &InputCloud){

	pcl::RadiusOutlierRemoval<pcl::PointXYZI> radiusOutlierFilter;
	
	radiusOutlierFilter.setInputCloud(InputCloud);
	radiusOutlierFilter.setRadiusSearch(0.05);
	radiusOutlierFilter.setMinNeighborsInRadius(8);
	radiusOutlierFilter.filter(*InputCloud);
}


void LeiShenDrive::Start(){

    m_SutengDriveThread = std::thread( [this]() {  //lamda表达式创建线程

		bool bOnlineModel 			= m_stLSConfig.onlineModel;
		bool bPcapModel 			= m_stLSConfig.pcapRunningModel;
		bool bPcdModel 				= m_stLSConfig.pcdRunningModel;
		bool bSavePcdFlag 			= m_stLSConfig.savePcd;
		bool bCurbDetection			= m_stLSConfig.curbDetection;
		bool bGroundLoadDetection	= m_stLSConfig.groundLoadDetection;
		bool bProjectionModel 		= m_stLSConfig.projectionModel;
    	bool bSavePictureModel 		= m_stLSConfig.savePictureModel;

		PointCloud2Intensity::Ptr pPointCloud(new PointCloud2Intensity);

		if(bOnlineModel && !bPcapModel && !bPcdModel){
			LOG_RAW("在线模式\n");
		}
		else if(bPcapModel && !bOnlineModel && !bPcdModel){
			LOG_RAW("pcap读取模式\n");
			pcl_viewer = std::make_shared<pcl::visualization::PCLVisualizer>("LSPointCloudViewer");	//标题栏定义一个名称"LSPointCloudViewer"
			pcl_viewer->setBackgroundColor(0.0, 0.0, 0.0);	//黑色背景
			pcl_viewer->addCoordinateSystem(1.0);	//显示坐标系统方向，可以通过使用X（红色）、Y（绿色）、Z（蓝色）圆柱体代表坐标轴的显示方式来解决，圆柱体的大小通过scale参数控制
			pcl_viewer->addPointCloud<pcl::PointXYZI>(pPointCloud, "lslidar");	// 显示点云，
			pcl_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "lslidar"); // 显示自定义颜色数据，每个点云的RGB颜色字段

			// /*为地面检测线准备*/
			if(bGroundLoadDetection){

			
				ground_viewer = std::make_shared<pcl::visualization::PCLVisualizer>("GroundViewer");	
				ground_viewer->setBackgroundColor(0.0, 0.0, 0.0);
				ground_viewer->addCoordinateSystem(1.0);
			}
			
		}
		else if(bPcdModel && !bPcapModel && !bOnlineModel){
			LOG_RAW("pcd读取模式\n");
			// pcl_viewer = std::make_shared< pcl::visualization::PCLVisualizer>("LSPointCloudViewer");
			// pcl_viewer->setBackgroundColor(0.0, 0.0, 0.0);
			// pcl_viewer->addCoordinateSystem(1.0);
			// pcl_viewer->addPointCloud<pcl::PointXYZI>(pPointCloud, "lslidar");
			// pcl_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "lslidar");
		}
		else{
			LOG_RAW("模式未选择\n");
			return ;
		}


		
		if(bPcdModel)
		{
			LOG_RAW("读取pcd模式\n");
			/*读一张pcd*/
			m_ullUseTimeStamp = GetPointCloudFromPcd(pPointCloud);

			//	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> point_color_handle(pPointCloud, "intensity");
			//	pcl_viewer->updatePointCloud<pcl::PointXYZI>(pPointCloud, point_color_handle, "lslidar");
			// // pcl_viewer->spinOnce();
	
			// VoxelGridProcess(pPointCloud);
			if(bCurbDetection && !bGroundLoadDetection){

				auto segmentationResult = m_pLidarCurbDetection->GroundSegmentationStart(pPointCloud, m_ullUseTimeStamp);
				if(!segmentationResult.first->empty()){
					auto computeResult = m_pLidarCurbDetection->EdgeClusteringProcess(segmentationResult.first, m_ullUseTimeStamp);
				
					// //点云转图像
					if(bProjectionModel){
						m_pViewer->ProjectPointCloud(pPointCloud, segmentationResult.first, computeResult.first, computeResult.second);
						// m_pLS_Viewer->Display();
						if(bSavePictureModel)
							m_pViewer->SaveImage(m_ullUseTimeStamp);
					}    
				
				}
			
			}
			else if(!bCurbDetection && bGroundLoadDetection){
				m_pCommonGroundDetection->CommonGroundSegmentStart( pPointCloud, m_ullUseTimeStamp);

			}
			else{
				LOG_RAW("CurbDetection 和 GroundLoadDetection 参数错误！");
				return ;
			}
		}	



        while (m_running)
        {
			if(!m_vLidarData.empty())
			{	

				LOG_RAW("------------time: %lld , m_vLidarData size: %ld \n", m_ullCatchTimeStamp, m_vLidarData.size());
				pPointCloud->clear();
				
				/*	pcap回放*/
				if(bPcapModel){
					
					// std::vector<MuchLidarData> m_vLidarData_offline;
					// {
					// 	std::lock_guard<std::mutex> lock(m_DataMutex);
					// 	m_vLidarData_offline = m_vLidarData;
					// 	m_ullUseTimeStamp = m_ullCatchTimeStamp;
					// }

					m_DataMutex.lock();
					std::vector<MuchLidarData> m_vLidarData_offline;
					m_vLidarData_offline = m_vLidarData;
					m_vLidarData.clear();
					m_ullUseTimeStamp = m_ullCatchTimeStamp;
					m_DataMutex.unlock();

					PointCloudTransform(m_vLidarData_offline, pPointCloud);

					// ApplyRadiusOutlierFilter(pPointCloud);

					pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> point_color_handle(pPointCloud, "intensity");
					pcl_viewer->updatePointCloud<pcl::PointXYZI>(pPointCloud, point_color_handle, "lslidar");
					pcl_viewer->spinOnce();

				}
				
				/*	现场小车跑*/
				if(bOnlineModel){
					
					m_ullUseTimeStamp = GetPointCloudFromOnline(pPointCloud); //涉及数据接收 + 雷达点云坐标转换, 这里pPointCloud是输出
				}	

				/*	保存点云*/
				if(bSavePcdFlag){
					LOG_RAW("保存点云\n");
					SavePcd(pPointCloud, m_ullUseTimeStamp); 
				}
				

				/*先使用体素滤波向下采样处理点云*/
				// VoxelGridProcess(pPointCloud);

				//数据处理+计算
				if(bCurbDetection && !bGroundLoadDetection){
					auto segmentationResult = m_pLidarCurbDetection->GroundSegmentationStart(pPointCloud, m_ullUseTimeStamp);
					
					if(segmentationResult.first != nullptr){
						if(!segmentationResult.first->empty()){
							auto computeResult = m_pLidarCurbDetection->EdgeClusteringProcess(segmentationResult.first, m_ullUseTimeStamp);
					
							// //点云转图像
							if(bProjectionModel){
								m_pViewer->ProjectPointCloud(pPointCloud, segmentationResult.first, computeResult.first, computeResult.second);
								// m_pLS_Viewer->Display();
								if(bSavePictureModel)
									m_pViewer->SaveImage(m_ullUseTimeStamp);
							}    
						}
						else{
							LOG_RAW("地面上的点云数量为0\n");
							continue;
						}
					}
					else{
						LOG_RAW("地面分割失败，指针为空\n");
						continue;
					}
				
				}
				else if(!bCurbDetection && bGroundLoadDetection){
					
					auto segmentationResult = m_pCommonGroundDetection->CommonGroundSegmentStart(pPointCloud, m_ullUseTimeStamp);
					
					if(segmentationResult.second != nullptr){

						if(!segmentationResult.second->empty()){

							ground_viewer->addPointCloud<pcl::PointXYZI>(segmentationResult.second, "groundlidar");
							ground_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "groundlidar");
							pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> point_color_handle(segmentationResult.second, "intensity");
							ground_viewer->updatePointCloud<pcl::PointXYZI>(segmentationResult.second, point_color_handle, "groundlidar");
							ground_viewer->spinOnce();

							auto computeResult = m_pLidarCurbDetection->EdgeClusteringProcess(segmentationResult.second, m_ullUseTimeStamp);
							// //点云转图像
							if(bProjectionModel){
								m_pViewer->ProjectPointCloud(pPointCloud, segmentationResult.second, computeResult.first, computeResult.second);
								// m_pLS_Viewer->Display();
								if(bSavePictureModel)
									m_pViewer->SaveImage(m_ullUseTimeStamp);
							} 
						}	
						else{
							LOG_RAW("地底的点云数量为0\n");
							continue;
						}   
					}
					else{
						LOG_RAW("地面分割失败，指针为空\n");
						continue;
					}
				}
				else{
					LOG_RAW("CurbDetection 和 GroundLoadDetection 参数错误！");
					return ;
				}
				
			}
			else{
				// LOG_RAW("m_vLidarData empty!!!\n");
				std::this_thread::sleep_for(std::chrono::milliseconds(1));
			}
			
        }
    });
}


void LeiShenDrive::Stop()
{
    m_running = false;

    // 关闭 socket 以解除 recvfrom 阻塞
    if (m_iDataSockFd >= 0)
    {
        shutdown(m_iDataSockFd, SHUT_RDWR);
        close(m_iDataSockFd);
        m_iDataSockFd = -1;
    }
    if (m_iDevSockFd >= 0)
    {
        shutdown(m_iDevSockFd, SHUT_RDWR);
        close(m_iDevSockFd);
        m_iDevSockFd = -1;
    }

    // 等待所有线程退出
    if (m_dataSockThread.joinable())
    {
        m_dataSockThread.join();
    }
    if (m_devSockThread.joinable())
    {
        m_devSockThread.join();
    }
    if (m_SutengDriveThread.joinable())
    {
        m_SutengDriveThread.join();
    }
}



}