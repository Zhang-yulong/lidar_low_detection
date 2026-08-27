#include <iostream>
// #include <memory>
#include <condition_variable>
#include <signal.h>
#include <atomic>
#include <chrono>
#include <pcl/visualization/pcl_visualizer.h>
#include "LeiShenDriver.h"
#include "SuTengDriver.h"
#include "UdpCommunication.h"
#include "ulog_api.h"
#include "localization_manager.h"
#include "pcap_localization_feed.h"
#include "coordinate_transformer.h"


using namespace Lidar_Low_Detection;

#define CvLane_MC_PORT 9139

// 在CMakeLists.txt和Makefile中定义了LOG_CFG_FILE_PATH
// #define LOG_CFG_FILE_PATH "/home/zyl/echiev_low_lidar_detection/config/ulog.cfg"
// #define LOG_CFG_FILE_PATH "/etc/ulog/curb/config/ulog.cfg"
// #define LOG_CFG_FILE_PATH "/etc/echiev/low_detection/ulog.cfg"

namespace Lidar_Low_Detection {
    extern bool test_EMX;
}

// 默认使用 /etc/echiev/low_detection/debug_config.yaml，可通过命令行 -f <path> 覆盖
// std::string Yaml_Path = "/home/zyl/echiev_low_lidar_detection/config/debug_config.yaml";
std::string Yaml_Path = "/etc/echiev/low_detection/debug_config.yaml";

std::unique_ptr<STR_LIDAR_CONFIG> strToMainLidarConfig = std::make_unique<STR_LIDAR_CONFIG>();
std::unique_ptr<STR_LIDAR_CONFIG> strToCarConfig = std::make_unique<STR_LIDAR_CONFIG>();

STR_FUSIONLOC g_strFusionLocation;
pthread_mutex_t g_SpeedMutex;
pthread_mutex_t g_ThreadMutex;
float  g_fSpeed = 0.0;

// pcap 回放模式下：从同一 pcap 解析 9110 定位报文喂给 LocalizationManager
// （rs_driver 回放不经过网络栈，OpenfusionLocMCClient 的 9110 socket 收不到数据）
PcapLocalizationFeed g_pcapLocFeed;

// ================================================================
// 信号处理：SIGINT (Ctrl+C) 优雅退出
// ================================================================
std::atomic<bool>* g_TerminateFlag = nullptr;  // 指向主线程中的标志

static void SignalHandler(int Sig)
{
    switch (Sig)
    {
        case SIGINT:
        case SIGTERM:
        case SIGQUIT:
        case SIGHUP:
        {
            printf("\nSignal %d received, shutting down...\n", Sig);
            if (g_TerminateFlag)
                g_TerminateFlag->store(true, std::memory_order_release);
            break;
        }
        default:
            // 忽略未处理的信号
            break;
    }
}

static void* SignalThread(void* Arg)
{
	// 【关键修复】第一行就赋值，防止空指针解引用
    g_TerminateFlag = static_cast<std::atomic<bool>*>(Arg);

    sigset_t set;
    sigemptyset(&set);
    sigaddset(&set, SIGINT);
    sigaddset(&set, SIGTERM);
    sigaddset(&set, SIGQUIT);
    sigaddset(&set, SIGHUP);
    // 注意：不要加 SIGKILL，它无法被捕获

    int sig;
    while (!g_TerminateFlag->load(std::memory_order_acquire))
    {
        int ret = sigwait(&set, &sig);
        if (ret == 0)
        {
            SignalHandler(sig);
        }
        else
        {
            // sigwait 被信号中断（EINTR），重试
            if (errno != EINTR)
            {
                perror("sigwait error");
                break;
            }
        }
    }
    return nullptr;
}



// void setSpeedInfo(float fSpeed)
// {
// 	pthread_mutex_lock( &g_SpeedMutex);
// 	g_fSpeed = fSpeed;
// 	pthread_mutex_unlock( &g_SpeedMutex );
// }
// void speed_set(unsigned char *pData, int Len)
// {
//     STR_CAN_SPEED canSpeed;
// 	memset(&canSpeed, 0, sizeof(STR_CAN_SPEED));
// 	memcpy(&canSpeed, pData, Len);
// 	setSpeedInfo(canSpeed.fCanSpeed);
// }

void SetFusionLocation (STR_FUSIONLOC*  pstrFusionLocation )
{
/*
	API_LOG (LOG_ERR, "rev fusionLoc data, long %.15lf %.15lf \n",
	pstrFusionLocation->strImu.dLongitude, pstrFusionLocation->strImu.dLatitude);
	*/
	pthread_mutex_lock( &g_ThreadMutex );
	memcpy(&g_strFusionLocation,pstrFusionLocation, sizeof(STR_FUSIONLOC));
	pthread_mutex_unlock( &g_ThreadMutex );
}
int GetFusionLocation (STR_FUSIONLOC*  pstrFusionLocation)
{
	pthread_mutex_lock( &g_ThreadMutex );
	memcpy(pstrFusionLocation,&g_strFusionLocation,sizeof(STR_FUSIONLOC) );
	pthread_mutex_unlock( &g_ThreadMutex );
	return 0;
}
void fusionLoc_set(unsigned char *pData, int Len)
{
	if ( Len >= sizeof(STR_FUSIONLOC) )
	{
		STR_FUSIONLOC* pstrFusionLocation = (STR_FUSIONLOC*)pData;
		SetFusionLocation( pstrFusionLocation );
		// 通知 LocalizationManager（内部解析 STR_FUSIONLOC 并记录新鲜度）
		Lidar_Low_Detection::LocalizationManager::instance().onDataReceived(pData, Len);
		//printf ("cccccccc rev fusionLoc data, len ok =%d \n", Len);
		
	} else{
		printf ("cccccccc rev fusionLoc data, len not ok =%d \n", Len);
	}
}



int CommInit(const SELF_DEBUG_CONFIG &config)
{
	ConfigReadError iRet_1, iRet_2;

	if(config.pathTolidarTransformConfig == 0){
		const char *PathToMainLidarConfig = "/home/zyl/echiev_lidar_curb_detection/config/小车11上的雷达配置文件/lsCH64w_front/lidar.cfg";
		const char *PathToCarConfig = "/home/zyl/echiev_lidar_curb_detection/config/小车11上的雷达配置文件/lidar.cfg";
		iRet_1 = readConfigFile(strToMainLidarConfig.get(), PathToMainLidarConfig);
		iRet_2 = readConfigFile(strToCarConfig.get(), PathToCarConfig);
	}
	else if(config.pathTolidarTransformConfig == 1){
		const char *PathToMainLidarConfig = "/etc/echiev/lidar/lsCH64w_front/lidar.cfg";
		const char *PathToCarConfig = "/etc/echiev/lidar/lidar.cfg";
		iRet_1 = readConfigFile(strToMainLidarConfig.get(), PathToMainLidarConfig);
		iRet_2 = readConfigFile(strToCarConfig.get(), PathToCarConfig);
	}
	else if(config.pathTolidarTransformConfig == 2){
		const char *PathToMainLidarConfig = "/home/zyl/echiev_lidar_curb_detection/config/佛山T1上的雷达配置文件/lsCH64w_front/lidar.cfg";
		const char *PathToCarConfig = "/home/zyl/echiev_lidar_curb_detection/config/佛山T1上的雷达配置文件/lidar.cfg";
		iRet_1 = readConfigFile(strToMainLidarConfig.get(), PathToMainLidarConfig);
		iRet_2 = readConfigFile(strToCarConfig.get(), PathToCarConfig);
	}
	else if (config.pathTolidarTransformConfig == 3) {
		const char *PathToMainLidarConfig = "/etc/echiev/lidar/lsCH64w_front/lidar.cfg";
		const char *PathToCarConfig = "/etc/echiev/lidar/lidar.cfg";
		iRet_1 = readConfigFile(strToMainLidarConfig.get(), PathToMainLidarConfig);
		iRet_2 = readConfigFile(strToCarConfig.get(), PathToCarConfig);
	}
	else if (config.pathTolidarTransformConfig == 4) {
		const char *PathToMainLidarConfig = "/home/zyl/echiev_low_lidar_detection/config/T05-lidar-shenzhen/rsairy_right_front/lidar.cfg";
		const char *PathToCarConfig = "/home/zyl/echiev_low_lidar_detection/config/T05-lidar-shenzhen/lidar.cfg";
		iRet_1 = readConfigFile(strToMainLidarConfig.get(), PathToMainLidarConfig);
		iRet_2 = readConfigFile(strToCarConfig.get(), PathToCarConfig);
	}
	else if (config.pathTolidarTransformConfig == 5) {
		const char *PathToMainLidarConfig = "/home/zyl/echiev_low_lidar_detection/config/T05-lidar-shenzhen/rsairy_left_front/lidar.cfg";
		const char *PathToCarConfig = "/home/zyl/echiev_low_lidar_detection/config/T05-lidar-shenzhen/lidar.cfg";
		iRet_1 = readConfigFile(strToMainLidarConfig.get(), PathToMainLidarConfig);
		iRet_2 = readConfigFile(strToCarConfig.get(), PathToCarConfig);
	}
	else if (config.pathTolidarTransformConfig == 6) {
		const char *PathToMainLidarConfig = "/home/zyl/echiev_low_lidar_detection/config/T05-lidar-hainan/rsairy_right_front/lidar.cfg";
		const char *PathToCarConfig = "/home/zyl/echiev_low_lidar_detection/config/T05-lidar-hainan/lidar.cfg";
		iRet_1 = readConfigFile(strToMainLidarConfig.get(), PathToMainLidarConfig);
		iRet_2 = readConfigFile(strToCarConfig.get(), PathToCarConfig);
	}
	else if (config.pathTolidarTransformConfig == 7) {
		const char *PathToMainLidarConfig = "/home/zyl/echiev_low_lidar_detection/config/T05-lidar-hainan/rsairy_left_back/lidar.cfg";
		const char *PathToCarConfig = "/home/zyl/echiev_low_lidar_detection/config/T05-lidar-hainan/lidar.cfg";
		iRet_1 = readConfigFile(strToMainLidarConfig.get(), PathToMainLidarConfig);
		iRet_2 = readConfigFile(strToCarConfig.get(), PathToCarConfig);
	}
	else if(config.pathTolidarTransformConfig == 8){
		const char *PathToMainLidarConfig = "/etc/echiev/lidar/rsairy_right_front/lidar.cfg";
		const char *PathToCarConfig = "/etc/echiev/lidar/lidar.cfg";
		iRet_1 = readConfigFile(strToMainLidarConfig.get(), PathToMainLidarConfig);
		iRet_2 = readConfigFile(strToCarConfig.get(), PathToCarConfig);
	}
	else if(config.pathTolidarTransformConfig == 9){
		const char *PathToMainLidarConfig = "/etc/echiev/lidar/rs16p_right_front/lidar.cfg";
		const char *PathToCarConfig = "/etc/echiev/lidar/lidar.cfg";
		iRet_1 = readConfigFile(strToMainLidarConfig.get(), PathToMainLidarConfig);
		iRet_2 = readConfigFile(strToCarConfig.get(), PathToCarConfig);
	}
	else if(config.pathTolidarTransformConfig == 10){
		const char *PathToMainLidarConfig = "/home/zyl/echiev_low_lidar_detection/config/E1-lidar-hainan/rs16p_right_front/lidar.cfg";
		const char *PathToCarConfig = "/home/zyl/echiev_low_lidar_detection/config/E1-lidar-hainan/lidar.cfg";
		iRet_1 = readConfigFile(strToMainLidarConfig.get(), PathToMainLidarConfig);
		iRet_2 = readConfigFile(strToCarConfig.get(), PathToCarConfig);
	}
	else if(config.pathTolidarTransformConfig == 11){
		const char *PathToMainLidarConfig = "/home/zyl/echiev_low_lidar_detection/config/T05-lidar-shenzhen/lidar.cfg";
		const char *PathToCarConfig = "/home/zyl/echiev_low_lidar_detection/config/T05-lidar-shenzhen/lidar.cfg";
		iRet_1 = readConfigFile(strToMainLidarConfig.get(), PathToMainLidarConfig);
		iRet_2 = readConfigFile(strToCarConfig.get(), PathToCarConfig);
	}
	else if(config.pathTolidarTransformConfig == 12){
		const char *PathToMainLidarConfig = "/etc/echiev/lidar/lidar.cfg";
		const char *PathToCarConfig = "/etc/echiev/lidar/lidar.cfg";
		iRet_1 = readConfigFile(strToMainLidarConfig.get(), PathToMainLidarConfig);
		iRet_2 = readConfigFile(strToCarConfig.get(), PathToCarConfig);
	}
	else{
		LOG_RAW("**** 2 **** [Error]read 《debug_config.yaml》 中的 pathTolidarTransformConfig 参数错误 \n");
		return -1;
	}	

	if(CFG_READ_SUCCESS != iRet_1){
		LOG_RAW("**** 3 **** [Error] 找不到/读 ToMainLidar Config File error, errcode:%d\n", iRet_1);
		return -1;
	}
	else{
		LOG_RAW("**** 3 **** read ToMainLidar Config File successful\n");
	}

	if(CFG_READ_SUCCESS != iRet_2){
		LOG_RAW("**** 4 **** [Error] 找不到/读 ToCar Config File (lidar文件夹最外层lidar.cfg) error, errcode:%d\n", iRet_2);
		return -1;
	}
	else{
		LOG_RAW("**** 4 **** read ToCar Config File successful\n");
	}


	/*
	if(OpenTimeMCClient(TIME_MC_PORT, 0) < 0) 
	{
		printf("open Time client fail\n");
	}
	*/

	/*
	pthread_mutex_init(&g_SpeedMutex, NULL);
	if(OpenVSpdMCClient(VSPD_MC_PORT, 0) < 0){
		LOG_RAW("**** 5 **** OpenVSpdMCClient fail.\n");
		return -1;
	}
	else{
		LOG_RAW("OpenVSpdMCClient successful.\n");
	}
	vehicle_speed_set_callback(speed_set);
	*/

	/*
	if(OpenLidarMCClient(LIDAR_MC_PORT, 0) < 0)  //读：雷达接收端口，9106
	{
		printf("OpenLidarMCClient failed\n");
		return -1;
	}
	Lidar_set_callback(Lidar_set);	
	*/
	
	/*接入定位模块
	 * 仅在 pcap / online 模式下启用；pcd 单帧调试不启用（见迁移设计文档第 6/9 章）
	 * 需同时满足: mapFilterModel(HDMap总开关) && Localization.enable && 非pcd模式
	 *
	 * 降级策略（见迁移文档第 27/28 章）：定位订阅失败 不退出 低矮检测。
	 * 定位库(OpenfusionLocMCClient)初始化失败 → 不订阅，HDMap 过滤自动关闭，
	 * 低矮障碍物检测照常运行。
	 */
	if( config.mapFilterModel && config.localizationEnable &&
	    (config.pcapRunningModel || config.onlineModel) && !config.pcdRunningModel){
		pthread_mutex_init(&g_ThreadMutex, NULL);
		if(OpenfusionLocMCClient(FUSIONLOC_MC_PORT, 0) < 0){
			LOG_RAW("**** 6 **** OpenfusionLocMCClient fail. localization disabled (degraded, low-obstacle detection continues).\n");
			// 降级：返回成功，继续低矮检测（HDMap 过滤因定位无效自动关闭）
		}
		else{
			LOG_RAW("OpenfusionLocMCClient successful.\n");
			fusionLoc_set_callback(fusionLoc_set);
		}
	}

	/*
	if(OpenCameraLaneMCServer(CvLane_MC_PORT, CvLane_MC_INTERVAL, 0X01) < 0){ 
		LOG_RAW("**** 7 **** OpenCameraLaneMCServer fail.\n");
		return -1;
	}
	else{
		LOG_RAW("OpenCameraLaneMCServer successful.\n");
	}
	*/
	return (CFG_READ_SUCCESS == iRet_1 && CFG_READ_SUCCESS == iRet_2) ? 1 : -1;
}

void Comm_Exit(void)
{
	// CloseTimeMCClient();
	// CloseVSpdMCClient();
	// CloseLidarMCClient();
	ClosefusionLocMCClient();
	// CloseCameraLaneMCServer();
	ulog_deinit();
}


STR_ALL_LIDAR_CONFIG_INFO GetAllTransformConfigInfo(const STR_LIDAR_CONFIG *toMainLidar, const STR_LIDAR_CONFIG *toCar){

	STR_LIDAR_CONFIG_INFO toMainLidarInfo, toCarInfo;

	Eigen::Matrix<float,4,4> R_ToMainLidar = Eigen::Matrix4f::Identity();
	R_ToMainLidar <<	toMainLidar->fComBinePara[0], toMainLidar->fComBinePara[1], toMainLidar->fComBinePara[2], toMainLidar->fComBinePara[3],
					 	-toMainLidar->fComBinePara[4], -toMainLidar->fComBinePara[5], -toMainLidar->fComBinePara[6], -toMainLidar->fComBinePara[7],
					 	toMainLidar->fComBinePara[8], toMainLidar->fComBinePara[9],	toMainLidar->fComBinePara[10], toMainLidar->fComBinePara[11],
					 	0, 0, 0, 1;
	

	// R_ToMainLidar <<	toMainLidar->fComBinePara[0], toMainLidar->fComBinePara[1], toMainLidar->fComBinePara[2], toMainLidar->fComBinePara[3],
	// 				 	toMainLidar->fComBinePara[4], toMainLidar->fComBinePara[5], toMainLidar->fComBinePara[6], toMainLidar->fComBinePara[7],
	// 				 	toMainLidar->fComBinePara[8], toMainLidar->fComBinePara[9],	toMainLidar->fComBinePara[10], toMainLidar->fComBinePara[11],
	// 				 	0, 0, 0, 1;
	
	std::cout<<"R_ToMainLidar = "<<R_ToMainLidar<<std::endl;
	// LOG_RAW(" ***********\n"\
	// 	"R_ToMainLidar： \n" \
	// 	"%.6f, %.6f, %.6f, %.6f \n",
	// 	toMainLidar->fComBinePara[0], toMainLidar->fComBinePara[1], toMainLidar->fComBinePara[2], toMainLidar->fComBinePara[3]
	// );	
	// LOG_RAW("%.6f, %.6f, %.6f, %.6f \n",
	// 	-toMainLidar->fComBinePara[4], -toMainLidar->fComBinePara[5], -toMainLidar->fComBinePara[6], -toMainLidar->fComBinePara[7]
	// );
	// LOG_RAW("%.6f, %.6f, %.6f, %.6f \n",
	// 	toMainLidar->fComBinePara[8], toMainLidar->fComBinePara[9],	toMainLidar->fComBinePara[10], toMainLidar->fComBinePara[11]
	// );
	// LOG_RAW("%f, %f, %f, %f \n",
	// 	0, 0, 0, 1
	// );


	toMainLidarInfo.transformMartix = R_ToMainLidar;




	float fAngle_ToCar = toCar->fLidar2Vehicle_Heading;
	std::cout<<"fAngle_ToCar = "<<fAngle_ToCar<<std::endl;
	float fRad_ToCar = (fAngle_ToCar * M_PI) / 180;

	Eigen::Matrix<float,4,4> R_ToCar = Eigen::Matrix4f::Identity();
	R_ToCar(0,0) = cos(fRad_ToCar);
	R_ToCar(0,1) = sin(fRad_ToCar);
	R_ToCar(0,3) = toCar->fComBinePara[3];

	R_ToCar(1,0) = -sin(fRad_ToCar);
	R_ToCar(1,1) = cos(fRad_ToCar);
	R_ToCar(0,3) = toCar->fLidar2Vehicle_X;
	R_ToCar(1,3) = toCar->fLidar2Vehicle_Y;

//标准逆时针旋转矩阵
// R_ToCar(0,1) = -sin(fRad_ToCar);	
// R_ToCar(1,0) = +sin(fRad_ToCar);
	

	std::cout<<"R_ToCar = "<<R_ToCar<<std::endl; 
	toCarInfo.transformMartix = R_ToCar;

	
	float AiryGroundHeight = static_cast<float>(toMainLidar->fGroundHeight); //里面的都是假的值
	toMainLidarInfo.reviseGroudHeight = toMainLidar->fComBinePara[11];  //这个只是airy到主雷达的高度
	
	float MainLidarGroundHeight = static_cast<float>(toCar->fGroundHeight); //只有最外层时lidar.cfg是正确的
	toCarInfo.reviseGroudHeight = MainLidarGroundHeight;
	LOG_RAW("外层lidar.cfg读取 fGroundHeight = %.3f\n", toCarInfo.reviseGroudHeight);

	toMainLidarInfo.rsairy_iInstallType = toMainLidar->iInstallType;


		
	if(test_EMX){
		Eigen::Matrix<float,4,4> R_EMXToMainLidar = Eigen::Matrix4f::Identity();
		R_EMXToMainLidar <<	0.999959, 0.00215855, 0.0087388, 0.136121,
					 	0.00194582, -0.999703, 0.0242785, 0.00555958,
					 	-0.00878861, 0.0242605,	0.999667, -0.829017,
					 	0, 0, 0, 1;
		toMainLidarInfo.transformMartix = R_EMXToMainLidar;
		toMainLidarInfo.reviseGroudHeight = 0.0;
		// toCarInfo.transformMartix = Eigen::Matrix4f::Identity();
		// toCarInfo.reviseGroudHeight = -0.40;
	}


	// 【HDMap 航向补偿】主雷达系(Grid) 与 融合IMU系 前向夹角 = 90° + fLidar2Vehicle_Heading
	// （点云 R_Combined 净旋转 = 90°+fLidar2Vehicle_Heading，见 docs/HDMap可视化.md 第五节）
	float fLidar2GridHeadingCorrectionDeg = 90.0f + fAngle_ToCar;
	LOG_RAW("fLidar2GridHeadingCorrectionDeg = %.4f  ( = 90 + fLidar2Vehicle_Heading )\n",
	        fLidar2GridHeadingCorrectionDeg);

	STR_ALL_LIDAR_CONFIG_INFO retInfo;
	retInfo.toMainLidarInfo = toMainLidarInfo;
	retInfo.toCarInfo       = toCarInfo;
	retInfo.fLidar2GridHeadingCorrectionDeg = fLidar2GridHeadingCorrectionDeg;
	return retInfo;
}









int main(int argc, char *argv[])
{
	// 命令行参数解析：./low_detection -f <path>
	// 无参数时使用默认 Yaml_Path；有 -f 参数时使用参数指定的 yaml 路径
	for (int i = 1; i < argc; ++i)
	{
		std::string arg = argv[i];
		if (arg == "-f" && i + 1 < argc)
		{
			Yaml_Path = argv[i + 1];
			++i;
		}
	}
	std::cout << "Yaml_Path = " << Yaml_Path << std::endl;

	// 旧ulog
	int iRet = ulog_init(LOG_CFG_FILE_PATH);
	// std::cout<<"ulog iRet: " <<iRet<<std::endl; //读取成功是0
	if(iRet){
		LOG_RAW("**** 0 **** [Error] %s 加载失败\n", LOG_CFG_FILE_PATH);
	}
	// ulog_init(argc, argv); //新ulog
	
	
	SELF_DEBUG_CONFIG strOpencvConfig;
	std::unique_ptr<YamlReader> pYamlReader = std::make_unique<YamlReader>(Yaml_Path);
    if (!pYamlReader->LoadConfig(strOpencvConfig)) {
		LOG_RAW("**** 1 **** [Error] debug_config.yaml加载失败\n");
        return -1; // 如果加载失败，则退出
    }
	
	
	if(CommInit(strOpencvConfig) != 1)
	{
		LOG_RAW("[Error] CommInit函数初始化失败\n");
		return -1;
	}

	// 配置 LocalizationManager（与 CommInit 中的定位订阅开关保持一致）
	{
		const bool loc_enable = (strOpencvConfig.mapFilterModel != 0) &&
		                        (strOpencvConfig.localizationEnable != 0) &&
		                        (strOpencvConfig.pcapRunningModel || strOpencvConfig.onlineModel) &&
		                        !strOpencvConfig.pcdRunningModel;
		LocalizationManager::instance().configure(
			loc_enable,
			strOpencvConfig.localizationTimeoutMs,
			strOpencvConfig.localizationDebugEnable != 0,
			strOpencvConfig.localizationDebugX,
			strOpencvConfig.localizationDebugY,
			strOpencvConfig.localizationDebugHeading);
	}

	// 1. 设置信号屏蔽集
	sigset_t sigset;
	sigemptyset(&sigset);
	sigaddset(&sigset, SIGINT);
	sigaddset(&sigset, SIGTERM);
	sigaddset(&sigset, SIGQUIT);
	sigaddset(&sigset, SIGHUP);
	pthread_sigmask(SIG_BLOCK, &sigset, nullptr);

	
	STR_ALL_LIDAR_CONFIG_INFO strAllLidarTransformInfo = GetAllTransformConfigInfo(strToMainLidarConfig.get(), strToCarConfig.get());

	// 【HDMap 航向补偿】由 lidar.cfg fLidar2Vehicle_Heading 自动计算（90°+fLidar2Vehicle_Heading），
	// 注入 CoordinateTransformer：可视化与过滤统一应用（勿写死，换雷达自动适配）
	CoordinateTransformer::setGridHeadingOffsetDeg(strAllLidarTransformInfo.fLidar2GridHeadingCorrectionDeg);
	LOG_RAW("CoordinateTransformer gridHeadingOffset = %.4f deg (from lidar.cfg)\n",
	        CoordinateTransformer::gridHeadingOffsetDeg());

   	// LeiShenDrive lsDrive(strOpencvConfig, strAllLidarTransformInfo);
	// lsDrive.Init();
	// lsDrive.Start();
	
	SutengDriver stDriver(strOpencvConfig, strAllLidarTransformInfo);
	stDriver.Init();
	stDriver.Start();

	// pcap 回放模式下：独立线程从同一 pcap 解析 9110 定位报文喂给 LocalizationManager
	// （rs_driver 回放不经过内核网络栈，9110 socket 收不到数据；在线模式 9110 由真实
	//  组播送入 OpenfusionLocMCClient，无需本模块）
	if (strOpencvConfig.pcapRunningModel &&
	    strOpencvConfig.mapFilterModel &&
	    strOpencvConfig.localizationEnable &&
	    !strOpencvConfig.localizationDebugEnable)
	{
		if (!g_pcapLocFeed.start(strOpencvConfig.rs_pcapPath, 1.0))
		{
			LOG_RAW("**** pcap 9110 feed start failed: %s\n",
			        strOpencvConfig.rs_pcapPath.c_str());
		}
	}

	// 【修复2】驱动启动后再创建信号线程
    std::atomic<bool> TerminateFlag{false};
    pthread_t sig_tid;
    if (pthread_create(&sig_tid, nullptr, SignalThread, &TerminateFlag) != 0) {
        perror("pthread_create SignalThread failed");
        stDriver.Stop();
        stDriver.Free();
        return -1;
    }

	// Viewer cvLaneView(strOpencvConfig);

	/*
	if(strOpencvConfig.isUdpRecEnable == 1){
		std::cout<<("IsUdpRecEnable = 1 \n")<<std::endl;
		//开一个线程接受离线pcap数据
		pthread_t recv_tid;
		
		if( pthread_create(&recv_tid, NULL, recv_thread, NULL) !=0 ){
			perror("udp receive pthread_create error");
			// exit(EXIT_FAILURE);
		}
		else{
			printf("create thread successful \n");
		}
	}
	else{
		printf("IsUdpRecEnable = 0");
	}
	*/

	// CvLaneDataCallback g_cvLaneCallback;

	// std::thread(recv_thread).detach();	// std::thread 缺少原生 pthread 的某些高级功能（如优先级设置）

#if 0
	//在main函数中加入可视化，暂时启用
	while(1){

		// auto data = CvLaneDataManager::GetInstance().WaitAndGetData();
		// cvLaneView.ProjectCvLane(data);
		
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}
#endif

    // ================================================================
    // 可视化主循环（PCL 必须在主线程运行）
    // ================================================================
    auto& visBuf = stDriver.GetVisBuffer();

    // 仅在 pcap / pcd 模式下启动可视化
    if (strOpencvConfig.pcapRunningModel || strOpencvConfig.pcdRunningModel)
    {
        InitGroundViewer();
        DrawCarBodyOutlines(strOpencvConfig, strAllLidarTransformInfo);

        PointCloud2Intensity::Ptr visGround(new PointCloud2Intensity);
        PointCloud2Intensity::Ptr visObstacle(new PointCloud2Intensity);
        std::vector<TrackedObstacle> visTracks;

        while (!TerminateFlag.load() && SpinGroundViewerOnce())
        {
            if (visBuf.Consume(visGround, visObstacle, visTracks))
            {
                UpdateGroundViewer(visGround, visObstacle, visTracks);
            }
        }
    }
    else
    {
        // online 模式：无可视化，保持主线程存活
        while (!TerminateFlag.load())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }

	// 4. 等待信号线程退出
	pthread_join(sig_tid, nullptr);

	// lsDrive.Stop();
    // lsDrive.Free();
	stDriver.Stop();
	stDriver.Free();
	g_pcapLocFeed.stop();   // 回收 pcap 9110 喂入线程
	Comm_Exit();
	
    return 0;
}
