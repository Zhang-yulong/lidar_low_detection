#include "ReadYamlFile.h"
#include <exception>
#include <stdexcept>



namespace Lidar_Low_Detection
{

YamlReader::YamlReader(const std::string& configFilePath) 
    : m_sConfigFilePath(configFilePath) 
{

}

/*
try:    尝试运行可能出错的代码块
catch:  捕获并处理异常
throw:  抛出异常
std::runtime_error: 标准运行时异常类，带错误信息。它是 std::exception 的子类，因此可以用 .what() 来获取异常信息。
*/

bool YamlReader::LoadConfig(SELF_DEBUG_CONFIG& config) 
{   try{
        cv::FileStorage fs(m_sConfigFilePath, cv::FileStorage::READ); // 打开文件以读取

        if (!fs.isOpened()) {
            std::cerr << "Error: Unable to open config file: " << m_sConfigFilePath << std::endl;
            return false;
        }

        cv::FileNode LidarNode = fs["Lidar"];
        if(LidarNode.empty()){
            throw std::runtime_error("Error: Lack <Lidar>");
        }

        cv::FileNode RoadNode = fs["Road"];
        if(RoadNode.empty()){
            throw std::runtime_error("Error: Lack <Road>");
        }

        cv::FileNode GroundSegmentationFromSACNode = fs["GroundSegmentationFromSAC"];
        if(GroundSegmentationFromSACNode.empty()){
            throw std::runtime_error("Error: Lack <GroundSegmentationFromSAC>");
        }

        cv::FileNode GroundSegmentationFromAPMFNode = fs["GroundSegmentationFromAPMF"];
        if(GroundSegmentationFromAPMFNode.empty()){
            throw std::runtime_error("Error: Lack <GroundSegmentationFromAPMF>");
        }

        cv::FileNode ClusteringNode = fs["Clustering"];
        if(ClusteringNode.empty()){
            throw std::runtime_error("Error: Lack <Clustering>");
        }

        cv::FileNode HoughLinesPNode = fs["HoughLinesP"];
        if(HoughLinesPNode.empty()){
            throw std::runtime_error("Error: Lack <HoughLinesP>");
        }

        cv::FileNode CurveFittingNode = fs["CurveFitting"];
        if(CurveFittingNode.empty()){
            throw std::runtime_error("Error: Lack <CurveFitting>");
        }

        cv::FileNode RANSACNode = fs["RANSAC"];
        if(RANSACNode.empty()){
            throw std::runtime_error("Error: Lack <RANSAC>");
        }

        cv::FileNode SlideWindowNode = fs["SlideWindow"];
        if(SlideWindowNode.empty()){
            throw std::runtime_error("Error: Lack <SlideWindow>");
        }

        cv::FileNode ModelNode = fs["Model"];
        if(ModelNode.empty()){
            throw std::runtime_error("Error: Lack <Model>");
        }

        cv::FileNode ViewerNode = fs["Viewer"];
        if(ViewerNode.empty()){
            throw std::runtime_error("Error: Lack <Viewer>");
        }

        cv::FileNode GroundLaneNode = fs["GroundLane"];
        if(GroundLaneNode.empty()){
            throw std::runtime_error("Error: Lack <GroundLane>");
        }

        cv::FileNode MapGroundFilterNode = fs["MapGroundFilter"];
        if(MapGroundFilterNode.empty()){
            throw std::runtime_error("Error: Lack <MapGroundFilter>");
        }

        // HdmapFilter 节为可选：缺失时按 mapFilterModel=0（不启用 HDMap 过滤）处理

        config.selfComputerIP   = ReadRequired<std::string>(LidarNode, "selfComputerIP");
        config.groupIP          = ReadRequired<std::string>(LidarNode, "groupIP");
        
        config.msopPort         = ValidatePort(ReadRequired<int>(LidarNode, "msopPort"));
        config.difopPort        = ValidatePort(ReadRequired<int>(LidarNode, "difopPort"));
  
        config.LSlidarType            = (std::string)fs["Lidar"]["LSlidarType"];

        std::string STlidarTypeStr   = (std::string)fs["Lidar"]["STlidarType"];
        auto it = kLidarTypeMap.find(STlidarTypeStr);
        if (it != kLidarTypeMap.end()) {
            config.STlidarType = it->second; // 找到了，赋值枚举值
        } else {
            throw std::runtime_error("Invalid LidarType in config: " + STlidarTypeStr);
        }

        config.verticalUpperAngle   = (int)fs["Lidar"]["verticalUpperAngle"];
        config.verticalLowerAngle   = (int)fs["Lidar"]["verticalLowerAngle"];
        config.scanRings            = (int)fs["Lidar"]["scanRings"];
        config.pathTolidarTransformConfig = (int)fs["Lidar"]["pathTolidarTransformConfig"];

        // 读取道路
        config.leftXMin             = (float)fs["Road"]["leftXMin"];
        config.leftXMax             = (float)fs["Road"]["leftXMax"];
        config.leftYMax             = (float)fs["Road"]["leftYMax"];
        config.rightXMin            = (float)fs["Road"]["rightXMin"];
        config.rightXMax            = (float)fs["Road"]["rightXMax"];
        config.rightYMax            = (float)fs["Road"]["rightYMax"];
        config.heightLower          = (float)fs["Road"]["heightLower"];
        config.heightUpper          = (float)fs["Road"]["heightUpper"];
        config.isDetectionLeftRoad  = (int)fs["Road"]["isDetectionLeftRoad"];
        config.isDetectionRightRoad = (int)fs["Road"]["isDetectionRightRoad"];

        // 读取地面分割SAC算法
        config.groundSegmentationThreshold  = (float)fs["GroundSegmentationFromSAC"]["groundSegmentationThreshold"];
        
        // 读取形态学分割算法
        config.maxWindowSize            = (int)fs["GroundSegmentationFromAPMF"]["maxWindowSize"];
        config.slope                    = (float)fs["GroundSegmentationFromAPMF"]["slope"];
        config.initialDistance          = (float)fs["GroundSegmentationFromAPMF"]["initialDistance"];
        config.maxDistance              = (float)fs["GroundSegmentationFromAPMF"]["maxDistance"];
        config.cellSize                 = (int)fs["GroundSegmentationFromAPMF"]["cellSize"];
        config.base                     = (int)fs["GroundSegmentationFromAPMF"]["base"];

        // 读取聚类参数(欧式聚类)
        config.leftMinClusterThreshold  = (float)fs["Clustering"]["leftMinClusterThreshold"];
        config.leftMaxClusterThreshold  = (float)fs["Clustering"]["leftMaxClusterThreshold"];
        config.leftPointsDistance       = (float)fs["Clustering"]["leftPointsDistance"];
        config.rightMinClusterThreshold = (float)fs["Clustering"]["rightMinClusterThreshold"];
        config.rightMaxClusterThreshold = (float)fs["Clustering"]["rightMaxClusterThreshold"];
        config.rightPointsDistance      = (float)fs["Clustering"]["rightPointsDistance"];

        // 读取霍夫变换
        config.hough_width          = (int)fs["HoughLinesP"]["width"];
        config.hough_height         = (int)fs["HoughLinesP"]["height"];
        config.hough_scale          = (float)fs["HoughLinesP"]["scale"];
        config.hough_offsetX        = (float)fs["HoughLinesP"]["x"];
        config.hough_left_offsetY   = (float)fs["HoughLinesP"]["left_y"];
        config.hough_right_offsetY  = (float)fs["HoughLinesP"]["right_y"];
        config.hough_rho            = (float)fs["HoughLinesP"]["rho"];
        config.hough_theta          = (float)fs["HoughLinesP"]["theta"];
        config.hough_threshold      = (float)fs["HoughLinesP"]["threshold"];
        config.hough_minLineLength  = (float)fs["HoughLinesP"]["minLineLength"];
        config.hough_maxLineGap     = (float)fs["HoughLinesP"]["maxLineGap"];

        // 读取曲线拟合
        config.distinguishRoadSideThreshold = (float)fs["CurveFitting"]["distinguishRoadSideThreshold"];

        // 读取RANSAC
        config.interations  = (int)fs["RANSAC"]["interations"];
        config.simga        = (float)fs["RANSAC"]["sigma"];
        config.kMin         = (float)fs["RANSAC"]["kMin"];
        config.kMax         = (float)fs["RANSAC"]["kMax"];

        // 读取滑窗滤波
        config.isRunningFilter  = (int)fs["SlideWindow"]["isRunningFilter"];  
        config.slideWindowCount = (int)fs["SlideWindow"]["slideWindowCount"];

        // 读取模式
        config.onlineModel          = (int)fs["Model"]["onlineModel"];
        config.pcapRunningModel     = (int)fs["Model"]["pcapRunningModel"];
        config.rs_pcapPath             = (std::string)fs["Model"]["rs_pcapPath"];
        config.pcdRunningModel      = (int)fs["Model"]["pcdRunningModel"];
        
        config.savePcd              = (int)fs["Model"]["savePcd"];    // 读取pcd保存
        config.pcdPath              = (std::string)fs["Model"]["pcdPath"];
        config.pcdPathTime          = (std::string)fs["Model"]["pcdPathTime"];
        config.savePcdReflection    = (int)fs["Model"]["savePcdReflection"];

        config.curbDetection        = (int)fs["Model"]["curbDetection"];
        config.groundLoadDetection  = (int)fs["Model"]["groundLoadDetection"];

        config.openGroundViewer      = (int)fs["Model"]["openGroundViewer"];
        config.openClusterViewer    = (int)fs["Model"]["openClusterViewer"];

        // 读取Viewer
        config.projectionModel  = (int)fs["Viewer"]["projectionModel"];
        config.savePictureModel = (int)fs["Viewer"]["savePictureModel"];
        config.viewer_width     = (int)fs["Viewer"]["width"];
        config.viewer_height    = (int)fs["Viewer"]["height"];
        config.viewer_scale     = (float)fs["Viewer"]["scale"];
        config.viewer_offsetX   = (float)fs["Viewer"]["x"];
        config.viewer_offsetY   = (float)fs["Viewer"]["y"];

        //GroundLane
        config.GL_heightLower       = (float)fs["GroundLane"]["GL_heightLower"];
        config.GL_heightUpper       = (float)fs["GroundLane"]["GL_heightUpper"];
        config.GL_intensityLower    = (int)fs["GroundLane"]["GL_intensityLower"];
        config.GL_intensityUpper    = (int)fs["GroundLane"]["GL_intensityUpper"];

        //UDP
        config.isUdpRecEnable   = (float)fs["UDP"]["isUdpRecEnable"];


        ///======== 低矮检测参数 ========
        config.roi_x_min            = (float)fs["MapGroundFilter"]["roi_x_min"];
        config.roi_x_max            = (float)fs["MapGroundFilter"]["roi_x_max"];
        config.roi_y_min            = (float)fs["MapGroundFilter"]["roi_y_min"];
        config.roi_y_max            = (float)fs["MapGroundFilter"]["roi_y_max"];
        config.grid_resolution      = (float)fs["MapGroundFilter"]["grid_resolution"];
        config.slope_threshold      = (float)fs["MapGroundFilter"]["slope_threshold"];
        config.height_diff_threshold = (float)fs["MapGroundFilter"]["height_diff_threshold"];
        config.min_points_per_cell = (int)fs["MapGroundFilter"]["min_points_per_cell"];
        // config.sensor_height = (float)fs["MapGroundFilter"]["sensor_height"];

        //车子参数
        config.car_half_x = (float)fs["MapGroundFilter"]["car_half_x"];
        config.car_half_y = (float)fs["MapGroundFilter"]["car_half_y"];
        // Body Filter
        config.body_filter_x_threshold = (float)fs["MapGroundFilter"]["body_filter_x_threshold"];
        config.body_filter_y_threshold = (float)fs["MapGroundFilter"]["body_filter_y_threshold"];

        config.obstacle_top_height_min_near = (float)fs["MapGroundFilter"]["obstacle_top_height_min_near"];
        config.obstacle_top_height_max_near = (float)fs["MapGroundFilter"]["obstacle_top_height_max_near"];
        config.obstacle_top_height_min_far  = (float)fs["MapGroundFilter"]["obstacle_top_height_min_far"];
        config.obstacle_top_height_max_far  = (float)fs["MapGroundFilter"]["obstacle_top_height_max_far"];
        config.near_range_boundary          = (float)fs["MapGroundFilter"]["near_range_boundary"];

        // ======== DebugViewer 调试可视化配置 ========
        cv::FileNode DebugViewerNode = fs["DebugViewer"];
        if (!DebugViewerNode.empty())
        {
            // 辅助 lambda: 从 FileNode 读取 DEBUG_VIEWER_CONFIG
            auto readDebugCfg = [](const cv::FileNode& parent, const std::string& key) -> DEBUG_VIEWER_CONFIG {
                DEBUG_VIEWER_CONFIG cfg;
                cv::FileNode child = parent[key];
                if (!child.empty())
                {
                    int raw_enable = (int)child["enable"];
                    std::cout << "[ReadYaml] " << key << ".enable raw=" << raw_enable << std::endl;
                    cfg.enable = (int)child["enable"];
                    cfg.show   = (int)child["show"];
                    cfg.save   = (int)child["save"];
                }
                return cfg;
            };

            config.debug_save_dir     = (std::string)DebugViewerNode["save_dir"];

            config.HeightMap          = readDebugCfg(DebugViewerNode, "HeightMap");
            config.GroundMask         = readDebugCfg(DebugViewerNode, "GroundMask");
            config.Slope              = readDebugCfg(DebugViewerNode, "Slope");
            config.GroundReference    = readDebugCfg(DebugViewerNode, "GroundReference");
            config.ObstacleCandidate  = readDebugCfg(DebugViewerNode, "ObstacleCandidate");
            config.Cluster            = readDebugCfg(DebugViewerNode, "Cluster");
            config.BoundingBox        = readDebugCfg(DebugViewerNode, "BoundingBox");
            config.Overlay            = readDebugCfg(DebugViewerNode, "Overlay");
            config.TrackerOverlay     = readDebugCfg(DebugViewerNode, "TrackerOverlay");
            config.HistoricalFeedback = readDebugCfg(DebugViewerNode, "HistoricalFeedback");
            config.RawImageObb         = readDebugCfg(DebugViewerNode, "RawImageObb");
        }
        else{
            throw std::runtime_error("Error: Lack <DebugViewer>");
        }


        // ======== hdmap ========
        cv::FileNode HdNode = fs["HdmapFilter"];
        if (!HdNode.empty())
        {
            config.mapFilterModel       = (int)HdNode["mapFilterModel"];
            config.mapPath              = (std::string)HdNode["mapPath"];
            config.hdmapFilterMode      = HdNode["filterMode"].empty()     ? 0     : (int)HdNode["filterMode"];
            config.hdmapExpandDistance  = HdNode["expandDistance"].empty() ? 0.5f  : (float)HdNode["expandDistance"];
            config.hdmapLogEveryN       = HdNode["logEveryN"].empty()      ? 50    : (int)HdNode["logEveryN"];
        }
        else
        {
            config.mapFilterModel       = 0;
            config.mapPath              = "";
            config.hdmapFilterMode      = 0;
            config.hdmapExpandDistance  = 0.5f;
            config.hdmapLogEveryN       = 50;
        }

        // mapPath 为空 -> 使用默认地图地址（/etc/echiev/hdmap/hdmap.bin）
        if (config.mapPath.empty())
        {
            config.mapPath = "/etc/echiev/hdmap/hdmap.bin";  ////etc/echiev/from_11_hdmap/hdmap.bin
        }

        // ======== Localization（新增，可选节）========
        cv::FileNode LocNode = fs["Localization"];
        if (!LocNode.empty())
        {
            config.localizationEnable     = (int)LocNode["enable"];
            config.localizationTimeoutMs  = LocNode["timeout_ms"].empty() ? 1000 : (int)LocNode["timeout_ms"];
            config.localizationDebugEnable = LocNode["debugEnable"].empty() ? 0 : (int)LocNode["debugEnable"];
            config.localizationDebugX      = LocNode["debugX"].empty() ? 0.0 : (double)LocNode["debugX"];
            config.localizationDebugY      = LocNode["debugY"].empty() ? 0.0 : (double)LocNode["debugY"];
            config.localizationDebugHeading = LocNode["debugHeading"].empty() ? 0.0 : (double)LocNode["debugHeading"];
        }
        else
        {
            config.localizationEnable     = 0;
            config.localizationTimeoutMs  = 1000;
            config.localizationDebugEnable = 0;
            config.localizationDebugX      = 0.0;
            config.localizationDebugY      = 0.0;
            config.localizationDebugHeading = 0.0;
        }

        fs.release(); // 释放文件
        return true;
    }
    catch(const std::exception &e){
        std::cerr<<"[Error config param]"<<e.what()<<std::endl;
        return false;
    }
}





/**-----------------------------------------
    业务逻辑校验函数
 -----------------------------------------*/
int ValidatePort(int port) {
    if (port < 0 || port > 65535) {
        throw std::runtime_error("端口号越界: " + std::to_string(port));
    }
    return static_cast<int>(port);
}

std::string ValidateLidarType(const std::string& type) {
    const static std::set<std::string> validTypes = {"CH64w", "CH128", "VLP16"};
    if (!validTypes.count(type)) {
        throw std::runtime_error("非法雷达型号: " + type);
    }
    return type;
}

double ValidateAngle(double angle) {
    if (angle < -360.0 || angle > 360.0) {
        throw std::runtime_error("角度值非法: " + std::to_string(angle));
    }
    return angle;
}

int ValidateScanRings(int rings) {
    if (rings <= 0 || rings > 256) {
        throw std::runtime_error("扫描线数越界: " + std::to_string(rings));
    }
    return rings;
}

void ValidateAngleOrder(double upper, double lower) {
    if (upper <= lower) {
        throw std::runtime_error("上下角度范围错误: upper=" 
            + std::to_string(upper) + " lower=" + std::to_string(lower));
    }
}



}