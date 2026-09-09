#pragma once
#ifndef READ_YAML_FILE_H
#define READ_YAML_FILE_H


#include <rs_driver/driver/driver_param.hpp>

#include <opencv2/core/persistence.hpp>
#include <opencv2/opencv.hpp>

#include <stdexcept>
#include <string>
#include <iostream>
#include <set>
#include <map>

// using namespace cv; //禁止在头文件里写using namespace xxx。会产生很多库链接的问题。要写也是写在cpp文件中。

namespace Lidar_Low_Detection
{

/**
 * @brief 单个调试图的显示/保存配置
 *        enable: 是否启用该调试图
 *        show:   是否实时显示（cv::imshow）
 *        save:   是否保存到磁盘（cv::imwrite）
 */
struct DEBUG_VIEWER_CONFIG
{
    bool enable = false;
    bool show   = false;
    bool save   = false;
};


const std::map<std::string, robosense::lidar::LidarType> kLidarTypeMap = {
    {"RS_MECH", robosense::lidar::LidarType::RS_MECH},
    {"RS16", robosense::lidar::LidarType::RS16},
    {"RS32", robosense::lidar::LidarType::RS32},
    {"RSBP", robosense::lidar::LidarType::RSBP},
    {"RSAIRY", robosense::lidar::LidarType::RSAIRY},
    {"RSHELIOS", robosense::lidar::LidarType::RSHELIOS},
    {"RSHELIOS_16P", robosense::lidar::LidarType::RSHELIOS_16P},
    {"RS128", robosense::lidar::LidarType::RS128},
    {"RS80", robosense::lidar::LidarType::RS80},
    {"RS48", robosense::lidar::LidarType::RS48},
    {"RSP128", robosense::lidar::LidarType::RSP128},
    {"RSP80", robosense::lidar::LidarType::RSP80},
    {"RSP48", robosense::lidar::LidarType::RSP48},

    {"RS_MEMS", robosense::lidar::LidarType::RS_MEMS},
    {"RSM1", robosense::lidar::LidarType::RSM1},
    {"RSM2", robosense::lidar::LidarType::RSM2},
    {"RSM3", robosense::lidar::LidarType::RSM3},
    {"RSE1", robosense::lidar::LidarType::RSE1},
    {"RSMX", robosense::lidar::LidarType::RSMX},
    {"RSEM4", robosense::lidar::LidarType::RSEM4},
    
    {"RSEMX", robosense::lidar::LidarType::RSEMX},

    {"RS_JUMBO", robosense::lidar::LidarType::RS_JUMBO},
    {"RSM1_JUMBO", robosense::lidar::LidarType::RSM1_JUMBO},

    
    // RS_MECH = 0x01,
    // RS16 = RS_MECH,
   
    // mems
    //   RS_MEMS = 0x20,
    //   RSM1 = RS_MEMS,

    // jumbo
    //  RS_JUMBO= 0x100,
    //  RSM1_JUMBO = RS_JUMBO + RSM1,
    // ... 把所有需要的类型都列在这里
};



struct SELF_DEBUG_CONFIG{
    
    // Lidar
    std::string selfComputerIP;
    std::string groupIP;
    int msopPort;
    int difopPort;
    robosense::lidar::LidarType STlidarType;
    std::string LSlidarType;
    int verticalUpperAngle;
    int verticalLowerAngle;
    int scanRings;
    int pathTolidarTransformConfig;

    // Road
    float leftXMin;
    float leftXMax;
    float leftYMax;
    float rightXMin;
    float rightXMax;
    float rightYMax;
    float heightUpper;
    float heightLower;
    int isDetectionLeftRoad;
    int isDetectionRightRoad;

    // GroundSegmentationFromSAC:
    float groundSegmentationThreshold;
    
    // GroundSegmentationFromAPMF:
    int maxWindowSize;
    float slope;
    float initialDistance;
    float maxDistance;
    int cellSize;
    int base;

    // Clustering:
    float leftMinClusterThreshold;
    float leftMaxClusterThreshold;
    float leftPointsDistance;
    float rightMinClusterThreshold;
    float rightMaxClusterThreshold;
    float rightPointsDistance;

    //HoughLinesP:
    int hough_width;
    int hough_height;
    float hough_scale;          // 默认是50个像素表示1米
    float hough_offsetX;        // v
    float hough_left_offsetY;   // u
    float hough_right_offsetY;  // u
    float hough_rho;
    float hough_theta;
    float hough_threshold;
    float hough_minLineLength;
    float hough_maxLineGap;

    // CurveFitting:
    float distinguishRoadSideThreshold;

    // RANSAC:
    int interations;
    float simga;
    float kMin;
    float kMax;

    // SlidingWindow
    int isRunningFilter;
    int slideWindowCount;

    // Model
    int onlineModel;
    int pcapRunningModel;
    std::string rs_pcapPath;
    int pcdRunningModel;
    int savePcd;
    std::string pcdPath;
    std::string pcdPathTime;
    int savePcdReflection;
    int curbDetection;
    int groundLoadDetection;

    int openGroundViewer;
    int openClusterViewer;

    // Viewer
    int projectionModel;
    int savePictureModel;
    
    int viewer_width;
    int viewer_height;
    float viewer_scale;     //默认是50个像素表示1米
    float viewer_offsetX;
    float viewer_offsetY;

    //GroundLane
    float GL_heightLower;
    float GL_heightUpper;
    int GL_intensityLower;
    int GL_intensityUpper;

    //UDP
    int isUdpRecEnable;

    //======== 低矮检测参数 ========
    float roi_x_min;
    float roi_x_max;
    float roi_y_min;
    float roi_y_max;
    float grid_resolution;
    float slope_threshold;
    float height_diff_threshold;
    int min_points_per_cell;
    // float sensor_height;

    //车子参数
    float car_half_x;
    float car_half_y;

    // Body Filter（车身剔除参数，可通过 YAML 调整）
    float body_filter_x_threshold;  // x > 此值保留（车头前方）
    float body_filter_y_threshold;  // y < 此值保留（车身右侧外）

    float obstacle_top_height_min_near;
    float obstacle_top_height_max_near;
    float obstacle_top_height_min_far;
    float obstacle_top_height_max_far;
    float near_range_boundary;

    // ======== DebugViewer 调试可视化配置 ========
    std::string debug_save_dir;              // 调试图像保存目录

    DEBUG_VIEWER_CONFIG HeightMap;           // Grid 高程热力图
    DEBUG_VIEWER_CONFIG GroundMask;          // Ground Label 分类图
    DEBUG_VIEWER_CONFIG Slope;               // 坡度热力图
    DEBUG_VIEWER_CONFIG GroundReference;     // Ground Reference 热力图
    DEBUG_VIEWER_CONFIG ObstacleCandidate;   // 障碍物候选 + 竖直结构
    DEBUG_VIEWER_CONFIG Cluster;             // 聚类 Label 图
    DEBUG_VIEWER_CONFIG BoundingBox;         // 包围盒俯视图
    DEBUG_VIEWER_CONFIG Overlay;             // 最终点云叠加图
    DEBUG_VIEWER_CONFIG TrackerOverlay;      // 第九层: 跟踪结果点云叠加图
    DEBUG_VIEWER_CONFIG HistoricalFeedback;  // 第十层: Historical Feedback 验证图（Current ∪ Historical → Fused）

    // ======== hdmap过滤 ========
    int mapFilterModel;
    std::string mapPath;

    // ======== Localization + HDMap 扩展配置（新增，见迁移设计文档）========
    int localizationEnable;        // Localization.enable    : 0=不订阅定位 1=订阅
    int localizationTimeoutMs;     // Localization.timeout_ms: 定位新鲜度阈值(ms)
    int localizationDebugEnable;   // Localization.debugEnable: 0=off 1=离线调试固定位姿
    double localizationDebugX;     // Localization.debugX     : 调试位姿 X（地图坐标 m）
    double localizationDebugY;     // Localization.debugY     : 调试位姿 Y（地图坐标 m）
    double localizationDebugHeading; // Localization.debugHeading: 调试航向（度）
    int hdmapFilterMode;           // HdmapFilter.filterMode  : 0=仅标记(默认) 1=软约束
    float hdmapExpandDistance;     // HdmapFilter.expandDistance: 道路边界外扩距离(m)
    int hdmapLogEveryN;            // HdmapFilter.logEveryN   : 每 N 帧打印 cluster 级日志
};

class YamlReader {
public:
    // 构造函数，传入配置文件路径
    YamlReader(const std::string& configFilePath);
      
    // 从配置文件中加载参数
    bool LoadConfig(SELF_DEBUG_CONFIG& config);

    /*函数模版*/
    template<typename T>
    T ReadRequired(const cv::FileNode &node, const std::string &key);
private:

    template<typename T>
    bool CheckType(const cv::FileNode &node);

    std::string m_sConfigFilePath; // 配置文件路径
};



/*-----------------------------------------
    通用模板（不支持的类型默认返回 false）
    1.c++17中使用：std::is_same_v，它是是C++17中引入的一个模板变量，用于在编译时检查两个类型是否相同‌。
    2.现在修改成c++11，使用模版特化的方法
    3.static 是为了让我们可以直接用 TypeChecker<int>::Check(node) 这种方式调用，而不需要创建对象。
 -----------------------------------------*/
template<typename T>
struct TypeChecker{
    static bool Check(const cv::FileNode&){
        return false;
    }
};

//特化 int 类型
template <>
struct TypeChecker<int>{
    static bool Check(const cv::FileNode& node){
        return node.isInt();
    }
};

//特化 float 类型
template <>
struct TypeChecker<float>{
    static bool Check(const cv::FileNode& node){
        return node.isReal();
    }
};

//特化 String 类型
template <>
struct TypeChecker<std::string>{
    static bool Check(const cv::FileNode& node){
        return node.isString();
    }
};

/**-----------------------------------------
    类型检查模版
    ReadRequired 中调用它
 -----------------------------------------*/
template<typename T>
bool YamlReader::CheckType(const cv::FileNode& node) {
    return TypeChecker<T>::Check(node);
}

/**-----------------------------------------
    安全读取模版
 -----------------------------------------*/
template<typename  T> 
T YamlReader::ReadRequired(const cv::FileNode &node, const std::string &key){
    cv::FileNode target = node[key];
    
    if(target.empty()){
        throw std::runtime_error("缺少必要参数: " + key);
    }

    if(!CheckType<T>(target)){
        throw std::runtime_error(key + "类型不匹配");
    }

    return (T)target;
}


int ValidatePort(int port);
std::string ValidateLidarType(const std::string& type);
double ValidateAngle(double angle);
int ValidateScanRings(int rings);
void ValidateAngleOrder(double upper, double lower);

};



#endif