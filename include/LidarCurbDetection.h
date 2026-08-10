#pragma once
#ifndef LIDAR_CURB_DETECTION_H
#define LIDAR_CURB_DETECTION_H

#include <pcl/common/common.h>
#include <pcl/PointIndices.h>
#include <pcl/ModelCoefficients.h>  // 模型系数的定义
#include <pcl/sample_consensus/method_types.h> // 包含用于采样一致性算法的不同方法的定义，如RANSAC、MSAC等
#include <pcl/sample_consensus/model_types.h> // 包含用于采样一致性算法的不同模型的定义，如平面、球体、圆柱体
#include <pcl/segmentation/sac_segmentation.h>  // 包含用于分割点云的采样一致性算法（SACSegmentation）的定义，用于识别点云的几何模型
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>    // 包含用于从点云中提取特定索引的函数和类，用于根据索引提取点云中的子集
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/approximate_progressive_morphological_filter.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <Eigen/Dense>

#include <algorithm>
#include <chrono>
#include <time.h>
#include <condition_variable>
#include <queue>
#include <thread>
#include <chrono>
#include <utility>
#include <vector>
#include <unordered_set>

#include "Type.h"
#include "CurveFitting.h"
#include "Viewer.h"

#include "struct_typedef.h"
#include "communication.h"

namespace Lidar_Low_Detection
{

class LidarCurbDetection;

struct LineInfo {
    cv::Vec4f lineParams; // 直线参数 (vx, vy, x0, y0)
    double length;        // 直线长度
    double avg_x;
};

struct PolarLine{
    double rho;     //距原点的距离
    double theta;   //法向量与x轴的夹角
};

struct LineRunningInfo{
    long runningValue;   //程序运行次数
    bool existFlag;
    PointCloud2Intensity::Ptr linePointCloud;
};

// 计算直线的斜率和截距
struct HoughSPLineInfo {
    double slope;  // 斜率
    double intercept;  // 截距
    cv::Vec4i line;  // 原始直线段
};

struct Line {
    int id;
    cv::Point2f start;
    cv::Point2f end;
    // float slope;
    float angle;
    float length;
    bool isParallelLine;
    int range;

    float A;
    float B;
    float C;
    
    //默认构造
    Line(): id(0), start(cv::Point2f(0, 0)), end(cv::Point2f(0, 0)), 
            angle(0), length(0), isParallelLine(false), range(0), 
            A(0), B(0), C(0) {}

    // 有参构造函数
    Line(cv::Vec4i& line) {
      
        int x1 = line[0], y1 = line[1];
        int x2 = line[2], y2 = line[3];

        // 按 y从小到大排序，
        if (y1 > y2) {
            std::swap(line[0], line[2]);
            std::swap(line[1], line[3]);
        }
        
        start = cv::Point2f(line[0], line[1]);
        end = cv::Point2f(line[2], line[3]);


        // std::cout<<"起点 ( " <<start.x<<" , "<<start.y<<" )"<<std::endl;
        // std::cout<<"终点 ( " <<end.x<<" , "<<end.y<<" )"<<std::endl;

        // slope = (end.x - start.x) != 0 ? (end.y - start.y) / (end.x - start.x) : std::numeric_limits<float>::infinity();
        
        // 使用 atan2 计算角度，避免斜率除以零的问题； 上面修改了起始点和终点，导致0°->180°是x轴负方向->正方向
        float angle_1 = std::abs(std::atan2(end.y - start.y, end.x - start.x))* 180 / M_PI;
        
        if((std::abs(angle_1 - 90) < 2)){
            float midX = std::floor((start.x + end.x) / 2);
            std::cout<<"原始start:【"<<start.x<<","<<start.y<<"】, end:【"<<end.x<<","<<end.y<<"】, 原始angle_1: "<<angle_1<<"; 现在start:【"<<midX<<","<<start.y<<"】"<<std::endl;
            start.x = midX;
            end.x = midX;
        }
        else if( (std::abs(start.x - end.x) <=4) && (std::abs(start.y - end.y) < 125) ){
            float midX = std::floor((start.x + end.x) / 2);
            std::cout<<"原始start:【"<<start.x<<","<<start.y<<"】, end:【"<<end.x<<","<<end.y<<"】, 原始angle_1: "<<angle_1<<"; 现在start:【"<<midX<<","<<start.y<<"】"<<std::endl;
            start.x = midX;
            end.x = midX;
        }
        else{
            std::cout<<"原始start:【"<<start.x<<","<<start.y<<"】, end:【"<<end.x<<","<<end.y<<"】, 原始angle_1: "<<angle_1<<std::endl;
        }

        // if( (std::abs(angle_1 - 90) < 2)  && (std::abs(start.x - end.x) <=4) ){ //横向距离小于4个像素
            
        //     float midX = std::floor((start.x + end.x) / 2);
        //     std::cout<<"原始start:【"<<start.x<<","<<start.y<<"】, end:【"<<end.x<<","<<end.y<<"】, 原始angle_1: "<<angle_1<<"; 现在start:【"<<midX<<","<<start.y<<"】"<<std::endl;
        //     start.x = midX;
        //     end.x = midX;
        // }
        // else{
        //     std::cout<<"原始start:【"<<start.x<<","<<start.y<<"】, end:【"<<end.x<<","<<end.y<<"】, 原始angle_1: "<<angle_1<<std::endl;
        // }
        
        angle = std::abs(std::atan2(end.y - start.y, end.x - start.x))* 180 / M_PI;
        length = std::sqrt((end.x - start.x) * (end.x - start.x) + (end.y - start.y) * (end.y - start.y));
        
        range = angle / 10; //0°-180°分为18份区域
        if(range >= 18){
            range = 17;
        }

        A = start.y - end.y;
        B = end.x - start.x;
        C = start.x * end.y - end.x * start.y;
        // std::cout<<" angle: "<<angle<<" , 直线方程："<< A <<" X + "<< B <<" Y + "<< C <<" = 0"<<std::endl;
       
        
        if(angle < 20 || angle > 160 ){
            isParallelLine = true;  
        }
        else{
            isParallelLine = false;
        }
            
    }
};

struct ClusterInfo{

    int id;
    float length;
    // Line masterLine;
    std::vector<Line> masterLines;
};



class LidarCurbDetection{

public:

    LidarCurbDetection(const SELF_DEBUG_CONFIG & config);
    
    ~LidarCurbDetection();

    const int &GetValueOfScanRings();

    std::pair<PointCloud2Intensity::Ptr, PointCloud2Intensity::Ptr> GroundSegmentationStart(PointCloud2Intensity::Ptr pInCloud, const unsigned long long &ullTime);
    
    void CloudSegmentation(PointCloud2Intensity::Ptr pInCloud, unsigned long long ullTime); 

    std::pair<PointCloud2RGB::Ptr, PointCloud2RGB::Ptr> EdgeClusteringProcess(PointCloud2Intensity::Ptr pNoGroundPoints, const unsigned long long &ullTime);
                                 
private:   

    /*计算是哪条id，ls_drive镭神雷达解析的时候应该已经计算过了*/
    void CalculateScanID(PointCloud2Intensity::Ptr pInCloud, PointCloud2Intensity::Ptr &pOutCloud);

    void GroundFilterFromSAC(PointCloud2Intensity::Ptr &pGroundPoints, PointCloud2Intensity::Ptr &pNoGroundPoints);

    void GroundFilterFromAPMF(PointCloud2Intensity::Ptr &pGroundPoints, PointCloud2Intensity::Ptr &pNoGroundPoints);

    void ExtractGroundFromSAC( PointCloud2Intensity::Ptr &pIncloud,
                        PointCloud2Intensity::Ptr &pOutcloud,
                        pcl::PointIndices::Ptr pIndices,
                        bool bSetNeg);

    void ExtractGroundFromPMF( PointCloud2Intensity::Ptr &pIncloud,
                        PointCloud2Intensity::Ptr &pOutcloud,
                        pcl::PointIndicesPtr pIndices,
                        bool bSetNeg);

    void ExtractPointsByHeightDifference(PointCloud2Intensity::Ptr &pInCloud);
    
    bool IsClustering(  const float PointsDistance, 
                        const float MaximumClusterThreshold, 
                        const float MimimumClusterThreshold, 
                        PointCloud2Intensity::Ptr pInputCloud,
                        std::vector<pcl::PointIndices> &vClusterIndices);

    bool IsClustering(  const float PointsDistance, 
                        const float MaximumClusterThreshold, 
                        const float MimimumClusterThreshold, 
                        PointCloud2Intensity::Ptr pInputCloud, 
                        std::vector<PointCloud2Intensity::Ptr> &vClusterPtr);

    double euclideanDistance(const cv::Point& p1, const cv::Point& p2);
    double calculateAngle(const cv::Vec4f& line1, const cv::Vec4f& line2);
    double computeXDistance(const cv::Point2f& center1, const cv::Point2f& center2);
    bool isDirectionSimilar(const cv::Vec4f& line1, const cv::Vec4f& line2, double angleThreshold);
    bool isBoundingBoxOverlapping(const cv::Rect& bbox1, const cv::Rect& bbox2);
    double computeYDistance(const cv::Point2f& center1, const cv::Point2f& center2);
    double pointToLineDistance(const cv::Point& pt, const cv::Vec4f& line);
    bool isLineAngleValid(const cv::Vec4f& line, double maxAngle);
    void normalizeLineABC(double &A, double &B, double &C);    

    std::vector<std::vector<Line>> clusterLinesBySlope(const std::vector<Line>& lines, float slope_threshold);
    bool checkLineSame(const Line &l1, const Line &l2);
    std::vector<std::vector<Line>> clusterLinesBySlope(std::vector<Line>& lines, float slope_threshold, const int &iSideFlag);

    std::vector<Line> findLongestCluster(const std::vector<std::vector<Line>>& clusters);
    std::vector<Line> findSuitableCluster(const std::vector<std::vector<Line>>& All_clusters, const int &iSideFlag);
    double calculateSlope(const cv::Vec4i& line);
    double calculateIntercept(const cv::Vec4i& line, double slope);
    std::vector<std::vector<HoughSPLineInfo>> clusterLines(const std::vector<HoughSPLineInfo>& lines, double slope_threshold, double distance_threshold);
    cv::Vec4f fitLineToCluster(const std::vector<HoughSPLineInfo>& cluster);
    float calculateLength(const cv::Point& start, const cv::Point& end);
    

    cv::Point2f computeCentroid(const std::vector<cv::Point>& contour);
    void splitContoursByCentroid(const std::vector<cv::Point>& contour, 
                             std::vector<cv::Point>& finalContour, 
                             int iSideFlag, 
                             cv::Mat &image);
    void ransacFitLine(const std::vector<cv::Point>& points, cv::Vec4f& bestLine, std::vector<cv::Point>& inliers, 
                   int maxIterations , double threshold , double maxAngle , double inlierRatio );
    
    void findOnlyFitCluster(std::vector<std::vector<cv::Point>> &InitContours, cv::Mat &image);
    void findOnlyFitCluster(std::vector<std::vector<cv::Point>> &InitContours, cv::Mat &image, int iSideFlag);
    

    bool IsClustering_hough(  
                        PointCloud2Intensity::Ptr pInputCloud, 
                        std::vector<PointCloud2Intensity::Ptr> &vClusterPtr,
                        const unsigned long long &ullTime,
                        const int &iSideFlag);                    
    
    

    PointCloud2Intensity::Ptr SlideWindowProcess(std::queue<LineRunningInfo> &qFinalClusterInfo);

    // void CurveFitting(PointCloud2Intensity::Ptr pInCloud, std::vector<pcl::PointIndices> &vClusterIndices);  //尝试写的另一个文件里
    
    PointCloud2Intensity::Ptr getCloserRightCluster(const std::vector<PointCloud2Intensity::Ptr>& clusters);
    PointCloud2Intensity::Ptr getCloserLeftCluster(const std::vector<PointCloud2Intensity::Ptr>& clusters);


private:  

    float m_fGroundSegmentationThreshold;
    float m_fLowerBound;    //雷达的最低垂直视场角 
    float m_fUpperBound;
    int m_iScanRings;       //可以把雷达分成多少线
    float m_fFactor;
    
    Eigen::Matrix<float,3,3> R_ToLeftProject;
    Eigen::Matrix<float,3,3> R_ToRightProject;
    Eigen::Matrix<float,3,3> R_Inv_ToLeftProject;
    Eigen::Matrix<float,3,3> R_Inv_ToRightProject;

    std::vector<PointCloud2Intensity::Ptr> m_vCloudPtrList;

    std::shared_ptr<pcl::visualization::PCLVisualizer> ground_viewer;
    std::shared_ptr<pcl::visualization::PCLVisualizer> no_ground_viewer;
    std::shared_ptr<pcl::visualization::PCLVisualizer> cluster_viewer;

    PointCloud2RGB::Ptr m_pAll_deal_cluster_cloud;
    PointCloud2RGB::Ptr m_pAll_output_curve_cloud;

    CurveFitting *m_pCurveFitting;
    const SELF_DEBUG_CONFIG &m_strLCDConfig;
    
    long m_iSystemRunCount;   //记录程序运行次数
    std::queue<LineRunningInfo> m_qLeftLineInfo;
    std::queue<LineRunningInfo> m_qRightLineInfo;

    int iLastClustersRange_right, iLastClustersRange_left;
    std::vector<Line> vLastClusters_right, vLastClusters_left;
    int m_iPastRange;

};
}
#endif
