#pragma once
#ifndef VIEWER_H
#define VIEWER_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include "Type.h"
#include "ReadYamlFile.h"
#include "struct_typedef.h"

using namespace cv;

namespace Lidar_Low_Detection
{
class Viewer
{
public:
    // 构造函数，初始化图像大小、比例和偏移量
    Viewer(const SELF_DEBUG_CONFIG & config);
    ~Viewer();
    void ProjectPointCloud( const PointCloud2Intensity::Ptr &pAllInit, 
                            const PointCloud2Intensity::Ptr &pAllNoGround, 
                            const PointCloud2RGB::Ptr &pAllCluster, 
                            const PointCloud2RGB::Ptr &pAllCurve);
    
    void ProjectCvLane(const std::vector<STR_CV_LANE_DATA> &vLaneData);
    
    void Display();
    cv::Mat Display() const;
    
    void SaveImage(unsigned long long ullTime);
    void SaveImage(std::string folderPath, unsigned long long ullTime);

    cv::Mat DrawLCDtempImage(const cv::Mat &result, const cv::Point2d &offset); //聚类过程中图像投影
    
private:
    
    int m_iWidth;           // 图像宽度
    int m_iHeight;          // 图像高度          
    double m_dScale;        // 真实环境1米对应的像素数        
    cv::Point2d m_offset;   // 偏移量，定义原点在图像中的位置
    cv::Mat m_image;        // 用于显示点云的图像

    // 聚类过程中图像投影参数
    int m_iHough_width;
    int m_iHough_height;
    float m_iHough_scale;
    float m_iHough_offsetX;
    float m_iHough_left_offsetY;
    float m_iHough_right_offsetY;


    void DrawAxes();
    void DrawEquidistantLines();
    void DrawHorizontalEquidistantLines();
    void DrawViewerNotes();

};
}
#endif
