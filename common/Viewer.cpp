#include "Viewer.h"

#include <opencv2/core/core.hpp>

namespace Lidar_Low_Detection
{

std::string Image_folderPath= "/home/zyl/echiev_lidar_curb_detection/log/image/";

Viewer::Viewer(const SELF_DEBUG_CONFIG & config)
{
    // Viewer
    m_iWidth = config.viewer_width;
    m_iHeight = config.viewer_height;
    m_dScale = config.viewer_scale;
    m_offset.x = config.viewer_offsetX;
    m_offset.y = config.viewer_offsetY;
    m_image = Mat::zeros(m_iHeight, m_iWidth, CV_8UC3);  // 创建图像并初始化为黑色

    // HoughLinesP
    m_iHough_width = config.hough_width;
    m_iHough_height = config.hough_height;
    m_iHough_scale = config.hough_scale;
    m_iHough_offsetX = config.hough_offsetX;
    m_iHough_left_offsetY = config.hough_left_offsetY;
    m_iHough_right_offsetY = config.hough_right_offsetY;
}

Viewer::~Viewer(){

}

// #if 0
// //以下图像都是Y轴正方向向右，x轴正方向向下


// // 绘制图像坐标轴
// // Y轴向右，x轴向下
// void Viewer::DrawAxes() {
//     // 绘制y轴（水平向右）
//     line(m_image, Point(0, m_offset.x), Point(m_iWidth, m_offset.x), Scalar(255, 0, 0), 1);  // 蓝色线
//     putText(m_image, "Y", Point(m_iWidth - 50, m_offset.x - 10), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 0, 0), 1);

//     // 绘制x轴（垂直向下）
//     line(m_image, Point(m_offset.y, 0), Point(m_offset.y, m_iHeight), Scalar(0, 0, 255), 1); // 红色线
//     putText(m_image, "X", Point(m_offset.y + 10, m_iHeight - 10), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255), 1);
    
// }


// // 绘制与x轴平行的等距线（x=interval的倍数）
// void Viewer::DrawEquidistantLines() 
// {
//     // 等距线间隔，单位为米
//     double interval = 5.0;

//     // 计算等距线在图像中的位置并绘制
//     for (double x = -50; x <= 50; x += interval) {
//         int y = static_cast<int>(x * m_dScale + m_offset.y); // 计算等距线在图像中的位置
//         if(x == 0)
//             continue;
//         else{
//             if (y >= 0 && y < m_iWidth) {  // 确保线在图像边界内
//                 // line(image, Point(y, 0), Point(y, height), Scalar(200, 200, 200), 1); // 浅灰色
//                 putText(m_image, std::to_string(static_cast<int>(x)) + "m", Point(y + 5, m_offset.x - 10), 
//                         FONT_HERSHEY_SIMPLEX, 0.5, Scalar(200, 200, 200), 1);
//             }
//         }   
//     }

// }


// // 绘制与y轴平行的等距线（y=interval的倍数）
// void Viewer::DrawHorizontalEquidistantLines() 
// {
//     // 等距线间隔，单位为米
//     double interval = 1.0;

//     // 计算等距线在图像中的位置并绘制
//     for (double y = -50; y <= 50; y += interval) {
//         int x = static_cast<int>(y * m_dScale + m_offset.x); // 计算等距线在图像中的位置   //若(-y * scale + offset.x)那么x轴上的数字就反了
//         if (x >= 0 && x < m_iHeight) {  // 确保线在图像边界内
//             line(m_image, Point(0, x), Point(m_iWidth, x), Scalar(200, 200, 200), 1); // 浅灰色
//             putText(m_image, std::to_string(static_cast<int>(y)) + "m", Point(m_offset.y + 10, x - 5), 
//                     FONT_HERSHEY_SIMPLEX, 0.5, Scalar(200, 200, 200), 1);
//         }
//     }
// }


// void Viewer::ProjectPointCloud(const PointCloud2RGB::Ptr &pAllCluster, const PointCloud2RGB::Ptr &pAllCurve)
// {
    
//     // 清空图像
//     m_image.setTo(Scalar(0, 0, 0));   //黑色

//     DrawAxes();
//     DrawEquidistantLines();
//     DrawHorizontalEquidistantLines();

//     // 遍历所有聚类簇点云
//     for (const auto& pt : pAllCluster->points) {
        
//         // 投影时将3D点的x和y坐标互换，并应用比例缩放和偏移
//         int x = static_cast<int>(pt.x * m_dScale + m_offset.x);
//         int y = static_cast<int>(pt.y * m_dScale + m_offset.y);

//         // 检查是否在图像边界内
//         if (x >= 0 && x < m_iWidth && y >= 0 && y < m_iHeight)
//             circle(m_image, Point(y, x), 3, Scalar(0, 255, 0), -1); // 绘制绿色小圆点
//         else
//             printf("聚类簇点云不在图像内 (%d , %d)\n",x,y);
         
//     }

//     // 遍历所有拟合直线点云
//     for (const auto& pt : pAllCurve->points) {
//         // 投影时将3D点的x和y坐标互换，并应用比例缩放和偏移
//         int x = static_cast<int>(pt.x * m_dScale + m_offset.x);
//         int y = static_cast<int>(pt.y * m_dScale + m_offset.y);

//         // 检查是否在图像边界内
//         if (x >= 0 && x < m_iWidth && y >= 0 && y < m_iHeight)
//             circle(m_image, Point(y, x), 3, Scalar(255, 255, 255), -1); // 绘制白色小圆点
//         else
//             printf("拟合直线点云不在图像内 (%d , %d)\n",x,y);

//     }



// }
// #endif


// 显示投影后的图像
void Viewer::Display()
{
    imshow("Projected Point Cloud", m_image);
    
}


// 显示投影后的图像
cv::Mat Viewer::Display() const
{
    return m_image;
    
}

void Viewer::SaveImage(unsigned long long ullTime)
{
    std::string sFilePath = Image_folderPath + std::to_string(ullTime) + ".png";
    imwrite(sFilePath, m_image);
}


void Viewer::SaveImage(std::string folderPath, unsigned long long ullTime)
{
    std::string sFilePath = folderPath + std::to_string(ullTime) + ".png";
    imwrite(sFilePath, m_image);
}



#if 1
// 与点云同样方向

// 绘制图像坐标轴
// 这里车辆中心点会提高一点，要查原因
void Viewer::DrawAxes() {
    // 绘制x轴
    line(m_image, Point(0, m_offset.x), Point(m_iWidth, m_offset.x), Scalar(255, 0, 0), 1);  // 蓝色线
    putText(m_image, "X", Point(m_iWidth - 50, m_offset.x - 10), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 0, 0), 1);

    // 绘制y轴
    line(m_image, Point(m_offset.y, 0), Point(m_offset.y, m_iHeight), Scalar(255, 0, 0), 1); // 蓝色线
    putText(m_image, "Y", Point(m_offset.y + 10,  20), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0, 0, 255), 1);
    
}

void Viewer::DrawViewerNotes(){

    std::string white = "white: Init PointCloud";
    std::string green = "green: Clusters";
    std::string blue = "blue: Result Points";
    std::string red = "red: No Ground PointCloud";
    std::string yellow = "yellow: Result Lines";

    putText(m_image, white, Point(0 , m_offset.x + 50), 
                    FONT_HERSHEY_SIMPLEX, 1, Scalar(200, 200, 200), 1);

    putText(m_image, red, Point(m_offset.y + 60 , m_offset.x + 50), 
                    FONT_HERSHEY_SIMPLEX, 1, Scalar(200, 200, 200), 1);

    putText(m_image, green, Point(0 , m_offset.x + 80), 
                    FONT_HERSHEY_SIMPLEX, 1, Scalar(200, 200, 200), 1);

    putText(m_image, yellow, Point(m_offset.y + 60, m_offset.x + 80), 
                    FONT_HERSHEY_SIMPLEX, 1, Scalar(200, 200, 200), 1);

}


// 绘制与y轴平行的等距线（y=interval的倍数）
void Viewer::DrawEquidistantLines() 
{
    // 等距线间隔，单位为米
    double interval = 1.0;

    for (double x = -50; x <= 50; x += interval) {
        int u = static_cast<int>(x * m_dScale + m_offset.y); // 计算等距线在图像中的位置
        if(x == 0)
            continue;
        else{
            if (u >= 0 && u < m_iWidth) {  // 确保线在图像边界内
                // line(m_image, Point(y, 0), Point(y, m_iHeight), Scalar(200, 200, 200), 1); // 浅灰色
                putText(m_image, std::to_string(static_cast<int>(x)), Point(u, m_offset.x + 20), 
                        FONT_HERSHEY_SIMPLEX, 0.5, Scalar(200, 200, 200), 1);
            }
        }
        
        int five_Scale = 5 * m_dScale;
        if((int)u % five_Scale == 0){
            line(m_image, Point(u, m_offset.x - 20), Point(u, m_offset.x), Scalar(200, 200, 200), 1); // 浅灰色
        }
        else{
            line(m_image, Point(u, m_offset.x - 10), Point(u, m_offset.x), Scalar(200, 200, 200), 1); // 浅灰色
        }  
    }

}


// 绘制与x轴平行的等距线（x=interval的倍数）
void Viewer::DrawHorizontalEquidistantLines() 
{
    // 等距线间隔，单位为米
    double interval = 1.0;

    for (double y = -50; y <= 50; y += interval) {
        int v = static_cast<int>(-y * m_dScale + m_offset.x); // 计算等距线在图像中的位置   //若(-y * scale + offset.x)那么x轴上的数字就反了
        if (v > 0 && v < m_offset.x) {  // 确保线在图像边界内

            // line(m_image, Point(m_offset.y - 10, x), Point(m_offset.y + 10, x), Scalar(200, 200, 200), 1); // 浅灰色
            if( (int)y % 5 == 0){
                line(m_image, Point(m_offset.y - 20, v), Point(m_offset.y + 20, v), Scalar(200, 200, 200), 1); // 浅灰色    
                putText(m_image, std::to_string(static_cast<int>(y)) + "m", Point(m_offset.y + 10, v - 5), 
                        FONT_HERSHEY_SIMPLEX, 0.5, Scalar(200, 200, 200), 1);
            }
            else{
                line(m_image, Point(m_offset.y - 10, v), Point(m_offset.y + 10, v), Scalar(200, 200, 200), 1); // 浅灰色
            }
        }
    }
}


void Viewer::ProjectPointCloud(const PointCloud2Intensity::Ptr &pAllInit, const PointCloud2Intensity::Ptr &pAllNoGround, const PointCloud2RGB::Ptr &pAllCluster, const PointCloud2RGB::Ptr &pAllCurve)
{
    // 清空图像
    m_image.setTo(Scalar(0, 0, 0));   //黑色

    DrawAxes();
    DrawEquidistantLines();
    DrawHorizontalEquidistantLines();
    DrawViewerNotes();
/*
    if(!pAllInit->empty()){
        // 遍历原始点云
        for (const auto& pt : pAllInit->points) {
            
            // 投影时将3D点的x和y坐标互换，并应用比例缩放和偏移
            int x = static_cast<int>(-pt.y * m_dScale + m_offset.x);
            int y = static_cast<int>(pt.x * m_dScale + m_offset.y);

            // 检查是否在图像边界内
            if (x >= 0 && x < m_iWidth && y >= 0 && y < m_iHeight)
                circle(m_image, Point(y, x), 1, Scalar(255, 255, 255), -1); // 绘制白色小圆点
            // else
            //     printf("原始点云不在图像内 (%d , %d)\n",x,y);
        
        }
    }
*/
    if(!pAllNoGround->empty())
    {    
        // 遍历所有非地面点点云
        for (const auto& pt : pAllNoGround->points) {
            // 投影时将3D点的x和y坐标互换，并应用比例缩放和偏移
            int x = static_cast<int>(-pt.y * m_dScale + m_offset.x);
            int y = static_cast<int>(pt.x * m_dScale + m_offset.y);

            // 检查是否在图像边界内
            if (x >= 0 && x < m_iWidth && y >= 0 && y < m_iHeight)
                circle(m_image, Point(y, x), 2, Scalar(0, 0, 255), -1); // 绘制红色小圆点
            // else
            //     printf("非地面点点云不在图像内 (%d , %d)\n",x,y);

        }
    }

    if(!pAllCluster->empty())
    {
        // 遍历所有聚类簇点云
        for (const auto& pt : pAllCluster->points) {
            // 投影时将3D点的x和y坐标互换，并应用比例缩放和偏移
            int x = static_cast<int>(-pt.y * m_dScale + m_offset.x);
            int y = static_cast<int>(pt.x * m_dScale + m_offset.y);

            // 检查是否在图像边界内
            if (x >= 0 && x < m_iWidth && y >= 0 && y < m_iHeight){
                // Scalar color = pt.x > 0 ? Scalar(0, 255, 0) : Scalar(0, 0, 255); // 绿色(x轴>0)或红色(x轴<0)
                // circle(m_image, Point(y, x), 1, color, -1);
                circle(m_image, Point(y, x), 5, Scalar(0, 255, 0), -1); // 绘制绿色小圆点
            }
            else
                printf("聚类簇点云不在图像内 (%d , %d)\n",x,y);
        }
    }

    if(!pAllCurve->empty())
    {
        Point start_r, end_r, start_l, end_l;
        // std::cout<<"画图中，最后线上有: "<<pAllCurve->points.size()<<" 个点"<<std::endl;
        for(int i = 0; i < pAllCurve->points.size(); i++){
            int x = static_cast<int>(-pAllCurve->points[i].y * m_dScale + m_offset.x);
            int y = static_cast<int>(pAllCurve->points[i].x * m_dScale + m_offset.y);
            
            if(i == 0){
                start_r.x = y;
                start_r.y = x;
            }
            else if(i == 1){
                end_r.x = y;
                end_r.y = x;
            }
            else if(i == 2){
                start_l.x = y;
                start_l.y = x;
            }
            else{
                end_l.x = y;
                end_l.y = x;
            }
        }
        line(m_image, start_r, end_r, Scalar(0, 255, 255), 1); // 黄色
        line(m_image, start_l, end_l, Scalar(0, 255, 255), 1); // 黄色
        
        //遍历所有拟合直线点云
        // for (const auto& pt : pAllCurve->points) {
        //     // 投影时将3D点的x和y坐标互换，并应用比例缩放和偏移
        //     int x = static_cast<int>(-pt.y * m_dScale + m_offset.x);
        //     int y = static_cast<int>(pt.x * m_dScale + m_offset.y);

        //     // 检查是否在图像边界内
        //     if (x >= 0 && x < m_iWidth && y >= 0 && y < m_iHeight){
        //         circle(m_image, Point(y, x), 10, Scalar(255, 0, 0), -1); // 绘制蓝色小圆点
        //     }
        //     else
        //         printf("拟合直线点云不在图像内 (%d , %d)\n",x,y);

        // }
    }
}

void Viewer::ProjectCvLane(const std::vector<STR_CV_LANE_DATA> &vLaneData){
    // 清空图像
    m_image.setTo(Scalar(0, 0, 0));   //黑色

    DrawAxes();
    DrawEquidistantLines();
    DrawHorizontalEquidistantLines();
    
    if(!vLaneData.empty()){

        for(int idx = 0; idx < vLaneData.size(); idx++){

            for(int j = 0; j < vLaneData[idx].iCounts; j++){

                Point start, end;
                //根据测试打印结果，调转x，y值
                float temp_start_x  = vLaneData[idx].pstrCVLane[j].pstrPoint2fBirdEyeView[0].fY;
                float temp_start_y  = vLaneData[idx].pstrCVLane[j].pstrPoint2fBirdEyeView[0].fX;
                float temp_end_x    = vLaneData[idx].pstrCVLane[j].pstrPoint2fBirdEyeView[1].fY;
                float temp_end_y    = vLaneData[idx].pstrCVLane[j].pstrPoint2fBirdEyeView[1].fX;
            
                start.x = static_cast<int>(temp_start_x * m_dScale + m_offset.y);
                start.y = static_cast<int>(-temp_start_y * m_dScale + m_offset.x);
                // std::cout<<"start<"<<start.x<<" , "<<start.y<<">"<<std::endl;

                end.x = static_cast<int>(temp_end_x * m_dScale + m_offset.y);
                end.y = static_cast<int>(-temp_end_y * m_dScale + m_offset.x);
                // std::cout<<"end<"<<end.x<<" , "<<end.y<<">"<<std::endl;

    
                if(temp_start_x > 0)  //左黄（0，255，255），右白
                    line(m_image, start, end, Scalar(255, 255, 255), 3); // 
                else
                    line(m_image, start, end, Scalar(255, 255, 255), 3);
                circle(m_image, start, 2, Scalar(0, 0, 255), 3); // 绘制红色小圆点
            }

        }

        SaveImage("/home/zyl/echiev_lidar_curb_detection/log/LanePicture/",vLaneData[0].ullTimestamp);
    }    

}


cv::Mat Viewer::DrawLCDtempImage(const cv::Mat &result, const cv::Point2d &offset){

    //修改图片
    int top = 0;
    int bottom = 100 - top; // 
    int left = 0;
    int right = 0;

    cv::Mat newImage;
    cv::copyMakeBorder(result, newImage, top, bottom, left, right, BORDER_CONSTANT, Scalar(0, 0, 0));

    //图片下面加注释
    std::string white = "white: All contours";
    std::string blue = "blue: contour less than 25";
    std::string red = "red: direction less than 60 degrees";
    std::string green = "green: result";
    // line(result, Point(0, offset.x -10), Point(iWidth, offset.x -10), Scalar(255, 255, 255), 1);  // 白色线
    
    line(newImage, Point(0, offset.x ), Point(m_iHough_width, offset.x ), Scalar(255, 255, 255), 3);  // 白色线
    line(newImage, Point(offset.y, 0), Point(offset.y, m_iHough_height), Scalar(255, 255, 255), 1); // 白色线
    putText(newImage, "Y", Point(offset.y + 10,  20), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 0, 0), 1);
    putText(newImage, "X", Point(m_iHough_width - 50, offset.x - 10), FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255, 0, 0), 1);
    
    circle(newImage, Point(offset.y, offset.x), 5, Scalar(0, 0, 255), -1); // 绘制红色小圆点
    // 等距线间隔，单位为米
    double interval_x = 5.0;
    // 计算等距线在图像中的位置并绘制
    for (double y = -50; y <= 50; y += interval_x) {
        int x = static_cast<int>(-y * m_iHough_scale + offset.x); // 计算等距线在图像中的位置   //若(-y * scale + offset.x)那么x轴上的数字就反了
        if (x >= 0 && x < m_iHough_height) {  // 确保线在图像边界内
            // line(m_image, Point(0, x), Point(m_iWidth, x), Scalar(200, 200, 200), 1); // 浅灰色
            putText(newImage, std::to_string(static_cast<int>(y)) + "m", Point(offset.y + 10, x - 5), 
                    FONT_HERSHEY_SIMPLEX, 0.5, Scalar(200, 200, 200), 1);
        }
    }

    // 等距线间隔，单位为米
    double interval_y= 1.0;
    // 计算等距线在图像中的位置并绘制
    for (double x = -50; x <= 50; x += interval_y) {
        int y = static_cast<int>(x * m_iHough_scale + offset.y); // 计算等距线在图像中的位置
        if(x == 0)
            continue;
        else{
            if (y >= 0 && y < m_iHough_width) {  // 确保线在图像边界内
                // line(image, Point(y, 0), Point(y, height), Scalar(200, 200, 200), 1); // 浅灰色
                putText(newImage, std::to_string(static_cast<int>(x)), Point(y, offset.x + 15), 
                        FONT_HERSHEY_SIMPLEX, 0.5, Scalar(200, 200, 200), 1);
            }
        }   
    }


    putText(newImage, white, Point(0, offset.x + 35 ), 
                    FONT_HERSHEY_SIMPLEX, 0.6, Scalar(200, 200, 200), 1);
    
    putText(newImage, blue, Point(0, offset.x + 55), 
                    FONT_HERSHEY_SIMPLEX, 0.6, Scalar(200, 200, 200), 1);

    putText(newImage, red, Point(0, offset.x + 75), 
                    FONT_HERSHEY_SIMPLEX, 0.6, Scalar(200, 200, 200), 1);
    
    putText(newImage, green, Point(0, offset.x + 90), 
                    FONT_HERSHEY_SIMPLEX, 0.6, Scalar(200, 200, 200), 1);
    // putText(newImage, green, Point(offset.y-80, offset.x + 10), 
    //                 FONT_HERSHEY_SIMPLEX, 0.7, Scalar(200, 200, 200), 1);

   
    return newImage;
}

}

#endif