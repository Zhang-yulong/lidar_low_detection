#include "CommonGroundDetection.h"


namespace Lidar_Low_Detection
{
    
CommonGroundDetection::CommonGroundDetection(const SELF_DEBUG_CONFIG & config)
    : m_CGDConfig(&config)
{
   
    // m_pGLD_CurveFitting = new CurveFitting(m_strGLDConfig);
}

CommonGroundDetection::~CommonGroundDetection(){

    // delete(m_pGLD_CurveFitting);
}

std::pair<PointCloud2Intensity::Ptr, PointCloud2Intensity::Ptr> CommonGroundDetection::CommonGroundSegmentStart(PointCloud2Intensity::Ptr &pInCloud, const unsigned long long &ullTime){

    // PointCloud2Intensity::Ptr pGroundPoints(new PointCloud2Intensity);
    // PointCloud2Intensity::Ptr pNoGroundPoints(new PointCloud2Intensity);

    PointCloud2Intensity::Ptr pGround_i(new PointCloud2Intensity);
    PointCloud2Intensity::Ptr pNo_Ground_i(new PointCloud2Intensity);
    pcl::PointIndicesPtr pInliers(new pcl::PointIndices);   
    
    int iMaxWindowSize = m_CGDConfig->maxWindowSize;
    float fSlope = m_CGDConfig->slope;
    float fInitialDistance = m_CGDConfig->initialDistance;
    float fMaxDistance = m_CGDConfig->maxDistance;
    int iCellSize = m_CGDConfig->cellSize;
    int iBase = m_CGDConfig->base;


    if(!pInCloud->empty()){

        // 渐进式形态学滤波
        pcl::ApproximateProgressiveMorphologicalFilter<pcl::PointXYZI> segmentation;
        segmentation.setInputCloud(pInCloud);						 // 待处理点云
        segmentation.setMaxWindowSize(iMaxWindowSize);		 // 最大窗口大小
        segmentation.setSlope(fSlope);										 // 地形坡度参数
        segmentation.setInitialDistance(fInitialDistance);      // 初始高差阈值
        segmentation.setMaxDistance(fMaxDistance);			    // 最大高差阈值
        segmentation.setCellSize(iCellSize);						// 设置窗口的大小
        segmentation.setBase(iBase);										// 设置计算渐进窗口大小时使用的基数
        segmentation.setExponential(true);									// 设置是否以指数方式增加窗口大小
        segmentation.extract(pInliers->indices);

        pcl::ExtractIndices<pcl::PointXYZI> extract_ground;
        extract_ground.setNegative(false);   //设置提取内点
        extract_ground.setIndices(pInliers);   //设置分割后的内点为需要提取的点集
        extract_ground.setInputCloud(pInCloud);
        extract_ground.filter(*pGround_i);  //开始分割


        pcl::ExtractIndices<pcl::PointXYZI> extract_no_ground;
        extract_no_ground.setNegative(true);   //设置提取内点
        extract_no_ground.setIndices(pInliers);   //设置分割后的内点为需要提取的点集
        extract_no_ground.setInputCloud(pInCloud);
        extract_no_ground.filter(*pNo_Ground_i);  //开始分割
    }

    LOG_RAW("[APMF处理] pNo_Ground_i 未过滤前的总数：%zu, pGround_i 未过滤前的总数：%zu\n", pNo_Ground_i->points.size(), pGround_i->points.size());

    if(!pGround_i->empty())
    {
        PointCloud2Intensity::Ptr temp(new PointCloud2Intensity);
        PointCloud2Intensity::Ptr SaveGround(new PointCloud2Intensity);

        // std::cout<<"pGround_i 未过滤前的总数： "<<pGround_i->points.size()<<" pNo_Ground_i 未过滤前的总数： "<<pNo_Ground_i->points.size()<<std::endl;
        // LOG_RAW("[APMF处理] pNo_Ground_i 未过滤前的总数：%zu, pGround_i 未过滤前的总数：%zu\n", pNo_Ground_i->points.size(), pGround_i->points.size());
        unsigned int count = 0; 
//         for(int i = 0; i < pGround_i->points.size(); i++ ){
//             pcl::PointXYZI tempPoint;
//             if( pGround_i->points[i].z < m_strGLDConfig.GL_heightUpper && pGround_i->points[i].z > m_strGLDConfig.GL_heightLower && 
//             pGround_i->points[i].intensity < m_strGLDConfig.GL_intensityUpper && pGround_i->points[i].intensity > m_strGLDConfig.GL_intensityLower 
//                 //&& pGround_i->points[i].x < 2.0 && pGround_i->points[i].x > 1.5
//             )
            
//             {
//                 count++;
//                 tempPoint.x = pGround_i->points[i].x;
//                 tempPoint.y = pGround_i->points[i].y;
//                 tempPoint.z = pGround_i->points[i].z;
//                 tempPoint.intensity = pGround_i->points[i].intensity;
//                 // std::cout<<"强度: "<<tempPoint.intensity<<std::endl;
//                 temp->points.push_back(tempPoint);
                
//                 //保存地面的pcd
//                 SaveGround->points.push_back(tempPoint);
//             }
//         }
// std::cout<<"过滤后的总数： "<<count<<std::endl;
        //保存地面的pcd
        // SaveGround->height = SaveGround->points.size();
        // SaveGround->width = 1;
        // SaveGround->is_dense = false;
        // char pchFileName_2[128];
        // bzero(pchFileName_2, sizeof (pchFileName_2));
        // sprintf(pchFileName_2, "/home/zyl/echiev_lidar_curb_detection/log/pcdRunningModel/Ground.pcd");
        // pcl::io::savePCDFileASCII (pchFileName_2, *SaveGround);

        // *pGround_i = *temp;
    }
    else{
        std::cout<<"地面点为空"<<std::endl;
    }
    
    if(pGround_i->empty()){
        return {nullptr,nullptr};}
    
    return {pNo_Ground_i, pGround_i};
}

}
