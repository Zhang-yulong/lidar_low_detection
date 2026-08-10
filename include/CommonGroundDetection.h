#pragma once

#ifndef COMMON_GROUND_DETECTION_H
#define COMMON_GROUND_DETECTION_H

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



#include "Type.h"
#include "CurveFitting.h"
#include "Viewer.h"

#include "struct_typedef.h"
#include "communication.h"

namespace Lidar_Low_Detection
{


class CommonGroundDetection{

public:

    CommonGroundDetection(const SELF_DEBUG_CONFIG & config);
    ~CommonGroundDetection();

    std::pair<PointCloud2Intensity::Ptr, PointCloud2Intensity::Ptr> CommonGroundSegmentStart(PointCloud2Intensity::Ptr &pInCloud, const unsigned long long &ullTime);

private: 

    const SELF_DEBUG_CONFIG *m_CGDConfig;
    // CurveFitting *m_pGLD_CurveFitting;
};

}


#endif