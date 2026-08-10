/**
 * @file DebugViewer.cpp
 * @brief 算法调试可视化模块实现
 *
 * 所有 OpenCV 绘图逻辑封装在此文件中。
 * GroundFilter 不直接依赖 OpenCV，只通过 DebugViewer 接口间接使用。
 */

#include "DebugViewer.h"
#include <cmath>
#include <algorithm>
#include <sys/stat.h>

// #include "ReadYamlFile.h"
// #include "ElevationMapGroundFilter.h"
// #include "opencv2/opencv.hpp"
namespace Lidar_Low_Detection
{

// ============================================================================
// 构造
// ============================================================================

DebugViewer::DebugViewer(const SELF_DEBUG_CONFIG& config)
    : m_DV_config(config)
    , m_frameCount(0)
{



    // 如果启用保存模式，确保保存目录存在
    if (!config.debug_save_dir.empty())
    {
        mkdir(config.debug_save_dir.c_str(), 0755);
    }
}



int DebugViewer::GetFrameCountValue() const {
        return m_frameCount;
    }



// ============================================================================
// 内部辅助：统一的 show / save 处理
// ============================================================================

void DebugViewer::ShowOrSave(const cv::Mat& image, const std::string& name,
                              const DEBUG_VIEWER_CONFIG& viewCfg)
{
    if (!viewCfg.enable)
    {
        return;
    }

    if (viewCfg.show)
    {
        cv::namedWindow(name, cv::WINDOW_NORMAL);
        cv::imshow(name, image);
    }

    if (viewCfg.save)
    {
        LOG_RAW("第%d张\n",m_frameCount);
        std::string filepath = m_DV_config.debug_save_dir + "/" +
                               std::to_string(m_frameCount) + "_" + name + ".png";
        cv::imwrite(filepath, image);
    }
}


void DebugViewer::AddBackGround(cv::Mat &image, const int img_w, const int img_h, const ElevationGridConfig& gridCfg){

    // 2. 基础参数计算
    float inv_resolution = 1.0f / gridCfg.grid_resolution;

    // 6. 计算物理原点 (0,0) 在图像上的位置
    // 物理 x=0 位于盲区左侧。
    // 像素 X = (expand/2) + (0 - roi_x_min) * inv_res * scale
    // 注意：如果 roi_x_min=0，则 px_origin_x = expand/2。
    // 之前的代码中 px_blind_offset 是基于 car_half_x 算的，这里我们要还原真实的物理 0 点。
    int px_origin_x = static_cast<int>((0.0f - gridCfg.roi_x_min) * inv_resolution * kGridPixelScale) + (expand_img_w / 2);
    int origin_row = static_cast<int>((0.0f - gridCfg.roi_y_min) * inv_resolution);
    int px_origin_y = (origin_row * kGridPixelScale) + (expand_img_h / 2);

    // 7. 绘制坐标轴 (红色)
    cv::Scalar axisColor(0, 0, 255); 
    int axisThickness = 2;

    // X 轴 (水平)
    cv::line(image, cv::Point(0, px_origin_y), cv::Point(img_w, px_origin_y), axisColor, axisThickness);
    cv::arrowedLine(image, cv::Point(img_w - 30, px_origin_y), cv::Point(img_w - 5, px_origin_y), axisColor, axisThickness, 8, 0.05);
    cv::putText(image, "Col++ (X+)", cv::Point(img_w - 100, px_origin_y - 10), cv::FONT_HERSHEY_SIMPLEX, 0.4, axisColor, 1);

    // Y 轴 (垂直)
    cv::line(image, cv::Point(px_origin_x, 0), cv::Point(px_origin_x, img_h), axisColor, axisThickness);
    cv::arrowedLine(image, cv::Point(px_origin_x, 30), cv::Point(px_origin_x, 5), axisColor, axisThickness, 8, 0.05);
    cv::putText(image, "Row++ (Y+ Left)", cv::Point(px_origin_x + 10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.4, axisColor, 1);

    // 标记物理原点
    cv::circle(image, cv::Point(px_origin_x, px_origin_y), 5, axisColor, -1);
    cv::putText(image, "Car(0,0)", cv::Point(px_origin_x + 8, px_origin_y - 8), cv::FONT_HERSHEY_SIMPLEX, 0.4, axisColor, 1);

    // 8. 绘制 1米大网格线 (基于物理坐标循环)
    cv::Scalar gridLineColor(100, 100, 100); 
    int lineThickness = 1;
    int cells_per_meter = static_cast<int>(std::round(1.0f / gridCfg.grid_resolution));
    if (cells_per_meter <= 0) cells_per_meter = 1;

    // --- 绘制水平网格线 (Y轴方向) ---
    // 遍历物理 Y 坐标 (从 roi_y_max 到 roi_y_min)
    for (float phys_y = gridCfg.roi_y_max; phys_y >= gridCfg.roi_y_min - 0.01f; phys_y -= 1.0f) {
        // 计算该物理 Y 对应的 Row 索引
        int r = static_cast<int>(std::round((gridCfg.roi_y_max - phys_y) * inv_resolution));
        
        // 转换为像素 Y
        int py = (r * kGridPixelScale) + (expand_img_h / 2);
        
        // 画横线
        cv::line(image, cv::Point(0, py), cv::Point(img_w, py), gridLineColor, lineThickness);

        // 刻度与标签
        int tick_len = 5;
        cv::line(image, cv::Point(px_origin_x - tick_len, py), cv::Point(px_origin_x + tick_len, py), axisColor, lineThickness);
        
        char buf[16];
        snprintf(buf, sizeof(buf), "%.1f", phys_y);
        cv::putText(image, buf, cv::Point(px_origin_x + tick_len + 5, py), cv::FONT_HERSHEY_SIMPLEX, 0.3, axisColor, 1);
    }

    // --- 绘制垂直网格线 (X轴方向) ---
    // 遍历物理 X 坐标 (从 0.0 到 6.0)
    // 注意：这里必须覆盖盲区 (0~0.9) 和 有效区 (0.9~6.0)
    for (float phys_x = gridCfg.roi_x_min; phys_x <= gridCfg.roi_x_max + 0.01f; phys_x += 1.0f) {
        // 计算该物理 X 对应的像素位置
        // 公式：边距 + (物理距离 * 分辨率倒数 * 像素比例)
        int px = static_cast<int>((phys_x - gridCfg.roi_x_min) * inv_resolution * kGridPixelScale) + (expand_img_w / 2);

        // 画竖线
        cv::line(image, cv::Point(px, 0), cv::Point(px, img_h), gridLineColor, lineThickness);

        // 刻度与标签
        int tick_len = 5;
        cv::line(image, cv::Point(px, px_origin_y - tick_len), cv::Point(px, px_origin_y + tick_len), axisColor, lineThickness);

        char buf[16];
        snprintf(buf, sizeof(buf), "%.1f", phys_x);
        // 标签位置：稍微偏下
        cv::putText(image, buf, cv::Point(px + 2, px_origin_y + tick_len + 12), cv::FONT_HERSHEY_SIMPLEX, 0.3, axisColor, 1);
    }
}




// ============================================================================
// 内部辅助：Z 值 → 热力图颜色 (BGR)
// 蓝(低) → 绿(中) → 黄(高) → 红(最高)
// ============================================================================

cv::Vec3b DebugViewer::ZValueToColor(float z_val, float z_min, float z_max)
{
    if (z_max <= z_min)
    {
        return cv::Vec3b(128, 128, 128);  // 灰色 fallback
    }

    // 归一化到 [0, 1]
    float t = (z_val - z_min) / (z_max - z_min);
    t = std::max(0.0f, std::min(1.0f, t));

    // 4 段颜色插值: 蓝→绿→黄→红
    // 0.00 ~ 0.33: 蓝→绿
    // 0.33 ~ 0.66: 绿→黄
    // 0.66 ~ 1.00: 黄→红
    float r, g, b;
    if (t < 0.33f)
    {
        float s = t / 0.33f;
        r = 0.0f;
        g = s * 255.0f;
        b = (1.0f - s) * 255.0f;
    }
    else if (t < 0.66f)
    {
        float s = (t - 0.33f) / 0.33f;
        r = s * 255.0f;
        g = 255.0f;
        b = 0.0f;
    }
    else
    {
        float s = (t - 0.66f) / 0.34f;
        r = 255.0f;
        g = (1.0f - s) * 255.0f;
        b = 0.0f;
    }

    return cv::Vec3b(static_cast<uchar>(b),
                     static_cast<uchar>(g),
                     static_cast<uchar>(r));
}

// ============================================================================
// 内部辅助：Slope → 热力图颜色 (BGR)
// ============================================================================

cv::Vec3b DebugViewer::SlopeToColor(float slope, float slope_max)
{
    if (slope_max <= 0.0f)
    {
        return cv::Vec3b(128, 128, 128);
    }

    float t = slope / slope_max;
    t = std::max(0.0f, std::min(1.0f, t));

    // 蓝→绿→黄→红
    float r, g, b;
    if (t < 0.33f)
    {
        float s = t / 0.33f;
        r = 0.0f;  g = s * 255.0f;  b = (1.0f - s) * 255.0f;
    }
    else if (t < 0.66f)
    {
        float s = (t - 0.33f) / 0.33f;
        r = s * 255.0f;  g = 255.0f;  b = 0.0f;
    }
    else
    {
        float s = (t - 0.66f) / 0.34f;
        r = 255.0f;  g = (1.0f - s) * 255.0f;  b = 0.0f;
    }

    return cv::Vec3b(static_cast<uchar>(b),
                     static_cast<uchar>(g),
                     static_cast<uchar>(r));
}

// ============================================================================
// 内部辅助：Label → 伪彩色
// ============================================================================

cv::Vec3b DebugViewer::LabelToColor(int label)
{
    if (label < 0)
    {
        return cv::Vec3b(40, 40, 40);  // 未标记: 深灰
    }
    // 使用 golden ratio 生成分布均匀的伪彩色
    float hue = std::fmod(label * 0.618033988749895f, 1.0f);
    float s = 0.85f;
    float v = 0.90f;

    // HSV → RGB 简易转换
    float c = v * s;
    float x = c * (1.0f - std::abs(std::fmod(hue * 6.0f, 2.0f) - 1.0f));
    float m = v - c;

    float r, g, b;
    int h_seg = static_cast<int>(hue * 6.0f);
    switch (h_seg)
    {
        case 0: r = c; g = x; b = 0; break;
        case 1: r = x; g = c; b = 0; break;
        case 2: r = 0; g = c; b = x; break;
        case 3: r = 0; g = x; b = c; break;
        case 4: r = x; g = 0; b = c; break;
        default: r = c; g = 0; b = x; break;
    }

    return cv::Vec3b(static_cast<uchar>((b + m) * 255),
                     static_cast<uchar>((g + m) * 255),
                     static_cast<uchar>((r + m) * 255));
}

// ============================================================================
// 内部辅助：世界坐标 → 像素坐标 (用于 Overlay 图)
// ============================================================================

void DebugViewer::WorldToPixel(float wx, float wy, int& px, int& py,
                                const ElevationGridConfig& gridCfg) const
{
    // Overlay 图: x(前向)→列, y(左右)→行
    // 将 ROI 映射到 [0, kOverlayWidth-1] × [0, kOverlayHeight-1]
    float norm_x = (wx - gridCfg.roi_x_min) / (gridCfg.roi_x_max - gridCfg.roi_x_min);
    float norm_y = (wy - gridCfg.roi_y_min) / (gridCfg.roi_y_max - gridCfg.roi_y_min);

    px = static_cast<int>(norm_x * (kOverlayWidth  - 1));
    py = static_cast<int>((1.0f - norm_y) * (kOverlayHeight - 1));  // Y 翻转

    px = std::max(0, std::min(kOverlayWidth  - 1, px));
    py = std::max(0, std::min(kOverlayHeight - 1, py));
}

// ============================================================================
// 1. DrawHeightMap —— Grid 高程热力图
// ============================================================================
#if 0
void DebugViewer::DrawHeightMap(const std::vector<GridCell>& grid,
                                 int rows, int cols,
                                 const ElevationGridConfig& gridCfg)
{
    const auto& viewCfg = m_DV_config.HeightMap;
    if (!viewCfg.enable) return;

    // 统计有效 cell 的 min_z 范围
    float z_min =  FLT_MAX;
    float z_max = -FLT_MAX;
    for (const auto& cell : grid)
    {
        if (cell.valid)
        {
            z_min = std::min(z_min, cell.min_z);
            z_max = std::max(z_max, cell.min_z);
        }
    }

    float inv_resolution = 1.0f / gridCfg.grid_resolution;;

    // 创建图像: 每 cell 映射 kGridPixelScale×kGridPixelScale 像素
    int expand_img_w = 100; //100
    int expand_img_h = 200;//200
    // // int img_w = ((m_DV_config.roi_x_max - m_DV_config.roi_x_min) * inv_resolution) * kGridPixelScale  + expand_img_w;
    // // int img_h = ((m_DV_config.roi_y_max - m_DV_config.roi_y_min) * inv_resolution) * kGridPixelScale + expand_img_h;
    int img_w = (cols * kGridPixelScale) + expand_img_w;
    int img_h = (rows * kGridPixelScale) + expand_img_h;
    cv::Mat image(img_h, img_w, CV_8UC3, cv::Scalar(40, 40, 40));  // 深灰背景
     

    // int blind_cols = static_cast<int>((gridCfg.car_half_x + gridCfg.body_filter_x_threshold) / gridCfg.grid_resolution); 
    // // 像素偏移量：盲区占据的像素宽度
    // int px_blind_offset = blind_cols * kGridPixelScale;
    // int img_w = px_blind_offset + (cols * kGridPixelScale) + expand_img_w;
    // int img_h = (rows * kGridPixelScale) + expand_img_h;
    // cv::Mat image(img_h, img_w, CV_8UC3, cv::Scalar(40, 40, 40));  // 深灰背景
     
#if 1
   
    for (int r = 0; r < rows; ++r)
    {
        for (int c = 0; c < cols; ++c)
        {
            int idx = r * cols + c;
            const GridCell& cell = grid[static_cast<size_t>(idx)];

            cv::Vec3b color;
            if (cell.valid)
            {
                color = ZValueToColor(cell.min_z, z_min, z_max);
            }
            else
            {
                color = cv::Vec3b(60, 60, 60);  // 无效: 深灰
            }

            // 填充像素块
            int px_start = c * kGridPixelScale + (expand_img_w / 2);
            // int px_start = c * kGridPixelScale + px_blind_offset + (expand_img_w / 2);
            int py_start = (r * kGridPixelScale) + (expand_img_h / 2); //看看要不要取整？
            cv::Rect roi(px_start, py_start, kGridPixelScale, kGridPixelScale);
            // 边界保护
            roi &= cv::Rect(0, 0, img_w, img_h);
            if (roi.width > 0 && roi.height > 0)
            {
                image(roi) = color;
            }
        }
    }
    // ── 2. Debug: 绘制物理原点与 Rows/Cols 坐标轴 ──
    // 1. 计算物理原点 (x=0.9, y=0) 在图像上的像素坐标
    // 物理 x=0.9 对应 col=0; 物理 y=0 对应 row=30
    int origin_col = 0;
    int origin_row = static_cast<int>((0.0f - gridCfg.roi_y_min) * inv_resolution); // 计算 y=0 对应的 row
    int px_origin_x = origin_col * kGridPixelScale + (expand_img_w / 2);
    // int px_origin_x = -blind_cols * kGridPixelScale + (expand_img_w / 2);
    int px_origin_y = (origin_row * kGridPixelScale) + (expand_img_h / 2);

    // 2. 绘制坐标轴 (红色)
    cv::Scalar axisColor(0, 0, 255); // BGR 红色
    int axisThickness = 2;
    // X 轴 (水平向右，代表 Col++ / 前向)
    cv::line(image, 
             cv::Point(px_origin_x, px_origin_y), 
             cv::Point(img_w, px_origin_y), 
             axisColor, axisThickness);
    // X 轴箭头与标签
    cv::arrowedLine(image, 
                    cv::Point(img_w - 30, px_origin_y), 
                    cv::Point(img_w - 5, px_origin_y), 
                    axisColor, axisThickness, 8, 0.05);
    cv::putText(image, "Col++ (X+)", 
                cv::Point(img_w - 100, px_origin_y - 10),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, axisColor, 1);

    // Y 轴 (垂直向下，代表 Row++ / 左向)
    cv::line(image, 
             cv::Point(px_origin_x, px_origin_y), 
             cv::Point(px_origin_x, 0),     // 向上画到图像顶部
             axisColor, axisThickness);
    // Y 轴箭头与标签
    cv::arrowedLine(image, 
                    cv::Point(px_origin_x, 30), 
                    cv::Point(px_origin_x, 5),  // 箭头指向上方
                    axisColor, axisThickness, 8, 0.05);
    cv::putText(image, "Row++ (Y+ Left)", 
                cv::Point(px_origin_x + 10, 20), // 标签放在顶部
                cv::FONT_HERSHEY_SIMPLEX, 0.4, axisColor, 1);
    // 3. 标记物理原点 (0,0)
    cv::circle(image, cv::Point(px_origin_x, px_origin_y), 5, axisColor, -1); // 红色实心圆
    cv::putText(image, "Car(0,0)", 
                cv::Point(px_origin_x + 8, px_origin_y - 8),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, axisColor, 1);

    
    cv::Scalar gridLineColor(100, 100, 100); // 浅灰色网格线
    int lineThickness = 1;

    // 计算 1 米对应的 Grid Cell 数量
    // 例如: yaml文件grid_resolution=0.1 -> 10个格子; yaml文件grid_resolution=0.2 -> 5个格子
    int cells_per_meter = static_cast<int>(std::round(1.0f / gridCfg.grid_resolution));
    // 防止除零或配置异常
    if (cells_per_meter <= 0) cells_per_meter = 1; 

    // 绘制水平线 (代表 Rows 的边界)
    for (int r = 0; r <= rows; ++r) 
    {
        // 每满 cells_per_meter 个格子画一条线 (包含 r=0 的起始线)
        if (r % cells_per_meter == 0)
        {
            int y = (r * kGridPixelScale) + (expand_img_h / 2);
            cv::line(image, cv::Point(0, y), cv::Point(img_w, y), gridLineColor, lineThickness);
        
            // 2. 绘制刻度线 (在 Y 轴左侧画一个短横线)
            int tick_len = 5; 
            cv::line(image, 
                    cv::Point(px_origin_x - tick_len, y), 
                    cv::Point(px_origin_x + tick_len, y), 
                    axisColor, lineThickness);

            // 3. 计算并绘制物理 Y 坐标标签
            // 物理 Y 坐标 = roi_y_max - r * grid_resolution
            float phys_y = gridCfg.roi_y_max - (r * gridCfg.grid_resolution);
            char buf[16];
            snprintf(buf, sizeof(buf), "%.1f", phys_y);
            
            // 标签放在刻度线左侧，稍微向上偏移以对齐基线
            cv::putText(image, buf, 
                        cv::Point(px_origin_x + tick_len + 5, y),
                        cv::FONT_HERSHEY_SIMPLEX, 0.3, axisColor, 1);
        }
    }

    // 绘制垂直线 (代表 Cols 的边界)
    for (int c = 0; c <= cols; ++c) 
    {
        // 每满 cells_per_meter 个格子画一条线 (包含 c=0 的起始线)
        if (c % cells_per_meter == 0)
        {
            int x = c * kGridPixelScale + (expand_img_w / 2);
            // int x = c * kGridPixelScale + px_blind_offset + (expand_img_w / 2);
            cv::line(image, cv::Point(x, 0), cv::Point(x, img_h), gridLineColor, lineThickness);
        
            // 2. 绘制刻度线 (在 X 轴下方画一个短竖线)
            int tick_len = 5;
            cv::line(image, 
                    cv::Point(x, px_origin_y - tick_len), 
                    cv::Point(x, px_origin_y + tick_len), 
                    axisColor, lineThickness);

            // 3. 计算并绘制物理 X 坐标标签
            // // 物理 X 坐标 = roi_x_min + c * grid_resolution
            // float phys_x = gridCfg.roi_x_min + (c * gridCfg.grid_resolution);

            // 物理 X 坐标 = roi_x_min + x方向盲区距离 + c * grid_resolution
            float phys_x = gridCfg.roi_x_min + (gridCfg.car_half_x + gridCfg.body_filter_x_threshold) + (c * gridCfg.grid_resolution);
            char buf[16];
            snprintf(buf, sizeof(buf), "%.1f", phys_x);
            
            // 标签放在刻度线下方，稍微向右偏移防止与 Y 轴重叠
            cv::putText(image, buf, 
                        cv::Point(x + 2, px_origin_y + tick_len + 12),
                        cv::FONT_HERSHEY_SIMPLEX, 0.3, axisColor, 1);
            
        }
    }

    
    // // 在部分格子中心打印具体的 (r, c) 索引，方便核对
    // for (int r = 0; r < rows; r += std::max(1, rows / 5)) 
    // {
    //     for (int c = 0; c < cols; c += std::max(1, cols / 5))
    //     {
    //         char buf[16];
    //         snprintf(buf, sizeof(buf), "%d,%d", r, c);
    //         int textX = c * kGridPixelScale + 4;
    //         int textY = r * kGridPixelScale + kGridPixelScale / 2 + 3;
    //         cv::putText(image, buf, cv::Point(textX, textY),
    //                     cv::FONT_HERSHEY_SIMPLEX, 0.25, cv::Scalar(255, 255, 255), 1);
    //     }
    // }
    
    // // ── 3. (优化版) 在部分格子中心打印“物理坐标 (x, y)” ──
    // // 注意：为了符合“向上为左(Y+)”的视觉直觉，这里对 Y 轴进行了翻转
    // for (int r = 0; r < rows; r += std::max(1, rows / 5)) 
    // {
    //     for (int c = 0; c < cols; c += std::max(1, cols / 5))
    //     {
    //         // 物理 X 坐标：正常随 col 增加（向右/前向）
    //         float phys_x = gridCfg.roi_x_min + (c + 0.5f) * gridCfg.grid_resolution; 
            
    //         // 物理 Y 坐标：翻转映射！
    //         // 图像顶部 (r=0) 对应 roi_y_max (左侧, y=3.0)
    //         // 图像底部 (r=rows-1) 对应 roi_y_min (右侧, y=-3.0)
    //         float phys_y = gridCfg.roi_y_max - (r + 0.5f) * gridCfg.grid_resolution; 

    //         char buf[32];
    //         snprintf(buf, sizeof(buf), "(%.1f,%.1f)", phys_x, phys_y);
            
    //         int textX = c * kGridPixelScale + 2;
    //         int textY = (r * kGridPixelScale ) + (expand_img_h / 2) + kGridPixelScale / 2 + 3;
    //         cv::putText(image, buf, cv::Point(textX, textY),
    //                     cv::FONT_HERSHEY_SIMPLEX, 0.2, cv::Scalar(255, 255, 255), 1);
    //     }
    // }
#endif 




    // 添加标题文本
    char title[128];
    snprintf(title, sizeof(title), "HeightMap [%.2f, %.2f]m", z_min, z_max);
    cv::putText(image, title, cv::Point(10, img_h - 20),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

    ShowOrSave(image, "HeightMap", viewCfg);
}
#endif

void DebugViewer::DrawHeightMap(const std::vector<GridCell>& grid,
                                int rows, int cols,
                                const ElevationGridConfig& gridCfg)
{
    const auto& viewCfg = m_DV_config.HeightMap;
    if (!viewCfg.enable) return;

    // 1. 统计有效 cell 的 min_z 范围 (用于颜色映射)
    float z_min = FLT_MAX;
    float z_max = -FLT_MAX;
    for (const auto& cell : grid) {
        if (cell.valid) {
            z_min = std::min(z_min, cell.min_z);
            z_max = std::max(z_max, cell.min_z);
        }
    }

    // // 2. 基础参数计算
    // float inv_resolution = 1.0f / gridCfg.grid_resolution;

    
    // 【关键】计算物理盲区对应的格子数和像素偏移
    // 假设盲区是从 roi_x_min 开始的一段距离（例如车前 0.9m）
    // 如果你的盲区是固定的 0.9m，可以直接用 0.9f，或者使用配置项：
    float blind_dist_x = gridCfg.car_half_x + gridCfg.body_filter_x_threshold; 
    int blind_cols = static_cast<int>(std::round(blind_dist_x / gridCfg.grid_resolution));
    int px_blind_offset = blind_cols * kGridPixelScale;

    // // 3. 图像尺寸计算 (必须包含盲区宽度)
    // int expand_img_w = 100; 
    // int expand_img_h = 200; 
    
    // 总宽 = 盲区像素 + 有效网格像素 + 右侧边距
    int img_w = px_blind_offset + (cols * kGridPixelScale) + expand_img_w;
    // 总高 = 有效网格像素 + 上下边距 (假设Y轴无盲区，若有需同理增加)
    int img_h = (rows * kGridPixelScale) + expand_img_h;

    cv::Mat image(img_h, img_w, CV_8UC3, cv::Scalar(40, 40, 40)); 

    // 4. 统一坐标映射函数 (Lambda)
    // 将 Grid Index (r, c) 转换为 Image Pixel (x, y)
    // c=0 对应的是物理上的 blind_dist_x 处
    auto GridToPixel = [&](int c, int r) -> cv::Point {
        int px = px_blind_offset + (c * kGridPixelScale) + (expand_img_w / 2);
        int py = ((rows - 1 - r) * kGridPixelScale) + (expand_img_h / 2);
        return cv::Point(px, py);
    };

    // 5. 填充像素块 (Grid Cells)
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) {
            int idx = r * cols + c;
            const GridCell& cell = grid[static_cast<size_t>(idx)];

            cv::Vec3b color;
            if (cell.valid) {
                color = ZValueToColor(cell.min_z, z_min, z_max);
            } else {
                color = cv::Vec3b(60, 60, 60); // 无效区域深灰
            }

            cv::Point pStart = GridToPixel(c, r);
            cv::Rect roi(pStart.x, pStart.y, kGridPixelScale, kGridPixelScale);
            
            // 边界保护
            roi &= cv::Rect(0, 0, img_w, img_h);
            if (roi.width > 0 && roi.height > 0) {
                image(roi) = color;
            }
        }
    }

    AddBackGround(image, img_w, img_h, gridCfg);

    // 9. 标题
    char title[128];
    snprintf(title, sizeof(title), "HeightMap [%.2f, %.2f]m", z_min, z_max);
    cv::putText(image, title, cv::Point(10, img_h - 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

    ShowOrSave(image, "HeightMap", viewCfg);
}

// ============================================================================
// 2. DrawGroundMask —— Ground Label 分类图
// ============================================================================

void DebugViewer::DrawGroundMask(const std::vector<GridCell>& grid,
                                  int rows, int cols,
                                  const ElevationGridConfig& gridCfg)
{
    const auto& viewCfg = m_DV_config.GroundMask;
    if (!viewCfg.enable) return;

    
    float blind_dist_x = gridCfg.car_half_x + gridCfg.body_filter_x_threshold; 
    int blind_cols = static_cast<int>(std::round(blind_dist_x / gridCfg.grid_resolution));
    int px_blind_offset = blind_cols * kGridPixelScale;

    // 总宽 = 盲区像素 + 有效网格像素 + 右侧边距
    int img_w = px_blind_offset + (cols * kGridPixelScale) + expand_img_w;
    // 总高 = 有效网格像素 + 上下边距 (假设Y轴无盲区，若有需同理增加)
    int img_h = (rows * kGridPixelScale) + expand_img_h;

    // int img_w = cols * kGridPixelScale;
    // int img_h = rows * kGridPixelScale;
    cv::Mat image(img_h, img_w, CV_8UC3, cv::Scalar(40, 40, 40));

    int ground_cnt = 0, interp_cnt = 0, nonground_cnt = 0, invalid_cnt = 0;

    auto GridToPixel = [&](int c, int r) -> cv::Point {
        int px = px_blind_offset + (c * kGridPixelScale) + (expand_img_w / 2);
        int py = ((rows - 1 - r) * kGridPixelScale) + (expand_img_h / 2);
        return cv::Point(px, py);
    };


    for (int r = 0; r < rows; ++r)
    {
        for (int c = 0; c < cols; ++c)
        {
            int idx = r * cols + c;
            const GridCell& cell = grid[static_cast<size_t>(idx)];

            cv::Vec3b color;
            if (cell.is_interpolated_ground)
            {
                color = cv::Vec3b(0, 200, 255);  // 黄色/橙色: Interpolated Ground
                interp_cnt++;
            }
            else if (cell.valid && cell.is_ground)
            {
                color = cv::Vec3b(0, 220, 0);  // 绿色: Real Ground
                ground_cnt++;
            }
            else if (cell.valid && !cell.is_ground)
            {
                color = cv::Vec3b(0, 0, 220);  // 红色: Non-Ground
                nonground_cnt++;
            }
            else
            {
                color = cv::Vec3b(100, 100, 100);  // 灰色: Invalid Cell
                invalid_cnt++;
            }

            // int px_start = c * kGridPixelScale;
            // int py_start = r * kGridPixelScale;
            // cv::Rect roi(px_start, py_start, kGridPixelScale, kGridPixelScale);
            
            cv::Point pStart = GridToPixel(c, r);
            cv::Rect roi(pStart.x, pStart.y, kGridPixelScale, kGridPixelScale);
            
            roi &= cv::Rect(0, 0, img_w, img_h);
            if (roi.width > 0 && roi.height > 0)
            {
                image(roi) = color;
            }
        }
    }

    AddBackGround(image, img_w, img_h, gridCfg);

    char title[128];
    snprintf(title, sizeof(title), "GroundMask RealG:%d InterpG:%d NG:%d Inv:%d",
             ground_cnt, interp_cnt, nonground_cnt, invalid_cnt);
    cv::putText(image, title, cv::Point(10, 20),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

    ShowOrSave(image, "GroundMask", viewCfg);
}

// ============================================================================
// 3. DrawSlope —— 坡度热力图
// ============================================================================

void DebugViewer::DrawSlope(const std::vector<GridCell>& grid,
                             int rows, int cols,
                             const ElevationGridConfig& gridCfg)
{
    const auto& viewCfg = m_DV_config.Slope;
    if (!viewCfg.enable) return;

    // 统计有效 cell 的坡度范围
    float slope_max = 0.0f;
    for (const auto& cell : grid)
    {
        if (cell.valid && cell.slope < FLT_MAX)
        {
            slope_max = std::max(slope_max, cell.slope);
        }
    }
    // 上限截断，让颜色映射更合理
    float display_max = std::min(slope_max, 2.0f);

    // int img_w = cols * kGridPixelScale;
    // int img_h = rows * kGridPixelScale;

    float blind_dist_x = gridCfg.car_half_x + gridCfg.body_filter_x_threshold; 
    int blind_cols = static_cast<int>(std::round(blind_dist_x / gridCfg.grid_resolution));
    int px_blind_offset = blind_cols * kGridPixelScale;
    // 总宽 = 盲区像素 + 有效网格像素 + 右侧边距
    int img_w = px_blind_offset + (cols * kGridPixelScale) + expand_img_w;
    // 总高 = 有效网格像素 + 上下边距 (假设Y轴无盲区，若有需同理增加)
    int img_h = (rows * kGridPixelScale) + expand_img_h;
    cv::Mat image(img_h, img_w, CV_8UC3, cv::Scalar(40, 40, 40));

    auto GridToPixel = [&](int c, int r) -> cv::Point {
        int px = px_blind_offset + (c * kGridPixelScale) + (expand_img_w / 2);
        int py = ((rows - 1 - r) * kGridPixelScale) + (expand_img_h / 2);
        return cv::Point(px, py);
    };

    for (int r = 0; r < rows; ++r)
    {
        for (int c = 0; c < cols; ++c)
        {
            int idx = r * cols + c;
            const GridCell& cell = grid[static_cast<size_t>(idx)];

            cv::Vec3b color;
            if (cell.valid && cell.slope < FLT_MAX)
            {
                color = SlopeToColor(cell.slope, display_max);
            }
            else
            {
                color = cv::Vec3b(60, 60, 60);
            }

            // int px_start = c * kGridPixelScale;
            // int py_start = r * kGridPixelScale;
            // cv::Rect roi(px_start, py_start, kGridPixelScale, kGridPixelScale);
            cv::Point pStart = GridToPixel(c, r);
            cv::Rect roi(pStart.x, pStart.y, kGridPixelScale, kGridPixelScale);
            


            roi &= cv::Rect(0, 0, img_w, img_h);
            if (roi.width > 0 && roi.height > 0)
            {
                image(roi) = color;
            }
        }
    }

     AddBackGround(image, img_w, img_h, gridCfg);


    char title[128];
    snprintf(title, sizeof(title), "Slope [0, %.2f] max=%.2f",
             display_max, slope_max);
    cv::putText(image, title, cv::Point(10, 20),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

    ShowOrSave(image, "Slope", viewCfg);
}

// ============================================================================
// 4. DrawReference —— Ground Reference 热力图
// ============================================================================

void DebugViewer::DrawReference(const std::vector<GridCell>& grid,
                                 int rows, int cols,
                                 const ElevationGridConfig& gridCfg)
{
    const auto& viewCfg = m_DV_config.GroundReference;
    
    // std::cout << "[IN DrawReference] enable=" << viewCfg.enable << std::endl;
    if (!viewCfg.enable) return;

    // 统计有效 cell 的 ground_reference_z 范围
    float z_min =  FLT_MAX;
    float z_max = -FLT_MAX;
    for (const auto& cell : grid)
    {
        if (cell.valid)
        {
            z_min = std::min(z_min, cell.ground_reference_z);
            z_max = std::max(z_max, cell.ground_reference_z);
        }
    }

    // int img_w = cols * kGridPixelScale;
    // int img_h = rows * kGridPixelScale;
    // cv::Mat image(img_h, img_w, CV_8UC3, cv::Scalar(40, 40, 40));

    float blind_dist_x = gridCfg.car_half_x + gridCfg.body_filter_x_threshold; 
    int blind_cols = static_cast<int>(std::round(blind_dist_x / gridCfg.grid_resolution));
    int px_blind_offset = blind_cols * kGridPixelScale;

    // 总宽 = 盲区像素 + 有效网格像素 + 右侧边距
    int img_w = px_blind_offset + (cols * kGridPixelScale) + expand_img_w;
    // 总高 = 有效网格像素 + 上下边距 (假设Y轴无盲区，若有需同理增加)
    int img_h = (rows * kGridPixelScale) + expand_img_h;

    cv::Mat image(img_h, img_w, CV_8UC3, cv::Scalar(40, 40, 40)); 


    auto GridToPixel = [&](int c, int r) -> cv::Point {
        int px = px_blind_offset + (c * kGridPixelScale) + (expand_img_w / 2);
        int py = ((rows - 1 - r) * kGridPixelScale) + (expand_img_h / 2);
        return cv::Point(px, py);
    };

    for (int r = 0; r < rows; ++r)
    {
        for (int c = 0; c < cols; ++c)
        {
            int idx = r * cols + c;
            const GridCell& cell = grid[static_cast<size_t>(idx)];

            cv::Vec3b color;
            if (cell.valid)
            {
                color = ZValueToColor(cell.ground_reference_z, z_min, z_max);
            }
            else
            {
                color = cv::Vec3b(60, 60, 60);
            }

            // int px_start = c * kGridPixelScale;
            // int py_start = r * kGridPixelScale;
            // cv::Rect roi(px_start, py_start, kGridPixelScale, kGridPixelScale);
            
            cv::Point pStart = GridToPixel(c, r);
            cv::Rect roi(pStart.x, pStart.y, kGridPixelScale, kGridPixelScale);
            

            roi &= cv::Rect(0, 0, img_w, img_h);
            if (roi.width > 0 && roi.height > 0)
            {
                image(roi) = color;
            }
        }
    }

    AddBackGround(image, img_w, img_h, gridCfg);

    char title[128];
    snprintf(title, sizeof(title), "GroundReference [%.2f, %.2f]m", z_min, z_max);
    cv::putText(image, title, cv::Point(10, 20),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

    ShowOrSave(image, "GroundReference", viewCfg);
}

// ============================================================================
// 5. DrawObstacleCandidate —— 障碍物候选图
// ============================================================================

void DebugViewer::DrawObstacleCandidate(const std::vector<GridCell>& grid,
                                         int rows, int cols,
                                         const ElevationGridConfig& gridCfg)
{
    const auto& viewCfg = m_DV_config.ObstacleCandidate;
    if (!viewCfg.enable) return;

    // int img_w = cols * kGridPixelScale;
    // int img_h = rows * kGridPixelScale;
    // cv::Mat image(img_h, img_w, CV_8UC3, cv::Scalar(40, 40, 40));

    float blind_dist_x = gridCfg.car_half_x + gridCfg.body_filter_x_threshold; 
    int blind_cols = static_cast<int>(std::round(blind_dist_x / gridCfg.grid_resolution));
    int px_blind_offset = blind_cols * kGridPixelScale;

    // 总宽 = 盲区像素 + 有效网格像素 + 右侧边距
    int img_w = px_blind_offset + (cols * kGridPixelScale) + expand_img_w;
    // 总高 = 有效网格像素 + 上下边距 (假设Y轴无盲区，若有需同理增加)
    int img_h = (rows * kGridPixelScale) + expand_img_h;

    cv::Mat image(img_h, img_w, CV_8UC3, cv::Scalar(40, 40, 40)); 

    auto GridToPixel = [&](int c, int r) -> cv::Point {
        int px = px_blind_offset + (c * kGridPixelScale) + (expand_img_w / 2);
        int py = ((rows - 1 - r) * kGridPixelScale) + (expand_img_h / 2);
        return cv::Point(px, py);
    };

    int ground_cnt = 0, obs_cnt = 0, vert_cnt = 0, invalid_cnt = 0;

    for (int r = 0; r < rows; ++r)
    {
        for (int c = 0; c < cols; ++c)
        {
            int idx = r * cols + c;
            const GridCell& cell = grid[static_cast<size_t>(idx)];

            cv::Vec3b color;
            if (!cell.valid)
            {
                color = cv::Vec3b(100, 100, 100);  // 灰色: Invalid
                invalid_cnt++;
            }
            else if (cell.is_vertical_structure)
            {
                color = cv::Vec3b(0, 0, 220);  // 红色: Vertical Structure
                vert_cnt++;
            }
            else if (cell.is_obstacle_candidate)
            {
                color = cv::Vec3b(0, 220, 220);  // 黄色: Obstacle Candidate
                obs_cnt++;
            }
            else if (cell.is_ground)
            {
                color = cv::Vec3b(0, 220, 0);  // 绿色: Ground
                ground_cnt++;
            }
            else
            {
                color = cv::Vec3b(100, 100, 100);  // 灰色: 其他
                invalid_cnt++;
            }

            // int px_start = c * kGridPixelScale;
            // int py_start = r * kGridPixelScale;
            // cv::Rect roi(px_start, py_start, kGridPixelScale, kGridPixelScale);
            
            cv::Point pStart = GridToPixel(c, r);
            cv::Rect roi(pStart.x, pStart.y, kGridPixelScale, kGridPixelScale);
            
            
            roi &= cv::Rect(0, 0, img_w, img_h);
            if (roi.width > 0 && roi.height > 0)
            {
                image(roi) = color;
            }
        }
    }

    AddBackGround(image, img_w, img_h, gridCfg);

    char title[128];
    snprintf(title, sizeof(title), "ObstacleCandidate G:%d Obs:%d Vert:%d Inv:%d",
             ground_cnt, obs_cnt, vert_cnt, invalid_cnt);
    cv::putText(image, title, cv::Point(10, 20),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

    ShowOrSave(image, "ObstacleCandidate", viewCfg);
}

// ============================================================================
// 6. DrawCluster —— 聚类 Label 图
// ============================================================================

void DebugViewer::DrawCluster(const std::vector<GridCell>& grid,
                               int rows, int cols,
                               const ElevationGridConfig& gridCfg)
{
    const auto& viewCfg = m_DV_config.Cluster;
    if (!viewCfg.enable) return;

    // int img_w = cols * kGridPixelScale;
    // int img_h = rows * kGridPixelScale;
    // cv::Mat image(img_h, img_w, CV_8UC3, cv::Scalar(40, 40, 40));

    float blind_dist_x = gridCfg.car_half_x + gridCfg.body_filter_x_threshold; 
    int blind_cols = static_cast<int>(std::round(blind_dist_x / gridCfg.grid_resolution));
    int px_blind_offset = blind_cols * kGridPixelScale;

    // 总宽 = 盲区像素 + 有效网格像素 + 右侧边距
    int img_w = px_blind_offset + (cols * kGridPixelScale) + expand_img_w;
    // 总高 = 有效网格像素 + 上下边距 (假设Y轴无盲区，若有需同理增加)
    int img_h = (rows * kGridPixelScale) + expand_img_h;

    cv::Mat image(img_h, img_w, CV_8UC3, cv::Scalar(40, 40, 40));

    auto GridToPixel = [&](int c, int r) -> cv::Point {
        int px = px_blind_offset + (c * kGridPixelScale) + (expand_img_w / 2);
        int py = ((rows - 1 - r) * kGridPixelScale) + (expand_img_h / 2);
        return cv::Point(px, py);
    };

    
    int cluster_cell_cnt = 0;

    for (int r = 0; r < rows; ++r)
    {
        for (int c = 0; c < cols; ++c)
        {
            int idx = r * cols + c;
            const GridCell& cell = grid[static_cast<size_t>(idx)];

            cv::Vec3b color;
            if (cell.is_obstacle_candidate && cell.obstacle_label >= 0)
            {
                color = LabelToColor(cell.obstacle_label);
                cluster_cell_cnt++;
            }
            else if (cell.valid && cell.is_ground)
            {
                color = cv::Vec3b(0, 80, 0);  // 暗绿: Ground (背景)
            }
            else if (cell.valid)
            {
                color = cv::Vec3b(60, 20, 20);  // 暗红: 其他障碍物 (非候选)
            }
            else
            {
                color = cv::Vec3b(40, 40, 40);  // 深灰: Invalid
            }

            // int px_start = c * kGridPixelScale;
            // int py_start = r * kGridPixelScale;
            // cv::Rect roi(px_start, py_start, kGridPixelScale, kGridPixelScale);

            cv::Point pStart = GridToPixel(c, r);
            cv::Rect roi(pStart.x, pStart.y, kGridPixelScale, kGridPixelScale);
            
            roi &= cv::Rect(0, 0, img_w, img_h);
            if (roi.width > 0 && roi.height > 0)
            {
                image(roi) = color;
            }
        }
    }

    AddBackGround(image, img_w, img_h, gridCfg);

    char title[128];
    snprintf(title, sizeof(title), "Cluster Labels cells:%d", cluster_cell_cnt);
    cv::putText(image, title, cv::Point(10, 20),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

    ShowOrSave(image, "Cluster", viewCfg);
}

// ============================================================================
// 7. DrawBoundingBox —— 包围盒俯视图
// ============================================================================

void DebugViewer::DrawBoundingBox(const std::vector<GridCluster>& clusters,
                                   const ElevationGridConfig& gridCfg)
{
    const auto& viewCfg = m_DV_config.BoundingBox;
    if (!viewCfg.enable) return;

    // 黑色背景
    cv::Mat image(kOverlayHeight, kOverlayWidth, CV_8UC3, cv::Scalar(20, 20, 20));

    // 绘制 ROI 边框
    cv::rectangle(image, cv::Point(0, 0),
                  cv::Point(kOverlayWidth - 1, kOverlayHeight - 1),
                  cv::Scalar(80, 80, 80), 1);

    for (size_t i = 0; i < clusters.size(); ++i)
    {
        const auto& cluster = clusters[i];

        // 使用 OBB 角点（如果有的话），否则使用 AABB
        cv::Scalar color = LabelToColor(static_cast<int>(i));

        if (cluster.has_obb)
        {
            // OBB 4 个角点
            std::vector<cv::Point> pts(4);
            for (int j = 0; j < 4; ++j)
            {
                int px, py;
                WorldToPixel(cluster.obb_corners[j].x, cluster.obb_corners[j].y,
                             px, py, gridCfg);
                pts[j] = cv::Point(px, py);
            }
            // 绘制填充半透明多边形 + 边框
            cv::polylines(image, pts, true, color, 2);

            // 绘制中心点
            int cx, cy;
            WorldToPixel(cluster.obb_center_x, cluster.obb_center_y,
                         cx, cy, gridCfg);
            cv::circle(image, cv::Point(cx, cy), 4, color, -1);

            // 标注 ID
            cv::putText(image, std::to_string(cluster.id),
                        cv::Point(cx + 6, cy - 6),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
        }
        else
        {
            // AABB
            int x1, y1, x2, y2;
            WorldToPixel(cluster.min_x, cluster.min_y, x1, y1, gridCfg);
            WorldToPixel(cluster.max_x, cluster.max_y, x2, y2, gridCfg);
            cv::Rect rect(cv::Point(x1, y2), cv::Point(x2, y1));  // 注意 Y 翻转
            cv::rectangle(image, rect, color, 2);

            int cx, cy;
            WorldToPixel(cluster.center_x, cluster.center_y, cx, cy, gridCfg);
            cv::circle(image, cv::Point(cx, cy), 4, color, -1);
            cv::putText(image, std::to_string(cluster.id),
                        cv::Point(cx + 6, cy - 6),
                        cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255), 1);
        }
    }

    char title[128];
    snprintf(title, sizeof(title), "BoundingBox clusters:%zu", clusters.size());
    cv::putText(image, title, cv::Point(10, 20),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

    ShowOrSave(image, "BoundingBox", viewCfg);
}

// ============================================================================
// 8. DrawOverlay —— 最终点云叠加图
// ============================================================================

void DebugViewer::DrawOverlay(const pcl::PointCloud<pcl::PointXYZI>& groundCloud,
                               const pcl::PointCloud<pcl::PointXYZI>& obstacleCloud,
                               const std::vector<GridCluster>& clusters,
                               const ElevationGridConfig& gridCfg)
{
    const auto& viewCfg = m_DV_config.Overlay;
    if (!viewCfg.enable) return;

    // 黑色背景
    cv::Mat image(kOverlayHeight, kOverlayWidth, CV_8UC3, cv::Scalar(20, 20, 20));

    // ROI 边框
    cv::rectangle(image, cv::Point(0, 0),
                  cv::Point(kOverlayWidth - 1, kOverlayHeight - 1),
                  cv::Scalar(80, 80, 80), 1);

    // 绘制地面点 (绿色)
    for (const auto& pt : groundCloud.points)
    {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y)) continue;
        int px, py;
        WorldToPixel(pt.x, pt.y, px, py, gridCfg);
        if (px >= 0 && px < kOverlayWidth && py >= 0 && py < kOverlayHeight)
        {
            image.at<cv::Vec3b>(py, px) = cv::Vec3b(0, 180, 0);  // 绿色
        }
    }

    // 绘制障碍物点 (红色)
    for (const auto& pt : obstacleCloud.points)
    {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y)) continue;
        int px, py;
        WorldToPixel(pt.x, pt.y, px, py, gridCfg);
        if (px >= 0 && px < kOverlayWidth && py >= 0 && py < kOverlayHeight)
        {
            image.at<cv::Vec3b>(py, px) = cv::Vec3b(0, 0, 220);  // 红色
        }
    }

    // 绘制 cluster 包围盒 (白色)
    for (size_t i = 0; i < clusters.size(); ++i)
    {
        const auto& cluster = clusters[i];

        if (cluster.has_obb)
        {
            std::vector<cv::Point> pts(4);
            for (int j = 0; j < 4; ++j)
            {
                int px, py;
                WorldToPixel(cluster.obb_corners[j].x, cluster.obb_corners[j].y,
                             px, py, gridCfg);
                pts[j] = cv::Point(px, py);
            }
            cv::polylines(image, pts, true, cv::Scalar(255, 255, 255), 1);
        }
        else
        {
            int x1, y1, x2, y2;
            WorldToPixel(cluster.min_x, cluster.min_y, x1, y1, gridCfg);
            WorldToPixel(cluster.max_x, cluster.max_y, x2, y2, gridCfg);
            cv::Rect rect(cv::Point(x1, y2), cv::Point(x2, y1));
            cv::rectangle(image, rect, cv::Scalar(255, 255, 255), 1);
        }

        // 中心点 (黄色)
        int cx, cy;
        WorldToPixel(cluster.center_x, cluster.center_y, cx, cy, gridCfg);
        cv::circle(image, cv::Point(cx, cy), 3, cv::Scalar(0, 220, 220), -1);
    }

    char title[128];
    snprintf(title, sizeof(title), "Overlay G:%zu Obs:%zu Clusters:%zu",
             groundCloud.size(), obstacleCloud.size(), clusters.size());
    cv::putText(image, title, cv::Point(10, 20),
                cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

    ShowOrSave(image, "Overlay", viewCfg);
}

}  // namespace Lidar_Low_Detection
