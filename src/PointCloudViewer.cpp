
#include "PointCloudViewer.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <sstream>


void PointCloudViewer::displayProcessedCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr processedCloud, float resolution)
{
    if (!processedCloud || processedCloud->empty()) {
        std::cerr << "Error: Processed cloud is empty." << std::endl;
        return;
    }

     // 禁用 VTK 警告
    vtkObject::GlobalWarningDisplayOff();

    // 將 processedCloud 轉換為帶顏色的點雲 (固定綠色)
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudRGB(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (const auto &pt : processedCloud->points) {
        pcl::PointXYZRGB pt_rgb;
        pt_rgb.x = pt.x;
        pt_rgb.y = pt.y;
        pt_rgb.z = pt.z;
        pt_rgb.r = 0;
        pt_rgb.g = 255;
        pt_rgb.b = 0;
        cloudRGB->push_back(pt_rgb);
    }

    // 建立 PCL 可視化器
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Processed Point Cloud"));
    viewer->setBackgroundColor(0, 0, 0);

    // 使用 RGB 處理器顯示點雲
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbHandler(cloudRGB);
    viewer->addPointCloud<pcl::PointXYZRGB>(cloudRGB, rgbHandler, "processed_cloud");

    // 添加坐標系統
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    viewer->resetCamera();

    // 在視窗上顯示當前解析度與處理後點數的資訊
    std::stringstream ss;
    ss << "Leaf Size (r): " << resolution << " | Processed Cloud Size: " << processedCloud->size();
    viewer->addText(ss.str(), 10, 10, "info_text", 0);

    viewer->spin();

}
