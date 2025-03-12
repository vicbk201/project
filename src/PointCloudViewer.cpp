#include "PointCloudViewer.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <iostream>
#include <sstream>

void PointCloudViewer::displayProcessedCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr processedCloud, float resolution)
{
    if (!processedCloud || processedCloud->empty()) {
        std::cerr << "Error: Processed cloud is empty." << std::endl;
        return;
    }

    // 禁用 VTK 警告
    vtkObject::GlobalWarningDisplayOff();

    // 建立 PCL 可視化器
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Processed Point Cloud"));
    viewer->setBackgroundColor(0, 0, 0);

    // 使用 intensity 欄位作為顏色處理器
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_handler(processedCloud, "intensity");
    if (!intensity_handler.isCapable()) {
        std::cerr << "Intensity field not available or not capable!" << std::endl;
        return;
    }

    // 直接將原始的 XYZI 點雲加入檢視器
    viewer->addPointCloud<pcl::PointXYZI>(processedCloud, intensity_handler, "processed_cloud");

    // 添加坐標系統並初始化相機參數
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    viewer->resetCamera();

    // 在視窗中顯示解析度與點雲數量資訊
    std::stringstream ss;
    ss << "Leaf Size (r): " << resolution << " | Processed Cloud Size: " << processedCloud->size();
    viewer->addText(ss.str(), 10, 10, "info_text", 0);

    while (!viewer->wasStopped()) {
        viewer->spin();
    }
}
