#include "PointCloudViewer.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <iostream>
#include <sstream>

void PointCloudViewer::displayProcessedCloud(
    pcl::PointCloud<pcl::PointXYZI>::Ptr processedCloud, 
    float resolution,
    const std::vector<OrientedBoundingBox>& obb_list)
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
    viewer->addPointCloud<pcl::PointXYZI>(processedCloud, intensity_handler, "processed_cloud");

    // 顯示畫面
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    viewer->resetCamera();

    while (!viewer->wasStopped()) {
        viewer->spin();
    }
}
