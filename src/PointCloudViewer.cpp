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
    
     // 加入每個 OBB 的視覺化
    for (size_t i = 0; i < obb_list.size(); i++) {
        const auto &obb = obb_list[i];
        std::stringstream ss;
        ss << "bbox_" << i;
        // 在視窗中新增包圍盒
        viewer->addCube(obb.center, obb.orientation, 
                        obb.dimensions.x(), obb.dimensions.y(), obb.dimensions.z(), ss.str());
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                              pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, ss.str());
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, ss.str());
    }


    // 顯示畫面
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    viewer->resetCamera();

    while (!viewer->wasStopped()) {
        viewer->spin();
    }
}
