#include "PointCloudViewer.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <vtkVectorText.h>
#include <vtkPolyDataMapper.h>
#include <vtkFollower.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <iostream>
#include <sstream>
#include <cmath>
#include <tuple>
#include <vector>

pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertLabelToRGB(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    coloredCloud->points.resize(cloud->points.size());

    // 定義20種固定顏色 (RGB)
    std::vector<std::tuple<int, int, int>> colors = {
        {255,   0,   0},   // 紅
        {  0, 255,   0},   // 綠
        {  0,   0, 255},   // 藍
        {255, 255,   0},   // 黃
        {  0, 255, 255},   // 青
        {255,   0, 255},   // 品紅
        {128,   0,   0},   // 深紅
        {  0, 128,   0},   // 深綠
        {  0,   0, 128},   // 深藍
        {128, 128,   0},   // 橄欖
        {  0, 128, 128},   // 螢光藍
        {128,   0, 128},   // 紫
        {192, 192, 192},   // 銀
        {128, 128, 128},   // 灰
        { 64,  64,  64},   // 深灰
        {255, 128,   0},   // 橙
        {255,   0, 128},   // 粉紅橙
        {  0, 255, 128},   // 薄荷綠
        {128, 255,   0},   // 淡綠
        {  0, 128, 255}    // 淡藍
    };

    for (size_t i = 0; i < cloud->points.size(); i++) {
        const auto &pt = cloud->points[i];
        pcl::PointXYZRGB pt_rgb;
        pt_rgb.x = pt.x;
        pt_rgb.y = pt.y;
        pt_rgb.z = pt.z;
        // 使用 label % 20 來生成20種離散顏色
        int label = static_cast<int>(pt.intensity);
        int mod = std::abs(label) % 20;
        int r, g, b;
        std::tie(r, g, b) = colors[mod];
        pt_rgb.r = r;
        pt_rgb.g = g;
        pt_rgb.b = b;
        coloredCloud->points[i] = pt_rgb;
    }
    coloredCloud->width  = static_cast<uint32_t>(coloredCloud->points.size());
    coloredCloud->height = 1;
    coloredCloud->is_dense = cloud->is_dense;
    return coloredCloud;
}


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
    pcl::visualization::PCLVisualizer::Ptr viewer(
        new pcl::visualization::PCLVisualizer("Processed Point Cloud"));
    viewer->setBackgroundColor(0, 0, 0);

    /*
    // 使用 intensity 欄位作為顏色處理器
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_handler(processedCloud, "intensity");
    viewer->addPointCloud<pcl::PointXYZI>(processedCloud, intensity_handler, "processed_cloud");
    */

    // === 修改部分：用自訂轉換函式將點雲的 intensity (聚類標籤) 映射到固定色彩
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr coloredCloud = convertLabelToRGB(processedCloud);
    // 使用 RGB 欄位作為顏色處理器
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_handler(coloredCloud);
    viewer->addPointCloud<pcl::PointXYZRGB>(coloredCloud, rgb_handler, "processed_cloud");
    // === end 修改
    

    //加入每個 OBB 的視覺化
    for (size_t i = 0; i < obb_list.size(); i++) {
        const auto &obb = obb_list[i];
        std::stringstream cubeId;
        cubeId << "bbox_" << i;
        // 新增包圍盒
        viewer->addCube(obb.center, obb.orientation, 
                        obb.dimensions.x(), obb.dimensions.y(), obb.dimensions.z(), cubeId.str());
        viewer->setShapeRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
            pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cubeId.str());
        viewer->setShapeRenderingProperties(
            pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, cubeId.str());
            
        
        // 建立文字內容：尺寸資訊
        char text[100];
        float length = std::fabs(obb.dimensions.x());
        float width  = std::fabs(obb.dimensions.y());
        float height = std::fabs(obb.dimensions.z());
        sprintf(text, "L:%.2f, W:%.2f, H:%.2f", length, width, height);

        // 利用 vtkVectorText 產生 3D 文字幾何資料
        vtkSmartPointer<vtkVectorText> vectorText = vtkSmartPointer<vtkVectorText>::New();
        vectorText->SetText(text);

        // 建立 mapper
        vtkSmartPointer<vtkPolyDataMapper> textMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        textMapper->SetInputConnection(vectorText->GetOutputPort());

        // 建立 vtkFollower，作為 3D 文字的 actor
        vtkSmartPointer<vtkFollower> textActor = vtkSmartPointer<vtkFollower>::New();
        textActor->SetMapper(textMapper);
        // 設定正確的位置（記得呼叫 x()、y()、z()）
        textActor->SetPosition(static_cast<double>(obb.center.x()),
                               static_cast<double>(obb.center.y()),
                               static_cast<double>(obb.center.z()));
        // 調整縮放因子，可依需求修改
        textActor->SetScale(0.5);

        // 從 viewer 取得 Renderer，並加入文字 actor
        viewer->getRendererCollection()->InitTraversal();
        vtkRenderer* renderer = viewer->getRendererCollection()->GetFirstRenderer();
        if (renderer)
        {
            // 傳入原始指標
            renderer->AddActor(textActor.Get());
            // 設定 follower 的攝影機（以確保文字面向攝影機）
            textActor->SetCamera(renderer->GetActiveCamera());
        }
        
    }

    // 顯示座標系統並初始化攝影機參數
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    viewer->resetCamera();

    while (!viewer->wasStopped()) {
        viewer->spin();
    }
}
