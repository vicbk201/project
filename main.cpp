
#include <iostream>
#include "ArgumentParser.h"
#include "PCDLoader.h"
#include "OctreeProcessor.h"
#include "VoxelGridProcessor.h"
#include "PointCloudViewer.h"

int main(int argc, char** argv)
{
    std::string cloudFile;
    float resolution;
    std::string method;

    // 解析命令列參數，若解析失敗則退出
    if (!ArgumentParser::parseArguments(argc, argv, cloudFile, resolution, method)) {
        std::cerr << "正確使用方式: " << argv[0] << " --cloudfile <PCD檔案路徑> --method <octree|voxelgrid> [--resolution <解析度>]" << std::endl;
        return -1;
    }

    std::cout << "PCD File: " << cloudFile << std::endl;
    std::cout << "Method: " << method << std::endl;
    std::cout << "Resolution: " << resolution << std::endl;

    auto cloud = PCDLoader::loadPCD(cloudFile);
    if (!cloud || cloud->empty()) {
        std::cerr << "Error: 無法載入點雲或點雲為空。" << std::endl;
        return -1;
    }
    std::cout << "原始點雲數量: " << cloud->size() << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr processedCloud;
    if (method == "voxelgrid") {
        processedCloud = VoxelGridProcessor::processVoxelGrid(cloud, resolution);
    } else {
        processedCloud = OctreeProcessor::processOctree(cloud, resolution);
    }
    std::cout << "降採樣後點雲數量: " << processedCloud->size() << std::endl;
    
    try {
        PointCloudViewer::displayProcessedCloud(processedCloud, resolution);
    } catch(const std::exception &e) {
        std::cerr << "顯示點雲時發生錯誤: " << e.what() << std::endl;
    }

    return 0;
}
