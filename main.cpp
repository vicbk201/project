#include <iostream>
#include <pcl/io/pcd_io.h>
#include "ArgumentParser.h"
#include "PCDLoader.h"
#include "OctreeProcessor.h"
#include "VoxelGridProcessor.h"
#include "GroundRemovalProcessor.h"
#include "ConditionalClusteringProcessor.h"
#include "PointCloudViewer.h"
#include <iomanip>

int main(int argc, char** argv)
{
    std::string cloudFile;
    float resolution;
    std::string method;
    std::string outputCloudFile;

    // 解析命令列參數
    if (!ArgumentParser::parseArguments(argc, argv, cloudFile, resolution, method, outputCloudFile)) {
        std::cerr << "正確使用方式: " << argv[0]
                  << " --cloudfile <PCD檔案路徑> --method <octree|voxelgrid> [--resolution <解析度>] [--o|--output <完整輸出PCD檔案路徑>]"
                  << std::endl;
        return -1;
    }

    std::cout << "PCD File: " << cloudFile << std::endl;
    std::cout << "Method: " << method << std::endl;
    std::cout << "Resolution: " << resolution << std::endl;

    // 載入點雲
    auto cloud = PCDLoader::loadPCD(cloudFile);
    if (!cloud || cloud->empty()) {
        std::cerr << "Error: 無法載入點雲或點雲為空。" << std::endl;
        return -1;
    }
    std::cout << "原始點雲數量: " << cloud->size() << std::endl;

    // 選擇下採樣方法
    pcl::PointCloud<pcl::PointXYZI>::Ptr downsampledCloud;
    if (method == "voxelgrid") {
        auto voxelResult = VoxelGridProcessor::processVoxelGrid(cloud, resolution);
        downsampledCloud = voxelResult.cloud;
        std::cout << "降採樣後點雲數量: " << downsampledCloud->size()
                  << " (算法耗時: " << std::fixed << std::setprecision(2) << voxelResult.runtime_ms << " ms)" << std::endl;
    } else if (method == "octree") {
        auto octreeResult = OctreeProcessor::processOctree(cloud, resolution);
        downsampledCloud = octreeResult.cloud;
        std::cout << "Octree下採樣後點雲數量: " << downsampledCloud->size()
                  << " (算法耗時: " << std::fixed << std::setprecision(2) << octreeResult.runtime_ms << " ms)" << std::endl;
    } else {
        std::cerr << "Error: 未知的 method: " << method << std::endl;
        return -1;
    }

    // RANSAC 去除地面
    auto groundResult = GroundRemovalProcessor::removeGround(downsampledCloud);
    auto cloudNoGround = groundResult.cloud;
    std::cout << "去除地面後點雲數量: " << cloudNoGround->size()
              << " (算法耗時: " << std::fixed << std::setprecision(2) << groundResult.runtime_ms << " ms)" << std::endl;

    // 進行 Conditional Clustering
    ClusteringResult clusteringResult = ConditionalClusteringProcessor::clusterCloud(cloudNoGround);
    std::cout << "聚類後點雲數量: " << clusteringResult.cloud->size()
              << " (算法耗時: " << std::fixed << std::setprecision(2) << clusteringResult.runtime_ms << " ms)" << std::endl;



    // 儲存 PCD，確保是 `XYZI` 格式，並且 `intensity` 沒有變動
    if (!outputCloudFile.empty()) {
        std::cout << "Saving processed cloud to: " << outputCloudFile << std::endl;
        if (pcl::io::savePCDFile(outputCloudFile, *(clusteringResult.cloud)) == -1) {
            std::cerr << "Error: 儲存點雲失敗！" << std::endl;
            return -1;
        }
    }

    // 顯示點雲
    try {
        PointCloudViewer::displayProcessedCloud(clusteringResult.cloud,resolution);
    } catch(const std::exception &e) {
        std::cerr << "顯示點雲時發生錯誤: " << e.what() << std::endl;
    }

    return 0;
}
