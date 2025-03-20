#include <iostream>
#include <pcl/io/pcd_io.h>
#include "ArgumentParser.h"
#include "PCDLoader.h"
#include "OctreeProcessor.h"
#include "VoxelGridProcessor.h"
#include "GroundRemovalProcessor.h"
#include "EuclideanClusterProcessor.h"
#include "DBSCANClusteringProcessor.h"
#include "OBBFittingProcessor.h"
#include "PointCloudViewer.h"
#include <iomanip>

int main(int argc, char **argv)
{
    std::string cloudFile;
    float resolution;
    std::string downsampleMethod;
    std::string clusterMethod;
    std::string outputCloudFile;

    // 解析命令列參數
    if (!ArgumentParser::parseArguments(argc, argv, cloudFile, resolution, downsampleMethod, clusterMethod, outputCloudFile))
    {
        std::cerr << "正確使用方式: " << argv[0]
                  << " --cloudfile <PCD檔案路徑> --method <octree|voxelgrid> --cluster <euclidean|dbscan> [--resolution <解析度>] [--o|--output <完整輸出PCD檔案路徑>]"
                  << std::endl;
        return -1;
    }

    std::cout << "PCD File: " << cloudFile << std::endl;
    std::cout << "Downsample Method: " << downsampleMethod << std::endl;
    std::cout << "Clustering Method: " << clusterMethod << std::endl;
    std::cout << "Resolution: " << resolution << std::endl;

    // 載入點雲
    auto cloud = PCDLoader::loadPCD(cloudFile);
    if (!cloud || cloud->empty())
    {
        std::cerr << "Error: 無法載入點雲或點雲為空。" << std::endl;
        return -1;
    }
    std::cout << "原始點雲數量: " << cloud->size() << std::endl;

    // 選擇下採樣方法
    pcl::PointCloud<pcl::PointXYZI>::Ptr downsampledCloud;
    if (downsampleMethod == "voxelgrid")
    {
        auto voxelResult = VoxelGridProcessor::processVoxelGrid(cloud, resolution);
        downsampledCloud = voxelResult.cloud;
        std::cout << "降採樣後點雲數量: " << downsampledCloud->size()
                  << " (算法耗時: " << std::fixed << std::setprecision(2) << voxelResult.runtime_ms << " ms)" << std::endl;
    }
    else if (downsampleMethod == "octree")
    {
        auto octreeResult = OctreeProcessor::processOctree(cloud, resolution);
        downsampledCloud = octreeResult.cloud;
        std::cout << "Octree下採樣後點雲數量: " << downsampledCloud->size()
                  << " (算法耗時: " << std::fixed << std::setprecision(2) << octreeResult.runtime_ms << " ms)" << std::endl;
    }
    else
    {
        std::cerr << "Error: 未知的 downsample method: " << downsampleMethod << std::endl;
        return -1;
    }

    // RANSAC 去除地面
    auto groundResult = GroundRemovalProcessor::removeGround(downsampledCloud);
    auto groundRemovedCloud = groundResult.cloud;
    std::cout << "去除地面後點雲數量: " << groundRemovedCloud->size()
              << " (算法耗時: " << std::fixed << std::setprecision(2) << groundResult.runtime_ms << " ms)" << std::endl;

    // 進行聚類處理 (根據 --cluster 參數)
    pcl::PointCloud<pcl::PointXYZI>::Ptr clusteredCloud;
    if (clusterMethod == "euclidean")
    {
        auto clusteringResult = EuclideanClusterProcessor::clusterCloud(groundRemovedCloud, 0.5); // clusterTolerance
        std::cout << "Euclidean Cluster 後點雲數量: " << clusteringResult.cloud->size()
                  << " (算法耗時: " << std::fixed << std::setprecision(2) << clusteringResult.runtime_ms << " ms)" << std::endl;
        clusteredCloud = clusteringResult.cloud;
    else if (clusterMethod == "dbscan")
    {
        auto clusteringResult = DBSCANClusteringProcessor::clusterCloud(groundRemovedCloud, 1, 20); // eps, minPts
        std::cout << "DBSCAN Cluster 後點雲數量: " << clusteringResult.cloud->size()
                  << " (算法耗時: " << std::fixed << std::setprecision(2) << clusteringResult.runtime_ms << " ms)" << std::endl;
        clusteredCloud = clusteringResult.cloud;
    }
    else
    {
        std::cerr << "Error: 未知的 clustering method: " << clusterMethod << std::endl;
        return -1;
    }

    // 儲存處理後的點雲
    if (!outputCloudFile.empty())
    {
        std::cout << "Saving processed cloud to: " << outputCloudFile << std::endl;
        if (pcl::io::savePCDFile(outputCloudFile, *clusteredCloud) == -1)
        {
            std::cerr << "Error: 儲存點雲失敗！" << std::endl;
            return -1;
        }
    }

    // 呼叫的 displayProcessedCloud 函式
    try
    {
        PointCloudViewer::displayProcessedCloud(clusteredCloud, resolution,);
    }
    catch (const std::exception &e)
    {
        std::cerr << "顯示點雲時發生錯誤: " << e.what() << std::endl;
    }

    return 0;
}
