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
#include "OBBFittingProcessor.h"
#include <map>
#include <vector>
#include <pcl/console/print.h>

int main(int argc, char **argv)
{
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

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
    }    
    else if (clusterMethod == "dbscan")
    {
        auto clusteringResult = DBSCANClusteringProcessor::clusterCloud(groundRemovedCloud, 0.7,7); // eps, minPts
        std::cout << "DBSCAN Cluster 後點雲數量: " << clusteringResult.cloud->size()
                  << " (算法耗時: " << std::fixed << std::setprecision(2) << clusteringResult.runtime_ms << " ms)" << std::endl;
        clusteredCloud = clusteringResult.cloud;
    }
    else
    {
        std::cerr << "Error: 未知的 clustering method: " << clusterMethod << std::endl;
        return -1;
    }
    
    // 假設 clusteredCloud 為聚類後的點雲（來自 Euclidean 或 DBSCAN 處理器）
    std::map<int, pcl::PointCloud<pcl::PointXYZI>::Ptr> clusterMap;
    for (const auto &pt : clusteredCloud->points)
    {
        int label = static_cast<int>(pt.intensity);
        if (label == -2)  // 噪點略過
        continue;
    if (clusterMap.find(label) == clusterMap.end())
    {
        clusterMap[label] = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    }
    clusterMap[label]->points.push_back(pt);
    }

    std::cout << "Total clusters: " << clusterMap.size() << std::endl;
    
    // 計算每個聚類的 OBB 並根據尺寸過濾只保留人和車
    std::vector<OrientedBoundingBox> validOBB;
    int validCount = 0;
    for (const auto &cluster : clusterMap)
    {
        int clusterLabel = cluster.first;
        OrientedBoundingBox obb = OBBFittingProcessor::computeOBB(cluster.second);   
        validOBB.push_back(obb);
    }

    /*   
        // 定義尺寸：
        float length = obb.dimensions.x(); // X 軸範圍
        float width  = obb.dimensions.y(); // Y 軸範圍
        float height = obb.dimensions.z(); // Z 軸範圍

        std::cout << "Cluster " << clusterLabel << " dimensions (length, width, height): " 
                  << length << ", " << width << ", " << height << std::endl;

        //篩選條件：
        bool isHuman = (height >= 0.8f && height <= 2.2f && length <= 1.2f && width <= 1.2f);
        bool isCar = (height >= 1.2f && height <= 2.2f && length >= 3.0f && length <= 7.0f && width  >= 1.2f && width  <= 3.0f);

        if (isHuman || isCar)
        {
        validOBB.push_back(obb);
        validCount++;
        }
    }
    std::cout << "Valid clusters (people or vehicles): " << validCount << std::endl;
    */

    clusteredCloud->width = static_cast<uint32_t>(clusteredCloud->points.size());
    clusteredCloud->height = 1;
    clusteredCloud->is_dense = true;

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

    // 呼叫視覺化函式，同時顯示聚類點雲與計算好的 OBB
    try {
        PointCloudViewer::displayProcessedCloud(clusteredCloud, resolution,validOBB);
    }
    catch (const std::exception &e)
    {
        std::cerr << "顯示點雲時發生錯誤: " << e.what() << std::endl;
    }

    return 0;
}