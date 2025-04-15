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
#include <map>
#include <vector>
#include <pcl/console/print.h>
#include "BackgroundRemovalProcessor.h"
#include "OutlierRemovalProcessor.h"
#include "MeanShiftProcessor.h"
#include "FeatureExtractionProcessor.h"



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
    
    /*
    // 載入點雲
    auto cloud = PCDLoader::loadPCD(cloudFile);
    if (!cloud || cloud->empty())
    {
        std::cerr << "Error: 無法載入點雲或點雲為空。" << std::endl;
        return -1;
    }
    std::cout << "原始點雲數量: " << cloud->size() << std::endl;
    */
   
    
    //載入資料
    auto raw_input = PCDLoader::loadPCD(cloudFile);
    auto raw_background = PCDLoader::loadPCD("/home/semilux/Documents/fortsense_test_pcd/background/4084-563116000.pcd");
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    float background_resolution = 0.15f;
    
    if (!raw_input || raw_input->empty())
    {
        std::cerr << "Error: 輸入點雲為空。" << std::endl;
        return -1;
    }
    if (!raw_background || raw_background->empty())
    {
        std::cerr << "警告：背景點雲為空，跳過背景去除。" << std::endl;
        cloud = raw_input;
    }
    else
    {
        cloud = BackgroundRemovalProcessor::removeBackgroundByOctree(raw_input, raw_background, background_resolution);
    }

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

    // 使用固定平面係數進行地面分類（從空場景取得）
    pcl::ModelCoefficients::Ptr fixed_plane(new pcl::ModelCoefficients());
    fixed_plane->values = {-0.109833f, -0.0308086f, 0.993473f, 2.37525f};

    // auto groundResult = GroundRemovalProcessor::removeGround(downsampledCloud) ;
    auto groundResult = GroundRemovalProcessor::removeGroundWithPlane(downsampledCloud, fixed_plane, 0.07f);  // 第三個參數是距離閾值 threshold
    auto groundRemovedCloud = groundResult.cloud;
    std::cout << "地面去除後點雲數量: " << groundRemovedCloud->size()
              << " (算法耗時: " << std::fixed << std::setprecision(2) << groundResult.runtime_ms << " ms)" << std::endl;
    /*          
    std::cout << "RANSAC 擬合地面平面係數: ["
          << groundResult.ground_coefficients[0] << ", "
          << groundResult.ground_coefficients[1] << ", "
          << groundResult.ground_coefficients[2] << ", "
          << groundResult.ground_coefficients[3] << "]" << std::endl;
    */
    
    auto filteredCloud = OutlierRemovalProcessor::removeOutliers(groundRemovedCloud, 0.35f, 3);
    groundRemovedCloud = filteredCloud;

    
    // 進行聚類處理
    pcl::PointCloud<pcl::PointXYZI>::Ptr clusteredCloud;
    if (clusterMethod == "euclidean")
    {
        auto clusteringResult = EuclideanClusterProcessor::clusterCloud(groundRemovedCloud, 100);
        std::cout << "Euclidean Cluster 後點雲數量: " << clusteringResult.cloud->size()
                  << " (算法耗時: " << std::fixed << std::setprecision(2) << clusteringResult.runtime_ms << " ms)" << std::endl;
        clusteredCloud = clusteringResult.cloud;
    }
    else if (clusterMethod == "dbscan")
    {
        auto clusteringResult = DBSCANClusteringProcessor::clusterCloud(groundRemovedCloud, 0.5,20);
        std::cout << "DBSCAN Cluster 後點雲數量: " << clusteringResult.cloud->size()
                  << " (算法耗時: " << std::fixed << std::setprecision(2) << clusteringResult.runtime_ms << " ms)" << std::endl;
        clusteredCloud = clusteringResult.cloud;
    }
    else
    {
        std::cerr << "Error: 未知的 clustering method: " << clusterMethod << std::endl;
        return -1;
    }

    
    // 將分群後的點雲依照 intensity (label) 打包到 map
    std::map<int, pcl::PointCloud<pcl::PointXYZI>::Ptr> clusterMap;
    for (const auto &pt : clusteredCloud->points)
    {
        int label = static_cast<int>(pt.intensity);
        if (label == -2) continue;
        if (clusterMap.find(label) == clusterMap.end())
        {
            clusterMap[label] = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
        }
        clusterMap[label]->points.push_back(pt);
    }

    std::cout << "原始clusters: " << clusterMap.size() << std::endl;
    
    /*
    // 先複製原 clusterMap 的所有 key（避免直接在迭代中修改 map）
    std::vector<int> clusterKeys;
    for (const auto &kv : clusterMap)
        clusterKeys.push_back(kv.first);

    // 使用 newClusterMap 儲存最終結果（拆分後的 cluster）
    std::map<int, pcl::PointCloud<pcl::PointXYZI>::Ptr> newClusterMap = clusterMap;

    // 用來保存有效 OBB（顯示用）
    std::vector<OrientedBoundingBox> validOBB;
    int newBaseLabel = 1000;

    
    // 針對每個 cluster 計算 OBB
    for (auto clusterLabel : clusterKeys)
    {
        auto cloudSeg = clusterMap[clusterLabel];
        if (!cloudSeg || cloudSeg->empty()) continue;

        // 計算大群的 OBB 並取得尺寸
        OrientedBoundingBox obb = OBBFittingProcessor::computeOBB(cloudSeg, &groundResult.ground_coefficients);
        float length = std::fabs(obb.dimensions.x());
        float width  = std::fabs(obb.dimensions.y());
        float height = std::fabs(obb.dimensions.z());
    
        // 定義條件：只顯示符合單一行人或多人候選的聚類
        bool singleCandidate = (length < 1.2f && width < 1.2f && height >= 0.8f && height <= 2.0f);
        bool multiCandidate  = (!singleCandidate) &&
                               (((length > 1.2f && length < 3.0f) || (width > 1.2f && width < 3.0f)) &&
                                (height >= 0.8f && height <= 2.0f));
    
        if(singleCandidate)
        {
            // 單一行人直接加入 OBB 顯示
            for (auto &pt : cloudSeg->points)
                pt.intensity = float(clusterLabel);
            newClusterMap[clusterLabel] = cloudSeg;
            validOBB.push_back(obb);
        }
        else if(multiCandidate && obb.localCloud && !obb.localCloud->empty())
        {
            // 多人候選的群組，使用 MeanShift 進行拆分，拆分後每個子群計算新的 OBB 並加入有效 OBB 列表
            float bandwidth = 0.2f; // 可根據實際情況調整
            int minPoints = 20;
            std::vector<int> subLabels = MeanShiftProcessor::separateClosePedestriansByLocalCloud(
                                             obb, bandwidth, minPoints, newBaseLabel);

            // 統計拆分結果（可用於 debug）
            std::map<int,int> labelCount;
            for (auto lbl : subLabels)
                labelCount[lbl]++;

            if(labelCount.size() <= 1)
            {
                // MeanShift 沒有有效拆分出多個群組時，保留原群但不加入 OBB
                for (auto &pt : cloudSeg->points)
                    pt.intensity = float(clusterLabel);
                newClusterMap[clusterLabel] = cloudSeg;
                // 可選：如果你不想顯示不合格的群組，不 push_back OBB
            }
            else if(subLabels.size() == cloudSeg->size())
            {
                // 拆分出多個子群時，對每個子群重新計算 OBB 並加入顯示
                std::map<int, pcl::PointCloud<pcl::PointXYZI>::Ptr> subClusterMap;
                for (size_t i = 0; i < cloudSeg->size(); i++)
                {
                    int subLabel = subLabels[i];
                    if (subLabel == -2) continue; // 忽略噪點
                    if (subClusterMap.find(subLabel) == subClusterMap.end())
                        subClusterMap[subLabel] = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
                    subClusterMap[subLabel]->points.push_back(cloudSeg->points[i]);
                }

                for (auto &kv_sub : subClusterMap)
                {
                    int newLabel = kv_sub.first;
                    pcl::PointCloud<pcl::PointXYZI>::Ptr subCloud = kv_sub.second;
                    OrientedBoundingBox subOBB = OBBFittingProcessor::computeOBB(subCloud, &groundResult.ground_coefficients);
                    validOBB.push_back(subOBB);
                    for (auto &pt : subCloud->points)
                        pt.intensity = float(newLabel);
                    newClusterMap[newLabel] = subCloud;
                }
                newClusterMap.erase(clusterLabel);
                // 避免 label 重複，更新 newBaseLabel（可依實際需求調整）
                int maxSub = -1;
                for (auto &p : subClusterMap)
                    if(p.first > maxSub)
                         maxSub = p.first;
                if(maxSub >= 0)
                     newBaseLabel = maxSub + 10;
            }
            else
            {
                std::cerr << "[Warning] MeanShift output size 與 cloudSeg size 不匹配\n";
            }
        }
        else
        {
            // 若該聚類不符合任何條件，就不添加 OBB
            for (auto &pt : cloudSeg->points)
                pt.intensity = float(clusterLabel);
            newClusterMap[clusterLabel] = cloudSeg;
            // 不 push_back 有效 OBB
        }
    }

    // 組合 newClusterMap 成 finalCloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr finalCloud(new pcl::PointCloud<pcl::PointXYZI>);
    // 在這裡印出拆分後 cluster 的數量
    std::cout << "拆分cluster: " << newClusterMap.size() << std::endl;

    // 組合只有 validOBB 中的 cluster（表示符合條件的 OBB 才會輸出）
    std::set<int> validLabels;
    for (const auto& obb : validOBB)
    { 
        // OBBFittingProcessor 裡應該要有 label，你可以在 OrientedBoundingBox 裡加入 int label;
        // 如果沒有，可以考慮在 push_back 時搭配 label 一起儲存
        // 這裡暫時改成透過 intensity推估
        if (obb.localCloud && !obb.localCloud->empty())
        {
            int label = static_cast<int>(obb.localCloud->points[0].intensity);
            validLabels.insert(label);
        }
    }

    for (auto &kv : newClusterMap)
    {
        int label = kv.first;
        if (validLabels.find(label) == validLabels.end())
            continue; // 跳過沒有 OBB 的群組

        auto &segCloud = kv.second;
        for (auto &pt : segCloud->points)
            finalCloud->points.push_back(pt);
    }
    /*
    for (auto &kv : newClusterMap)
    {
        auto &segCloud = kv.second;
        for (auto &pt : segCloud->points)
                finalCloud->points.push_back(pt);
    }
    finalCloud->width  = finalCloud->points.size();
    finalCloud->height = 1;
    finalCloud->is_dense = true;
    */

     // 若指定要輸出檔
    if (!outputCloudFile.empty())
    {        
        std::cout << "Output file specified: " << outputCloudFile << std::endl;
        std::cout << "Saving processed cloud to: " << outputCloudFile << std::endl;
        if (pcl::io::savePCDFile(outputCloudFile, *groundRemovedCloud) == -1)
        {
            std::cerr << "Error: 儲存點雲失敗！" << std::endl;
            return -1;
        }
    }


    // 將 finalCloud 給 viewer
    try {
        PointCloudViewer::displayProcessedCloud(groundRemovedCloud, resolution);
    }
    catch (const std::exception &e)
    {
        std::cerr << "顯示點雲時發生錯誤: " << e.what() << std::endl;
    }

   
    return 0;
}