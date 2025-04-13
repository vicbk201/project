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
    float background_resolution = 0.1f;
    
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

    auto groundResult = GroundRemovalProcessor::removeGround(downsampledCloud) ;
    // auto groundResult = GroundRemovalProcessor::removeGroundWithPlane(downsampledCloud, fixed_plane, 0.08f);  // 第三個參數是距離閾值 threshold
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
    std::cout << "離群點移除後點雲數量: " << groundRemovedCloud->size() << std::endl;

    // 進行聚類處理
    pcl::PointCloud<pcl::PointXYZI>::Ptr clusteredCloud;
    if (clusterMethod == "euclidean")
    {
        auto clusteringResult = EuclideanClusterProcessor::clusterCloud(groundRemovedCloud, 3);
        std::cout << "Euclidean Cluster 後點雲數量: " << clusteringResult.cloud->size()
                  << " (算法耗時: " << std::fixed << std::setprecision(2) << clusteringResult.runtime_ms << " ms)" << std::endl;
        clusteredCloud = clusteringResult.cloud;
    }
    else if (clusterMethod == "dbscan")
    {
        auto clusteringResult = DBSCANClusteringProcessor::clusterCloud(groundRemovedCloud, 0.5, 20);
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

    std::cout << "Total clusters: " << clusterMap.size() << std::endl;

    // 先複製原 clusterMap 的所有 key（避免直接在迭代中修改 map）
    std::vector<int> clusterKeys;
    for (const auto &kv : clusterMap)
        clusterKeys.push_back(kv.first);

    // 使用 newClusterMap 儲存最終結果（拆分後的 cluster）
    std::map<int, pcl::PointCloud<pcl::PointXYZI>::Ptr> newClusterMap = clusterMap;

    // 用來保存有效 OBB（顯示用）
    std::vector<OrientedBoundingBox> validOBB;
    int newBaseLabel = 1000;

    for (auto clusterLabel : clusterKeys)
    {
        auto cloudSeg = clusterMap[clusterLabel];
        if (!cloudSeg || cloudSeg->empty()) continue;

        // 先計算整個大群的 OBB
        OrientedBoundingBox obb = OBBFittingProcessor::computeOBB(cloudSeg, &groundResult.ground_coefficients);
        float length = std::fabs(obb.dimensions.x());
        float width  = std::fabs(obb.dimensions.y());
        float height = std::fabs(obb.dimensions.z());

        // 判斷是否符合二次分群條件
        bool heightOK = (height >= 0.8f && height <= 2.0f);
        bool lengthOK = (length > 1.2f && length < 3.0f);
        bool widthOK  = (width  > 0.8f && width  < 3.0f);
        bool needMeanShift = heightOK && (lengthOK || widthOK);

        if (needMeanShift && obb.localCloud && !obb.localCloud->empty())
        {
           float bandwidth = 0.2f; // 可根據實際調整
           int   minPoints = 20;
           /*
           std::cout << "[DEBUG] clusterLabel=" << clusterLabel
                     << " -> Trigger MeanShift, band=" << bandwidth 
                     << ", minPts=" << minPoints << std::endl;
           */          
        
            // 執行 MeanShift，得到 subLabels（subLabels 的大小應與 cloudSeg->size() 相同）
            std::vector<int> subLabels = MeanShiftProcessor::separateClosePedestriansByLocalCloud(
                                                 obb, bandwidth, minPoints, newBaseLabel);
                                                
            // Debug：輸出 subLabels 統計
            std::map<int,int> labelCount;
            for (auto lbl : subLabels) {
                 labelCount[lbl]++;
            }
            /*
            std::cout << "[DEBUG] subLabels size=" << subLabels.size() 
                      << ", unique labels => "; 
            

            for (auto &p : labelCount) {
                 std::cout << p.first << "(" << p.second << " pts) ";
            }
            std::cout << std::endl;

            */

            // 檢查拆分結果：如果唯一子群數 <= 1，代表 MeanShift 沒有實際拆分出多個群
            if(labelCount.size() <= 1)
            {
                /*std::cout << "[DEBUG] clusterLabel=" << clusterLabel 
                          << " 拆分結果僅一群, 保留原 OBB" << std::endl; */
                
                // 保留原群：更新 intensity 為原 clusterLabel
                for (auto &pt : cloudSeg->points)
                    pt.intensity = float(clusterLabel);
                newClusterMap[clusterLabel] = cloudSeg;
                // 同時保留原大群 OBB（如果你希望顯示原有框）
                validOBB.push_back(obb);
            }
            else if (subLabels.size() == cloudSeg->size())
            {
                // 如果有多個子群則拆分原群：
                std::map<int, pcl::PointCloud<pcl::PointXYZI>::Ptr> subClusterMap;
                for (size_t i = 0; i < cloudSeg->size(); i++) {
                    int subLabel = subLabels[i];
                    // 忽略噪點 (-2) 的點（或根據需求處理）
                    if (subLabel == -2) continue;
                    if (subClusterMap.find(subLabel) == subClusterMap.end()) {
                        subClusterMap[subLabel] = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
                    }
                    subClusterMap[subLabel]->points.push_back(cloudSeg->points[i]);
                }
            
                // 針對拆分出的每個子群重新計算 OBB，更新 intensity，並放入 newClusterMap
                for (auto &kv_sub : subClusterMap) {
                     int newLabel = kv_sub.first; // 此 label自 MeanShift 結果
                     pcl::PointCloud<pcl::PointXYZI>::Ptr subCloud = kv_sub.second;
                     // 計算子群的 OBB
                     OrientedBoundingBox subOBB = OBBFittingProcessor::computeOBB(subCloud, &groundResult.ground_coefficients);
                     validOBB.push_back(subOBB);
                     // 更新子群每個點的 intensity 為 newLabel
                     for (auto &pt : subCloud->points)
                          pt.intensity = float(newLabel);
                     // 將子群加入 newClusterMap
                     newClusterMap[newLabel] = subCloud;
                }
                // 刪除原本的大群
                newClusterMap.erase(clusterLabel);
            
                // 更新 newBaseLabel 避免新的子群 label 與之重複
                int maxSub = -1;
                for (auto &p : subClusterMap) {
                    if (p.first > maxSub)
                        maxSub = p.first;
                }
                if (maxSub >= 0)
                    newBaseLabel = maxSub + 10;
            }
            else
            {
                std::cerr << "[Warning] MeanShift output size 與 cloudSeg size 不匹配\n";
            }
        } 
        else
        {
             // std::cout << "[DEBUG] clusterLabel=" << clusterLabel << " -> No MeanShift" << std::endl;
             // 不執行二次分群，保留原群，更新 intensity
             for (auto &pt : cloudSeg->points)
                 pt.intensity = float(clusterLabel);
             newClusterMap[clusterLabel] = cloudSeg;
             validOBB.push_back(obb);
        }
    }

    // 組合 newClusterMap 成 finalCloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr finalCloud(new pcl::PointCloud<pcl::PointXYZI>);
    for (auto &kv : newClusterMap)
    {
        auto &segCloud = kv.second;
        for (auto &pt : segCloud->points)
             finalCloud->points.push_back(pt);
    }
    finalCloud->width  = finalCloud->points.size();
    finalCloud->height = 1;
    finalCloud->is_dense = true;

    // 將 finalCloud 給 viewer
    try {
        PointCloudViewer::displayProcessedCloud(finalCloud, resolution, validOBB);
    }
    catch (const std::exception &e)
    {
        std::cerr << "顯示點雲時發生錯誤: " << e.what() << std::endl;
    }

    // 若指定要輸出檔
    if (!outputCloudFile.empty())
    {
        std::cout << "Saving processed cloud to: " << outputCloudFile << std::endl;
        if (pcl::io::savePCDFile(outputCloudFile, *finalCloud) == -1)
        {
            std::cerr << "Error: 儲存點雲失敗！" << std::endl;
            return -1;
        }
    }

    return 0;
}
