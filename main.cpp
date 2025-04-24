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
#include "FeatureExporter.h"
#include <filesystem>
#include <tuple>


 namespace fs = std::filesystem;

int main(int argc, char **argv)
{
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

    std::string cloudFile;
    float resolution;
    std::string downsampleMethod;
    std::string clusterMethod;
    std::string outputCloudFile;

    // 解析命令列參數 (cloudFile 既可為單一 .pcd，也可為資料夾路徑)
    if (!ArgumentParser::parseArguments(argc, argv, cloudFile, resolution, downsampleMethod, clusterMethod, outputCloudFile))
    {
        std::cerr << "正確使用方式: " << argv[0]
                  << " --cloudfile <PCD檔案路徑> --method <octree|voxelgrid> --cluster <euclidean|dbscan> [--resolution <解析度>] [--o|--output <完整輸出PCD檔案路徑>]"
                  << std::endl;
        return -1;
    }

    std::cout << "Input Path: "   << cloudFile << std::endl;
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
   
    //─── 建立待處理清單：單檔或資料夾下所有 .pcd
    std::vector<std::string> fileList;
    if (fs::is_directory(cloudFile)) {
    // 遞迴掃描所有子目錄的 .pcd
    for (auto &e : fs::recursive_directory_iterator(cloudFile)) {
        if (e.is_regular_file() && e.path().extension() == ".pcd")
            fileList.push_back(e.path().string());
    }
    // Optional：去重
    std::sort(fileList.begin(), fileList.end());
    fileList.erase(std::unique(fileList.begin(), fileList.end()),
                   fileList.end());
    } else {
    // 單檔模式
    fileList.push_back(cloudFile);
    }

    // 最後一次性輸出的 CSV 路徑 (append 模式)
    const std::string csvPath = "/home/semilux/octreevoxelgrid/data/pedestrian_features.csv";
    // 要累積全檔案的 features
    std::vector<std::tuple<std::string,int,ClusterFeatures>> allFeatures;
    std::set<int>    allLabels;  // 用於每筆的 LabelFlag
    // 判斷是否為批次資料夾模式
    bool batchMode = fs::is_directory(cloudFile);

    // 批次處理前，保留最後一次單檔的 clusterMap / labels / OBB
    std::map<int, pcl::PointCloud<pcl::PointXYZI>::Ptr> vizClusterMap;
    std::set<int> vizValidLabels;
    std::vector<OrientedBoundingBox> vizValidOBB;

    // 批次處理每一支 .pcd
    for (auto &thisFile : fileList)
    {
        std::cout << "── 處理檔案: " << thisFile << std::endl;
        auto raw_input = PCDLoader::loadPCD(thisFile);
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
        auto groundResult = GroundRemovalProcessor::removeGroundWithPlane(downsampledCloud, fixed_plane, 0.08f);  // 第三個參數是距離閾值 threshold
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
        std::vector<int> labels;
        if (clusterMethod == "euclidean")
        {
            auto clusteringResult = EuclideanClusterProcessor::clusterCloud(groundRemovedCloud, 0.5);
            std::cout << "Euclidean Cluster 後點雲數量: " << clusteringResult.cloud->size()
                      << " (算法耗時: " << std::fixed << std::setprecision(2) << clusteringResult.runtime_ms << " ms)" << std::endl;
            clusteredCloud = clusteringResult.cloud;
            labels = std::move(clusteringResult.labels);
        }
        else if (clusterMethod == "dbscan")
        {
            auto clusteringResult = DBSCANClusteringProcessor::clusterCloud(groundRemovedCloud, 0.5, 20);
            std::cout << "DBSCAN Cluster 後點雲數量: " << clusteringResult.cloud->size()
                      << " (算法耗時: " << std::fixed << std::setprecision(2) << clusteringResult.runtime_ms << " ms)" << std::endl;
            clusteredCloud = clusteringResult.cloud;
            labels = std::move(clusteringResult.labels);
        }
        else
        {
            std::cerr << "Error: 未知的 clustering method: " << clusterMethod << std::endl;
            return -1;
        }

        //用 labels + 原始強度 建 clusterMap
        std::map<int, pcl::PointCloud<pcl::PointXYZI>::Ptr> clusterMap;
        for (size_t i = 0; i < clusteredCloud->points.size(); ++i) {
            int lbl = labels[i];
            if (lbl < 0) continue;  // 跳過噪點
            if (!clusterMap.count(lbl))
                clusterMap[lbl] = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
        // 用 groundRemovedCloud 保留原始 intensity
        clusterMap[lbl]->points.push_back(groundRemovedCloud->points[i]);
        
        }  
        std::cout << "原始 clusters: " << clusterMap.size() << "\n";

    /*
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
    */
    
    /*
    // 計算每個聚類的 OBB 並根據尺寸過濾只保留人
    std::vector<OrientedBoundingBox> validOBB;
    int validCount = 0;
    for (const auto &cluster : clusterMap)
    {
        int clusterLabel = cluster.first;
        OrientedBoundingBox obb = OBBFittingProcessor::computeOBB(cluster.second);
        validOBB.push_back(obb);

        
        // 直接取對齊後的尺寸
        float length = std::fabs(obb.dimensions.x());  // x 為長度
        float width  = std::fabs(obb.dimensions.y());  // y 為寬度
        float height = std::fabs(obb.dimensions.z());  // z 為高度

        // 篩選條件（根據您的應用調整閾值）
        bool isHuman = (height >= 0.8f && height <= 2.0f && length <= 1.2f && width <= 1.2f);

        if (isHuman)
        {
        validOBB.push_back(obb);
        validCount++;
        }
        
    }
   
    std::cout << "Valid clusters:" << validCount <<std::endl;

    clusteredCloud->width = static_cast<uint32_t>(clusteredCloud->points.size());
    clusteredCloud->height = 1;
    clusteredCloud->is_dense = true;
    */     

    
        // 先複製原 clusterMap 的所有 key（避免直接在迭代中修改 map）
        std::vector<int> clusterKeys;
        for (const auto &kv : clusterMap)
            clusterKeys.push_back(kv.first);

        // 使用 newClusterMap 儲存最終結果（拆分後的 cluster）
        std::map<int, pcl::PointCloud<pcl::PointXYZI>::Ptr> newClusterMap = clusterMap;

        // 用來保存有效 OBB（顯示用）
        std::vector<OrientedBoundingBox> validOBB;
        std::set<int> validLabels;
        int newBaseLabel = 1000;

        //保存特徵
        std::vector<std::tuple<std::string,int, ClusterFeatures>> featuresList;

        // 假設 LiDAR 安裝位置
        pcl::PointXYZ lidarPosition(0.0f, 0.0f, 2.4f);
  
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
    
            std::cout << "[DEBUG] Cluster " << clusterLabel 
                      << " OBB dimensions: length = " << length 
                      << ", width = " << width 
                      << ", height = " << height << std::endl;

            // 定義條件：只顯示符合單一行人或多人候選的聚類
            bool singleCandidate = (length < 1.2f && width < 1.2f && height >= 0.8f && height <= 2.0f);
            bool multiCandidate  = (!singleCandidate) &&
                                   (((length > 1.2f && length < 3.0f) || (width > 1.2f && width < 3.0f)) &&
                                    (height >= 0.8f && height <= 2.0f));
    
            if(singleCandidate)
            {
                // ─── 只對「單人候選」做特徵提取 ─────────────────────────────────────────
                ClusterFeatures feat = FeatureExtractionProcessor::computeFeatures(cloudSeg, obb, lidarPosition);

                std::cout << "=== Cluster " << clusterLabel << " Features ===\n";
                std::cout << "PointCount       : " << feat.pointCount        << "\n";
                std::cout << "CentroidDistance : " << feat.centroidDistance  << "\n";
                std::cout << "AverageDistance  : " << feat.averageDistance   << "\n";

                // Covariance Matrix (3×3)
                std::cout << "CovarianceMatrix : [";
                for (int i = 0; i < 9; ++i)
                    std::cout << feat.covarianceMatrix[i] << (i<8?", ":"");
                std::cout << "]\n";

                // Normalized Moments (6)
                std::cout << "NormalizedMoments: [";
                for (int i = 0; i < 6; ++i)
                    std::cout << feat.normalizedMoments[i] << (i<5?", ":"");
                std::cout << "]\n";

                std::cout << "OBB (L×W×H)      : "
                          << feat.obb_length << " × "
                          << feat.obb_width  << " × "
                          << feat.obb_height << "\n";
                std::cout << "L/W Ratio        : " << feat.lw_ratio       << "\n";

                std::cout << "EigenRatios      : [";
                for (int i = 0; i < 3; ++i)
                    std::cout << feat.eigenRatios[i] << (i<2?", ":"");
                std::cout << "]\n";

                std::cout << "IntensityMean    : " << feat.intensityMean   << "\n";
                std::cout << "IntensityStdDev  : " << feat.intensityStdDev << "\n";
                std::cout << "LocalDensity     : " << feat.localDensity    << "\n";
                std::cout << "MainAxisAngle    : " << feat.mainAxisAngle << " deg\n";
                std::cout << "========================================\n";
                // ────────────────────────────────────────────────────────────────


                // 單一行人直接加入 OBB 顯示
                for (auto &pt : cloudSeg->points)
                     pt.intensity = float(clusterLabel);
                newClusterMap[clusterLabel] = cloudSeg;
                validOBB.push_back(obb);
                // 加這一行，把 (thisFile,label, feat) 推進 featuresList
                featuresList.emplace_back(thisFile, clusterLabel, feat);
                validLabels.insert(clusterLabel);   // ← 直接記錄這個 clusterLabel
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
                        if (subLabel < 0) continue; // 忽略噪點
                        if (subClusterMap.find(subLabel) == subClusterMap.end())
                            subClusterMap[subLabel] = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
                        subClusterMap[subLabel]->points.push_back(cloudSeg->points[i]);
                    }

                    for (auto &kv_sub : subClusterMap)
                    {
                        int newLabel = kv_sub.first;
                        pcl::PointCloud<pcl::PointXYZI>::Ptr subCloud = kv_sub.second;
                        OrientedBoundingBox subOBB = OBBFittingProcessor::computeOBB(subCloud, &groundResult.ground_coefficients);

                        // ─── 同樣在這裡做特徵提取並印出 ─────────────────────────
                        ClusterFeatures feat = FeatureExtractionProcessor::computeFeatures(subCloud, subOBB, lidarPosition);
                        std::cout << "--- Sub-Cluster " << newLabel << " Features ---\n";
                        std::cout << "PointCount       : " << feat.pointCount        << "\n";
                        std::cout << "CentroidDistance : " << feat.centroidDistance  << "\n";
                        std::cout << "AverageDistance  : " << feat.averageDistance   << "\n";
                        std::cout << "CovarianceMatrix : [";
                        for (int i = 0; i < 9; ++i) std::cout << feat.covarianceMatrix[i] << (i<8?", ":"");
                        std::cout << "]\n";
                        std::cout << "NormalizedMoments: [";
                        for (int i = 0; i < 6; ++i) std::cout << feat.normalizedMoments[i] << (i<5?", ":"");
                        std::cout << "]\n";
                        std::cout << "OBB (L×W×H)      : " << feat.obb_length << "×"
                                  << feat.obb_width << "×" << feat.obb_height << "\n";
                        std::cout << "L/W Ratio        : " << feat.lw_ratio       << "\n";
                        std::cout << "EigenRatios      : [";
                        for (int i = 0; i < 3; ++i) std::cout << feat.eigenRatios[i] << (i<2?", ":"");
                        std::cout << "]\n";
                        std::cout << "IntensityMean    : " << feat.intensityMean   << "\n";
                        std::cout << "IntensityStdDev  : " << feat.intensityStdDev << "\n";
                        std::cout << "LocalDensity     : " << feat.localDensity    << "\n";
                        std::cout << "MainAxisAngle    : " << feat.mainAxisAngle   << " deg\n";
                        std::cout << "---------------------------------------\n";
                    
                        // 同樣把每個子群 (this, newLabel, feat) 推進 featuresList
                        featuresList.emplace_back(thisFile, newLabel, feat);
                        validLabels.insert(newLabel);    // ← 直接記錄這個 subLabel
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

    
        // ─── 全部特徵都收集完後，先累積到 allFeatures
        allFeatures.insert(
            allFeatures.end(),
            featuresList.begin(),
            featuresList.end());
        // LabelFlag (1 表示「淺在行人」)
        allLabels.insert(validLabels.begin(), validLabels.end());
        
        // 如果是單檔模式，保留本輪結果供可視化 (batchMode = false)
        if (!batchMode) {
            vizClusterMap  = newClusterMap;
            vizValidLabels = validLabels;
            vizValidOBB    = validOBB;
        }
    } // end for each pcd

    // 最後一次性輸出：如果 data 目錄不存在就先建
    if (!fs::exists("data")) fs::create_directory("data");
    FeatureExporter::exportToCSV(
        allFeatures,
        csvPath,
        allLabels
    );
   
    // 8. 組 finalCloud（使用最後一次保留的 vizClusterMap & vizValidLabels）
    pcl::PointCloud<pcl::PointXYZI>::Ptr finalCloud(new pcl::PointCloud<pcl::PointXYZI>);
    for (auto &kv : vizClusterMap) {
        if (vizValidLabels.count(kv.first) == 0) continue;
        finalCloud->points.insert(
            finalCloud->points.end(),
            kv.second->points.begin(),
            kv.second->points.end());
    }

    finalCloud->width  = finalCloud->points.size();
    finalCloud->height = 1;
    finalCloud->is_dense = true;
    
    /*
    // 組合 newClusterMap 成 finalCloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr finalCloud(new pcl::PointCloud<pcl::PointXYZI>);
    std::cout << "拆分cluster: " << newClusterMap.size() << std::endl;
    // 組合只有 validOBB 中的 cluster（表示符合條件的 OBB 才會輸出）
    std::set<int> validLabels;
    for (const auto& obb : validOBB)
    {
        // OBBFittingProcessor 裡應該要有 label，你可以在 OrientedBoundingBox 裡加入 int label;
        // 如果沒有，可以考慮在 push_back 時搭配 label 一起儲存
        // 這裡暫時改成透過 intensity 推估
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

    finalCloud->width  = finalCloud->points.size();
    finalCloud->height = 1;
    finalCloud->is_dense = true;
    */

    /*
    // 組合 newClusterMap 成 finalCloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr finalCloud(new pcl::PointCloud<pcl::PointXYZI>);
    std::cout << "拆分cluster: " << newClusterMap.size() << std::endl;
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
        std::cout << "Saving processed cloud to: " << outputCloudFile << std::endl;
        if (pcl::io::savePCDFile(outputCloudFile, *finalCloud) == -1)
        {
            std::cerr << "Error: 儲存點雲失敗！" << std::endl;
            return -1;
        }
    }

    // （可視化）如果是單檔模式，才跳出 viewer；批次模式就略過
    if (!batchMode)
    {
        try {
            PointCloudViewer::displayProcessedCloud(finalCloud, resolution, vizValidOBB);
        }
        catch (const std::exception &e)
        {
            std::cerr << "顯示點雲時發生錯誤: " << e.what() << std::endl;
        }
    }
    else
    {
        std::cout << "[Batch mode] 可視化已略過，完成全部檔案處理並輸出 CSV。" << std::endl;
    }
 

    return 0;
}