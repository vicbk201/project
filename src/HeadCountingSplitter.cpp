#include "HeadCountingSplitter.h"
#include <pcl/common/common.h>
#include <pcl/kdtree/kdtree.h>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <map>
#include "DBSCANClusteringProcessor.h"

// 將單一 cluster 根據頭部高度區域的密度分布切割為多個子群，使用 DBSCAN 作為二次分割方法
std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> HeadCountingSplitter::splitByHeadCount(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &cluster,
    float head_z_min,
    float head_z_max,
    float bin_size,
    int peak_threshold)
{
    std::vector<int> histogram(static_cast<int>((head_z_max - head_z_min) / bin_size), 0);
    std::vector<pcl::PointXYZI> headCandidates;

    // 統計 z 值落在 [head_z_min, head_z_max] 區間的點數分布
    for (const auto &pt : cluster->points) {
        if (pt.z >= head_z_min && pt.z < head_z_max) {
            int bin = static_cast<int>((pt.z - head_z_min) / bin_size);
            histogram[bin]++;
            headCandidates.push_back(pt);
        }
    }

    // 計算 histogram 中達到 threshold 的峰值數量
    int peak_count = 0;
    for (int count : histogram) {
        if (count >= peak_threshold)
            peak_count++;
    }

    if (peak_count < 2) {
        // 若認定為單人群，直接回傳原始 cluster
        return {cluster};
    }

    // 多人群：使用 DBSCAN 進行分割
    // 建立 DBSCANClusteringProcessor 實例
    DBSCANClusteringProcessor dbscanProc;
    // 設定 DBSCAN 的參數（依資料特性調整）
    double eps = 0.2;  // 鄰域半徑，可根據實際點雲尺度調整
    int minPts = 5;    // 最少鄰居數量

    // 注意：DBSCANClusteringProcessor::clusterCloud() 會將結果寫回 cloud 的 intensity 欄位，
    // 噪點標記為 -2，其它點則用聚類編號表示
    DBSCANResult dbscanResult = dbscanProc.clusterCloud(cluster, eps, minPts);

    // 根據 DBSCAN 分群標籤將點分到各個子群
    std::map<int, pcl::PointCloud<pcl::PointXYZI>::Ptr> subClusterMap;
    for (const auto &pt : dbscanResult.cloud->points) {
        int label = static_cast<int>(pt.intensity);
        // 跳過噪點（可根據需求決定是否捨棄噪點或獨立處理）
        if (label == -2)
            continue;
        if (subClusterMap.find(label) == subClusterMap.end())
            subClusterMap[label] = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
        subClusterMap[label]->points.push_back(pt);
    }

    // 將所有子群轉換為 vector 回傳
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> result;
    for (auto &kv : subClusterMap) {
        result.push_back(kv.second);
    }

    return result;
}
