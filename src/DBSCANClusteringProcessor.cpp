#include "DBSCANClusteringProcessor.h"
#include <pcl/search/kdtree.h>
#include <pcl/console/time.h>
#include <queue>
#include <vector>

// DBSCANResult 定義包含聚類後的點雲與運算耗時
DBSCANResult DBSCANClusteringProcessor::clusterCloud(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, double eps, int minPts)
{
    // 建立計時器，計算整個演算法運行時間
    pcl::console::TicToc tt;
    tt.tic();

    // 使用智慧指標建立 KDTree，目的是加速鄰域搜尋
    // KDTree 是一種資料結構，可快速搜尋在空間中與查詢點距離小於 eps 的所有點
    typename pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(cloud);  // 將點雲資料設定到 KDTree 中

    // 取得點雲中點的數量
    size_t numPoints = cloud->points.size();

    // 建立 visited 陣列，記錄每個點是否已經被訪問過，初始值皆為 false
    std::vector<bool> visited(numPoints, false);
    // 建立 labels 陣列，記錄每個點的叢集標籤，初始值設為 -1，代表尚未分配或視為噪點
    std::vector<int> labels(numPoints, -1);
    // 初始化叢集編號，從 0 開始
    int clusterId = 0;

    // 對每個點進行遍歷
    for (size_t i = 0; i < numPoints; i++) {
        // 如果該點已經被訪問過，則跳過
        if (visited[i])
            continue;
        // 標記當前點為已訪問
        visited[i] = true;

        // 進行鄰域搜尋：找出與當前點距離小於 eps 的所有點索引與其距離
        std::vector<int> neighborIndices;
        std::vector<float> neighborDistances;
        tree->radiusSearch(cloud->points[i], eps, neighborIndices, neighborDistances);

        // 如果在 eps 半徑內的鄰居數量小於 minPts，則將該點標記為噪點（留作 -1）
        if (neighborIndices.size() < static_cast<size_t>(minPts)) {
            labels[i] = -1;
            continue;
        }

        // 若鄰域點數足夠，則視為一個新的核心點，開始建立新的叢集
        labels[i] = clusterId;
        // 使用一個佇列來展開鄰域搜尋，實作 DBSCAN 中的「expand cluster」
        std::queue<int> expansionQueue;
        // 將當前點的所有鄰居（排除自身）加入擴展佇列
        for (int idx : neighborIndices) {
            if (idx != static_cast<int>(i))
                expansionQueue.push(idx);
        }

        // 當擴展佇列不為空時，持續擴展該叢集
        while (!expansionQueue.empty()) {
            int currIdx = expansionQueue.front();
            expansionQueue.pop();

            // 若該鄰居點尚未被訪問
            if (!visited[currIdx]) {
                // 將該點標記為已訪問
                visited[currIdx] = true;
                // 針對這個點再進行鄰域搜尋
                std::vector<int> currNeighborIndices;
                std::vector<float> currNeighborDistances;
                tree->radiusSearch(cloud->points[currIdx], eps, currNeighborIndices, currNeighborDistances);

                // 如果這個點的鄰域點數大於等於 minPts，
                // 表示該點也是一個核心點，將其鄰域中未訪問的點加入擴展佇列
                if (currNeighborIndices.size() >= static_cast<size_t>(minPts)) {
                    for (int nIdx : currNeighborIndices) {
                        if (!visited[nIdx])
                            expansionQueue.push(nIdx);
                    }
                }
            }
            // 若該點尚未被分配到任何叢集（標記為 -1），則將其分配到當前的叢集
            if (labels[currIdx] == -1)
                labels[currIdx] = clusterId;
        }
        // 完成當前叢集的擴展後，將叢集編號遞增，準備建立下一個叢集
        clusterId++;
    }

    // 將聚類結果寫回 intensity 欄位：
    // 噪點設定為 -2，其餘直接使用對應的聚類編號
    for (size_t i = 0; i < numPoints; i++) {
        if (labels[i] == -1)
            cloud->points[i].intensity = -2;  // 噪點
        else
            cloud->points[i].intensity = static_cast<float>(labels[i]);
    }


    // 計算整個演算法的執行時間（毫秒）
    double elapsed = tt.toc();

    // 將結果存入 DBSCANResult 結構中返回
    DBSCANResult result;
    result.cloud = cloud;
    result.runtime_ms = elapsed;
    return result;
}
