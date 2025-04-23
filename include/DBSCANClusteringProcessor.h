#ifndef DBSCAN_CLUSTERING_PROCESSOR_H
#define DBSCAN_CLUSTERING_PROCESSOR_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// 結果結構，包含聚類後的點雲以及執行時間（毫秒）
struct DBSCANResult {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    double runtime_ms;
    std::vector<int> labels;    // 加入：每個點的 cluster label

};

class DBSCANClusteringProcessor {
public:
    // 使用 DBSCAN 演算法聚類點雲
    // eps: 鄰域半徑，minPts: 最少鄰域點數（密度門檻）
    static DBSCANResult clusterCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, double eps, int minPts);
};

#endif // DBSCAN_CLUSTERING_PROCESSOR_H

