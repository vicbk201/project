#ifndef EUCLIDEAN_CLUSTER_PROCESSOR_H
#define EUCLIDEAN_CLUSTER_PROCESSOR_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct EuclideanClusteringResult {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    double runtime_ms;
};

class EuclideanClusterProcessor {
public:
    // 使用 PCL 的 Conditional Euclidean Clustering 進行聚類，
    // clusterTolerance 為鄰域距離
    static EuclideanClusteringResult clusterCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                                                   double clusterTolerance = 0.5);
};

#endif // EUCLIDEAN_CLUSTER_PROCESSOR_H
