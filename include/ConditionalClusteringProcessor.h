#ifndef CONDITIONAL_CLUSTERING_PROCESSOR_H
#define CONDITIONAL_CLUSTERING_PROCESSOR_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct ClusteringResult {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    double runtime_ms;
};

class ConditionalClusteringProcessor {
public:
    static ClusteringResult clusterCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud);
};

#endif // CONDITIONAL_CLUSTERING_PROCESSOR_H

