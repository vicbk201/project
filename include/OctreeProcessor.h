#ifndef OCTREE_PROCESSOR_H
#define OCTREE_PROCESSOR_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct OctreeResult {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    double runtime_ms;
};

class OctreeProcessor {
public:
    static OctreeResult processOctree(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, float resolution);
};

#endif // OCTREE_PROCESSOR_H
