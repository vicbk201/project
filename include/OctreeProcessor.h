
#ifndef OCTREE_PROCESSOR_H
#define OCTREE_PROCESSOR_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class OctreeProcessor {
public:
    static pcl::PointCloud<pcl::PointXYZ>::Ptr processOctree(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, float resolution);
};

#endif // OCTREE_PROCESSOR_H