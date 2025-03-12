#ifndef GROUND_REMOVAL_PROCESSOR_H
#define GROUND_REMOVAL_PROCESSOR_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct GroundRemovalResult {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    double runtime_ms;
};

class GroundRemovalProcessor {
public:
    static GroundRemovalResult removeGround(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud);
};

#endif // GROUND_REMOVAL_PROCESSOR_H
