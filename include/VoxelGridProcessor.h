#ifndef VOXEL_GRID_PROCESSOR_H
#define VOXEL_GRID_PROCESSOR_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class VoxelGridProcessor {
public:
    static pcl::PointCloud<pcl::PointXYZ>::Ptr processVoxelGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float resolution);
};

#endif // VOXEL_GRID_PROCESSOR_H
