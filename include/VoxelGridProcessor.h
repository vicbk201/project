#ifndef VOXEL_GRID_PROCESSOR_H
#define VOXEL_GRID_PROCESSOR_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

struct VoxelGridResult {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;
    double runtime_ms;
};

class VoxelGridProcessor {
public:
    static VoxelGridResult processVoxelGrid(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float resolution);
};

#endif // VOXEL_GRID_PROCESSOR_H
