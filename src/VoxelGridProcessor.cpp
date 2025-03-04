#include "VoxelGridProcessor.h"
#include <pcl/filters/voxel_grid.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr VoxelGridProcessor::processVoxelGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float resolution)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud);
    vg.setLeafSize(resolution, resolution, resolution);
    vg.filter(*filteredCloud);
    return filteredCloud;
}
