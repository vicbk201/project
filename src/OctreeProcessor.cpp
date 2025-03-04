#include "OctreeProcessor.h"
#include <pcl/octree/octree_search.h>
#include <pcl/octree/octree_pointcloud.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr OctreeProcessor::processOctree(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, float resolution)
{
    pcl::octree::OctreePointCloud<pcl::PointXYZ> octree(resolution);
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();

    // 使用 PCL 定義的對齊向量類型來存儲 Voxel 中心點
    pcl::octree::OctreePointCloud<pcl::PointXYZ>::AlignedPointTVector voxelCenters;
    octree.getOccupiedVoxelCenters(voxelCenters);

    // 將其轉換為 PCL 點雲類型
    pcl::PointCloud<pcl::PointXYZ>::Ptr voxelCloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto& point : voxelCenters)
    {
        voxelCloud->push_back(point);
    }

    return voxelCloud;
}
