#include "BackgroundRemovalProcessor.h"
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include <iostream>

namespace BackgroundRemovalProcessor {

pcl::PointCloud<pcl::PointXYZI>::Ptr removeBackgroundByOctree(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& current_cloud,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& background_cloud,
    float resolution)
{
    if (!current_cloud || current_cloud->empty()) {
        std::cerr << "[背景濾除] 當前點雲為空，無法處理。" << std::endl;
        return current_cloud;
    }
    if (!background_cloud || background_cloud->empty()) {
        std::cerr << "[背景濾除] 背景點雲為空，返回原始輸入。" << std::endl;
        return current_cloud;
    }

    // 建立 octree
    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZI> octree(resolution);

    // 第一次輸入：背景點雲
    octree.setInputCloud(background_cloud);
    octree.addPointsFromInputCloud();

    // 切換 buffer（儲存背景 voxel）
    octree.switchBuffers();

    // 第二次輸入：當前點雲
    octree.setInputCloud(current_cloud);
    octree.addPointsFromInputCloud();

    // 找出新增的 voxel（即前景點）
    std::vector<int> new_point_indices;
    octree.getPointIndicesFromNewVoxels(new_point_indices);

    pcl::PointCloud<pcl::PointXYZI>::Ptr foreground(new pcl::PointCloud<pcl::PointXYZI>);
    foreground->reserve(new_point_indices.size());

    for (int idx : new_point_indices) {
        foreground->points.push_back(current_cloud->points[idx]);
    }

    foreground->width = foreground->points.size();
    foreground->height = 1;
    foreground->is_dense = true;

    std::cout << "原始點雲：" << current_cloud->size()
              << "，濾除後剩餘：" << foreground->size() << std::endl;

    return foreground;
}

}
