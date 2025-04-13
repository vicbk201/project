#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

class HeadCountingSplitter {
public:
    // 將單一 cluster 根據頭部數量再切割為多個子群
    static std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>
    splitByHeadCount(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cluster,
                     float head_z_min = 1.4f,
                     float head_z_max = 2.0f,
                     float bin_size = 0.1f,
                     int peak_threshold = 20);
};
