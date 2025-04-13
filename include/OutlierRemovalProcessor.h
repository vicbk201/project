#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace OutlierRemovalProcessor {
    pcl::PointCloud<pcl::PointXYZI>::Ptr removeOutliers(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
        float radius = 0.35f,
        int min_neighbors = 3
    );
}
