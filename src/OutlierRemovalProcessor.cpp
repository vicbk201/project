#include "OutlierRemovalProcessor.h"
#include <pcl/filters/radius_outlier_removal.h>
#include <iostream>

namespace OutlierRemovalProcessor {

pcl::PointCloud<pcl::PointXYZI>::Ptr removeOutliers(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
    float radius,
    int min_neighbors)
{
    if (!input_cloud || input_cloud->empty()) {
        std::cerr << "[OutlierRemoval] 輸入點雲為空，跳過濾波。" << std::endl;
        return input_cloud;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::RadiusOutlierRemoval<pcl::PointXYZI> outrem;
    outrem.setInputCloud(input_cloud);
    outrem.setRadiusSearch(radius);
    outrem.setMinNeighborsInRadius(min_neighbors);
    outrem.filter(*filtered);

    std::cout << "離群點過濾後點雲數量：" << filtered->size() << std::endl;

    return filtered;
}

}
