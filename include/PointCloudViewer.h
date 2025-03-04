#ifndef POINT_CLOUD_VIEWER_H
#define POINT_CLOUD_VIEWER_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class PointCloudViewer {
public:
    // 接收降維後的點雲，並根據解析度改變顯示效果
    static void displayProcessedCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr processedCloud, float resolution);
};

#endif // POINT_CLOUD_VIEWER_H
