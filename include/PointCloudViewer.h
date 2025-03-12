#ifndef POINT_CLOUD_VIEWER_H
#define POINT_CLOUD_VIEWER_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class PointCloudViewer {
public:
    static void displayProcessedCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr processedCloud, float resolution);

};

#endif // POINT_CLOUD_VIEWER_H
