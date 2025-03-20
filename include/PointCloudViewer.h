#ifndef POINTCLOUDVIEWER_H
#define POINTCLOUDVIEWER_H

#include <pcl/visualization/pcl_visualizer.h>
#include "OBBFittingProcessor.h"  

class PointCloudViewer {
public:
    static void displayProcessedCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr processedCloud, float resolution, const std::vector<OrientedBoundingBox>& obb_list);
};

#endif // POINTCLOUDVIEWER_H
