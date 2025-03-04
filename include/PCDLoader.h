#ifndef PCD_LOADER_H
#define PCD_LOADER_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>

class PCDLoader {
public:
    static pcl::PointCloud<pcl::PointXYZ>::Ptr loadPCD(const std::string &filename);
};

#endif // PCD_LOADER_H
