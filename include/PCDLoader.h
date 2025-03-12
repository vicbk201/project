#ifndef PCDLOADER_H
#define PCDLOADER_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>

class PCDLoader {
public:
    // 載入點雲時使用 PointXYZI 型態
    static pcl::PointCloud<pcl::PointXYZI>::Ptr loadPCD(const std::string &filename);
};

#endif // PCDLOADER_H
