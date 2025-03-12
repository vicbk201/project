#include "PCDLoader.h"
#include <pcl/io/pcd_io.h>
#include <iostream>

pcl::PointCloud<pcl::PointXYZI>::Ptr PCDLoader::loadPCD(const std::string &filename)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(filename, *cloud) == -1)
    {
        std::cerr << "無法讀取 PCD 檔案: " << filename << std::endl;
        return nullptr;
    }
    return cloud;
}
