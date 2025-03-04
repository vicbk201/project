#include "PCDLoader.h"
#include <pcl/io/pcd_io.h>
#include <iostream>

pcl::PointCloud<pcl::PointXYZ>::Ptr PCDLoader::loadPCD(const std::string &filename)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud) == -1)
    {
        std::cerr << "無法讀取 PCD 檔案: " << filename << std::endl;
        return nullptr;
    }
    return cloud;
}