#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

struct OrientedBoundingBox
{
    Eigen::Vector3f center;
    Eigen::Quaternionf orientation;
    Eigen::Vector3f dimensions;

    // ===== 新增，用於保存 OBB 計算後的局部點雲 (已做 yaw-align, ground-correction) =====
    pcl::PointCloud<pcl::PointXYZI>::Ptr localCloud;
};

class OBBFittingProcessor
{
public:
    typedef pcl::PointXYZI PointType;

    static OrientedBoundingBox computeOBB(
        const pcl::PointCloud<PointType>::Ptr &cloud_cluster,
        const Eigen::Vector4f* ground_coeff_ptr = nullptr
    );
};
