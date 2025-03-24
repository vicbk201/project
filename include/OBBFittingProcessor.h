#ifndef OBB_FITTING_PROCESSOR_H
#define OBB_FITTING_PROCESSOR_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

typedef pcl::PointXYZI PointType;

// 定義 OrientedBoundingBox 結構：中心、四元數表示的方向以及尺寸
struct OrientedBoundingBox {
    Eigen::Vector3f center;
    Eigen::Quaternionf orientation;
    Eigen::Vector3f dimensions;
};

class OBBFittingProcessor {
public:
    // 給定一個點雲聚類，計算其 OBB
    static OrientedBoundingBox computeOBB(const pcl::PointCloud<PointType>::Ptr &cloud_cluster);
};

#endif // OBB_FITTING_PROCESSOR_H
