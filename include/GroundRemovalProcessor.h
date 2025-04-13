#ifndef GROUND_REMOVAL_PROCESSOR_H
#define GROUND_REMOVAL_PROCESSOR_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <pcl/ModelCoefficients.h> 


// 地面去除結果的結構
struct GroundRemovalResult {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud;    // 非地面點雲
    pcl::PointCloud<pcl::PointXYZI>::Ptr ground;   // 被分類為地面的點雲
    double runtime_ms;                             // 執行耗時（毫秒）
    Eigen::Vector4f ground_coefficients;           // RANSAC 擬合得到的地面平面係數
};

class GroundRemovalProcessor {
public:
    // 使用 RANSAC 動態偵測地面平面並分割地面與非地面點
    static GroundRemovalResult removeGround(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud);

    // 使用已知地面平面係數與距離閾值，將輸入點雲分為地面與非地面
    static GroundRemovalResult removeGroundWithPlane(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                                                     const pcl::ModelCoefficients::Ptr &plane_coeff,
                                                     float threshold);
};

#endif // GROUND_REMOVAL_PROCESSOR_H
