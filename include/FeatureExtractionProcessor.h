#ifndef FEATURE_EXTRACTION_PROCESSOR_H
#define FEATURE_EXTRACTION_PROCESSOR_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <Eigen/Dense>
#include <vector>

// 假設你的 OBB 模組中定義了 OrientedBoundingBox 結構，類似如下：
struct OrientedBoundingBox
{
    Eigen::Vector3f position;    // 中心點
    Eigen::Vector3f dimensions;  // 尺寸 (L, W, H)
    Eigen::Matrix3f rotation;    // 方向矩陣（主軸）
    pcl::PointCloud<pcl::PointXYZI>::Ptr localCloud; // 該 OBB 對應內部點雲（如果有）
};

struct ClusterFeatures
{
    int pointCount;             // 1 維
    float distance;             // 1 維：聚類中心與 LiDAR 位置之歐式距離
    float obb_length;           // 1 維：OBB 長
    float obb_width;            // 1 維：OBB 寬
    float obb_height;           // 1 維：OBB 高
    float lw_ratio;             // 1 維：L/W 比例
    std::vector<float> eigenRatios; // 3 維：PCA Eigenvalue 正規化比例（依小到大排序）
    float intensityMean;        // 1 維
    float intensityStdDev;      // 1 維
    float localDensity;         // 1 維：平均最近鄰距離
    float mainAxisAngle;        // 1 維：第一主軸與垂直向量 (0,0,1) 之夾角（單位：度）
};

class FeatureExtractionProcessor
{
public:
    // 輸入聚類點雲、已計算的 OBB，以及光達的安裝位置
    // lidarPosition 可根據你的專案設置（例如 2.3m 高的光達位置，以 pcl::PointXYZ 表示）
    static ClusterFeatures computeFeatures(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                                             const OrientedBoundingBox &obb,
                                             const pcl::PointXYZ &lidarPosition);
};

#endif // FEATURE_EXTRACTION_PROCESSOR_H
