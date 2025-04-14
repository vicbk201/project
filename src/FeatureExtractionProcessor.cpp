#include "FeatureExtractionProcessor.h"
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <cmath>

static const float DEG_PER_RAD = 57.2958f; // 180/π

ClusterFeatures FeatureExtractionProcessor::computeFeatures(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
                                                              const OrientedBoundingBox &obb,
                                                              const pcl::PointXYZ &lidarPosition)
{
    ClusterFeatures features;

    // 新增這行：印出第一個點的 intensity 值（debug 用）
    if (!cloud->empty()) {
        std::cout << "[FeatureExtractor] First point intensity: " << cloud->points[0].intensity << std::endl;
    }
    
    // 1. Cluster 點數
    features.pointCount = static_cast<int>(cloud->size());
    
    // 2. 聚類中心與 LiDAR 位置距離（以聚類質心計算）
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);
    Eigen::Vector3f cent3 = centroid.head<3>();
    Eigen::Vector3f lidarPos(lidarPosition.x, lidarPosition.y, lidarPosition.z);
    features.distance = (cent3 - lidarPos).norm();

    // 3. 以及 4. 從 OBB 中取得尺寸與 L/W 比例
    features.obb_length = std::fabs(obb.dimensions.x());
    features.obb_width  = std::fabs(obb.dimensions.y());
    features.obb_height = std::fabs(obb.dimensions.z());
    if (features.obb_width != 0)
        features.lw_ratio = features.obb_length / features.obb_width;
    else
        features.lw_ratio = 0.0f;

    // 5. PCA Eigenvalue比例計算：計算協方差矩陣並做 Eigen 分解
    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*cloud, centroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigensolver(covariance);
    // 取得 eigenvalues，依預設順序為從小到大
    Eigen::Vector3f eigenValues = eigensolver.eigenvalues();
    float sumEig = eigenValues.sum();
    features.eigenRatios.resize(3);
    if (sumEig > 0)
    {
        features.eigenRatios[0] = eigenValues[0] / sumEig;
        features.eigenRatios[1] = eigenValues[1] / sumEig;
        features.eigenRatios[2] = eigenValues[2] / sumEig;
    }
    else
    {
        features.eigenRatios[0] = features.eigenRatios[1] = features.eigenRatios[2] = 0.f;
    }

    // 6. Intensity 平均值與 7. 標準差
    float sumIntensity = 0.0f;
    float sqSumIntensity = 0.0f;
    for (const auto &pt : cloud->points)
    {
        sumIntensity += pt.intensity;
        sqSumIntensity += pt.intensity * pt.intensity;
    }
    float meanIntensity = sumIntensity / cloud->size();
    features.intensityMean = meanIntensity;
    features.intensityStdDev = std::sqrt(sqSumIntensity / cloud->size() - meanIntensity * meanIntensity);

    // 8. 局部點密度：使用 KDTree 計算每個點的最近鄰距離 (排除自身)
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud(cloud);
    double sumNNDistance = 0.0;
    int validCount = 0;
    // 當雲點數過少則跳過
    if (cloud->size() > 1)
    {
        for (size_t i = 0; i < cloud->size(); ++i)
        {
            std::vector<int> pointIdxNKNSearch(2);
            std::vector<float> pointNKNSquaredDistance(2);
            if (kdtree.nearestKSearch(cloud->points[i], 2, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
            {
                // 第一個索引是本身，第二個是最近鄰
                sumNNDistance += std::sqrt(pointNKNSquaredDistance[1]);
                validCount++;
            }
        }
        features.localDensity = (validCount > 0) ? static_cast<float>(sumNNDistance / validCount) : 0.f;
    }
    else
    {
        features.localDensity = 0.f;
    }

    // 9. 主軸與垂直方向夾角：利用 PCA 得到第一主軸（最大 eigenvalue 對應 eigenvector）
    // 可使用 pcl::PCA 來取得主軸向量（預設第一個 eigenvector對應最大 eigenvalue）
    pcl::PCA<pcl::PointXYZI> pca;
    pca.setInputCloud(cloud);
    Eigen::Vector3f principalComponent = pca.getEigenVectors().col(0);
    // 計算 principalComponent 與 (0,0,1) 的夾角
    Eigen::Vector3f vertical(0.0f, 0.0f, 1.0f);
    float dot_val = principalComponent.dot(vertical);
    // 為避免因負值導致角度超過 90 度，取絕對值
    dot_val = std::fabs(dot_val);
    float angleRad = std::acos(std::min(std::max(dot_val, 0.0f), 1.0f));  // 夾角 (弧度)
    features.mainAxisAngle = angleRad * DEG_PER_RAD; // 轉換為度

    return features;
}
