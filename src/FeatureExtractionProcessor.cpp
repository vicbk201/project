// FeatureExtractionProcessor.cpp
#include "FeatureExtractionProcessor.h"
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <cmath>

static const float DEG_PER_RAD = 57.2958f; // 角度轉換常數 (180/π)

ClusterFeatures FeatureExtractionProcessor::computeFeatures(
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
    const OrientedBoundingBox &obb,
    const pcl::PointXYZ &lidarPosition)
{
    ClusterFeatures features;

    // 1. 計算點數量
    features.pointCount = static_cast<int>(cloud->size());

    // 2. 計算質心到 LiDAR 的距離
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);
    Eigen::Vector3f cent3 = centroid.head<3>();
    Eigen::Vector3f lidarPos(lidarPosition.x, lidarPosition.y, lidarPosition.z);
    features.centroidDistance = (cent3 - lidarPos).norm();

    // 3. 計算所有點到 LiDAR 的平均距離
    double sumDist = 0.0;
    for (const auto& pt : cloud->points) {
        Eigen::Vector3f p(pt.x, pt.y, pt.z);
        sumDist += (p - lidarPos).norm();
    }
    features.averageDistance = (features.pointCount > 0) ? static_cast<float>(sumDist / features.pointCount) : 0.f;

    // 4. 計算 3x3 協方差矩陣
    Eigen::Matrix3f cov;
    pcl::computeCovarianceMatrixNormalized(*cloud, centroid, cov);
    features.covarianceMatrix.resize(9);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            features.covarianceMatrix[i*3 + j] = cov(i, j);

    // 5. 計算歸一化慣性張量的 6 個分量
    float Ixx=0, Iyy=0, Izz=0, Ixy=0, Ixz=0, Iyz=0;
    for (const auto& pt : cloud->points) {
        Eigen::Vector3f d(pt.x - cent3.x(), pt.y - cent3.y(), pt.z - cent3.z());
        Ixx += d.y()*d.y() + d.z()*d.z();
        Iyy += d.x()*d.x() + d.z()*d.z();
        Izz += d.x()*d.x() + d.y()*d.y();
        Ixy += -d.x()*d.y();
        Ixz += -d.x()*d.z();
        Iyz += -d.y()*d.z();
    }
    if (features.pointCount > 0) {
        float invN = 1.0f / features.pointCount;
        features.normalizedMoments = {Ixx*invN, Iyy*invN, Izz*invN,
                                      Ixy*invN, Ixz*invN, Iyz*invN};
    } else {
        features.normalizedMoments.assign(6, 0.f);
    }

    // 6-8. OBB 長寬高與長寬比
    features.obb_length = std::fabs(obb.dimensions.x());
    features.obb_width  = std::fabs(obb.dimensions.y());
    features.obb_height = std::fabs(obb.dimensions.z());
    features.lw_ratio   = (features.obb_width != 0.f) ? features.obb_length / features.obb_width : 0.f;

    // 9. 計算 PCA 特徵值比例
    pcl::computeCovarianceMatrixNormalized(*cloud, centroid, cov);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(cov);
    Eigen::Vector3f eigVals = eig.eigenvalues();
    float sumEig = eigVals.sum();
    features.eigenRatios.resize(3);
    if (sumEig > 0.f) {
        for (int i = 0; i < 3; ++i)
            features.eigenRatios[i] = eigVals[i] / sumEig;
    } else {
        features.eigenRatios = {0.f, 0.f, 0.f};
    }

    // 10-11. 計算強度的平均值與標準差
    double sumI = 0, sumI2 = 0;
    for (const auto& pt : cloud->points) {
        sumI  += pt.intensity;
        sumI2 += pt.intensity * pt.intensity;
    }
    features.intensityMean   = static_cast<float>(sumI / features.pointCount);
    features.intensityStdDev = std::sqrt(sumI2 / features.pointCount - features.intensityMean * features.intensityMean);

    // 12. 局部密度：平均最近鄰距離
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    kdtree.setInputCloud(cloud);
    double sumNN = 0; int cntNN = 0;
    if (features.pointCount > 1) {
        for (size_t i = 0; i < cloud->size(); ++i) {
            std::vector<int> idx(2);
            std::vector<float> dist2(2);
            if (kdtree.nearestKSearch(cloud->points[i], 2, idx, dist2) > 0) {
                sumNN += std::sqrt(dist2[1]);
                cntNN++;
            }
        }
    }
    features.localDensity = (cntNN > 0) ? static_cast<float>(sumNN / cntNN) : 0.f;

    // 13. 計算主軸與垂直方向夾角
    pcl::PCA<pcl::PointXYZI> pca;
    pca.setInputCloud(cloud);
    Eigen::Vector3f pc = pca.getEigenVectors().col(0);
    float dotv = std::fabs(pc.dot(Eigen::Vector3f(0,0,1)));
    features.mainAxisAngle = std::acos(std::min(std::max(dotv, 0.f), 1.f)) * DEG_PER_RAD;

    return features;
}
