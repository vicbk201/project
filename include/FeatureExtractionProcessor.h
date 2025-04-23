// FeatureExtractionProcessor.h
#ifndef FEATURE_EXTRACTION_PROCESSOR_H
#define FEATURE_EXTRACTION_PROCESSOR_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <Eigen/Dense>
#include <vector>
#include "OBBFittingProcessor.h"

// 封裝單一 cluster 的特徵
struct ClusterFeatures
{
    int pointCount;                           // 1. 點數量
    float centroidDistance;                   // 2. 質心到 LiDAR 的距離
    float averageDistance;                    // 3. 所有點到 LiDAR 的平均距離
    std::vector<float> covarianceMatrix;      // 4. 3x3 協方差矩陣 (9 個元素)
    std::vector<float> normalizedMoments;     // 5. 歸一化慣性張量 (6 個分量: Ixx, Iyy, Izz, Ixy, Ixz, Iyz)
    float obb_length;                         // 6. OBB 長度
    float obb_width;                          // 7. OBB 寬度
    float obb_height;                         // 8. OBB 高度
    float lw_ratio;                           // 9. 長寬比 (Length / Width)
    std::vector<float> eigenRatios;           // 10. PCA 特徵值比例 (三個值，總和為 1)
    float intensityMean;                      // 11. 強度平均值
    float intensityStdDev;                    // 12. 強度標準差
    float localDensity;                       // 13. 平均最近鄰距離 (局部密度)
    float mainAxisAngle;                      // 14. 第一主成分方向與垂直方向夾角 (度)
};

class FeatureExtractionProcessor
{
public:
    /**
     * @brief 計算 cluster 的各項特徵
     * @param cloud         輸入的點雲 (XYZI)
     * @param obb           已計算的 Oriented Bounding Box
     * @param lidarPosition LiDAR 安裝位置 (用於距離計算)
     * @return ClusterFeatures 返回所有特徵
     */
    static ClusterFeatures computeFeatures(
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud,
        const OrientedBoundingBox &obb,
        const pcl::PointXYZ &lidarPosition);
};

#endif // FEATURE_EXTRACTION_PROCESSOR_H