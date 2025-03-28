#include "OBBFittingProcessor.h"
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <iostream>
#include <numeric>

OrientedBoundingBox OBBFittingProcessor::computeOBB(const pcl::PointCloud<PointType>::Ptr &cloud_cluster)
{
    OrientedBoundingBox obb;
    if (cloud_cluster->empty())
    {
        std::cerr << "錯誤: 輸入點雲為空！" << std::endl;
        return obb;
    }
    
    // 1. 計算原始重心
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_cluster, centroid);
    
    // 2. 計算每個點與重心的距離
    std::vector<float> distances;
    distances.reserve(cloud_cluster->points.size());
    for (const auto &pt : cloud_cluster->points)
    {
        float d = (pt.getVector3fMap() - centroid.head<3>()).norm();
        distances.push_back(d);
    }
    
    // 3. 計算平均值與標準差
    float sum = std::accumulate(distances.begin(), distances.end(), 0.0f);
    float mean = sum / distances.size();
    float sq_sum = std::inner_product(distances.begin(), distances.end(), distances.begin(), 0.0f);
    float stdev = std::sqrt(sq_sum / distances.size() - mean * mean);
    
    // 4. 定義距離閾值：僅保留距離小於 (mean + 1.5* stdev) 的點
    float threshold = mean + 1.5f * stdev;
    
    // 建立新的點雲，只保留離群點濾除後的點
    pcl::PointCloud<PointType>::Ptr filteredCloud(new pcl::PointCloud<PointType>);
    for (const auto &pt : cloud_cluster->points)
    {
        float d = (pt.getVector3fMap() - centroid.head<3>()).norm();
        if (d <= threshold)
            filteredCloud->points.push_back(pt);
    }
    
    // 若過濾後點太少，則使用原始點雲
    if (filteredCloud->empty())
    {
        filteredCloud = cloud_cluster;
    }
    else
    {
        pcl::compute3DCentroid(*filteredCloud, centroid);
    }
    
    // 5. 計算共變異數矩陣（使用過濾後的點雲）
    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*filteredCloud, centroid, covariance);
    
    // 6. PCA：計算特徵向量與特徵值
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectors = eigen_solver.eigenvectors();
    // 確保正交化：重新計算第三個軸
    eigenVectors.col(2) = eigenVectors.col(0).cross(eigenVectors.col(1));
    eigenVectors.col(0) = eigenVectors.col(1).cross(eigenVectors.col(2));
    eigenVectors.col(1) = eigenVectors.col(2).cross(eigenVectors.col(0));
    
    // 將特徵向量重新排序，使得第一軸對應最大特徵值
    Eigen::Matrix3f eigenVectorsOrdered;
    eigenVectorsOrdered.col(0) = eigenVectors.col(2);
    eigenVectorsOrdered.col(1) = eigenVectors.col(1);
    eigenVectorsOrdered.col(2) = eigenVectors.col(0);
    eigenVectors = eigenVectorsOrdered;
    
    // 7. 僅考慮繞 Z 軸的旋轉（取 yaw 分量）
    Eigen::Vector3f eulerAngles = eigenVectors.eulerAngles(2, 1, 0);
    float yaw = eulerAngles[0];
    Eigen::AngleAxisf zRot(yaw, Eigen::Vector3f::UnitZ());
    
    // 8. 建立轉換矩陣：平移到重心，再繞 Z 軸旋轉
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translate(centroid.head<3>());
    transform.rotate(zRot);
    
    // 9. 對過濾後的點雲進行逆向轉換，轉到局部座標系
    pcl::PointCloud<PointType>::Ptr transformedCloud(new pcl::PointCloud<PointType>);
    pcl::transformPointCloud(*filteredCloud, *transformedCloud, transform.inverse());
    
    // 10. 取得局部座標系下的最小/最大值，進而得到尺寸與局部中心
    PointType min_pt, max_pt;
    pcl::getMinMax3D(*transformedCloud, min_pt, max_pt);
    float length = max_pt.x - min_pt.x;  // X 軸範圍
    float width  = max_pt.y - min_pt.y;    // Y 軸範圍
    float height = max_pt.z - min_pt.z;    // Z 軸範圍
    
    Eigen::Vector3f local_center((max_pt.x + min_pt.x) / 2.0f,
                                 (max_pt.y + min_pt.y) / 2.0f,
                                 (max_pt.z + min_pt.z) / 2.0f);
    
    // 11. 將局部中心轉回全域座標系
    Eigen::Vector3f global_center = transform * local_center;
    
    // 12. 儲存結果
    obb.center = global_center;
    obb.dimensions = Eigen::Vector3f(length, width, height);
    obb.orientation = Eigen::Quaternionf(zRot); // 僅用 Z 軸旋轉表示
    
    return obb;
}