#include "OBBFittingProcessor.h"
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <iostream>

OrientedBoundingBox OBBFittingProcessor::computeOBB(const pcl::PointCloud<PointType>::Ptr &cloud_cluster)
{
    OrientedBoundingBox obb;
    if (cloud_cluster->empty())
    {
        std::cerr << "錯誤: 輸入點雲為空！" << std::endl;
        return obb;
    }
    // 1. 計算重心
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_cluster, centroid);

    // 2. 計算共變異數矩陣（需使用重心）
    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*cloud_cluster, centroid, covariance);

    // 3. PCA：計算特徵向量與特徵值
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectors = eigen_solver.eigenvectors();
    // 確保正交化：重新計算第三個軸
    eigenVectors.col(2) = eigenVectors.col(0).cross(eigenVectors.col(1));
    eigenVectors.col(0) = eigenVectors.col(1).cross(eigenVectors.col(2));
    eigenVectors.col(1) = eigenVectors.col(2).cross(eigenVectors.col(0));

    // 將特徵向量重新排序，使得第一軸對應最大特徵值（假設 eigen_solver 以遞增順序返回）
    Eigen::Matrix3f eigenVectorsOrdered;
    eigenVectorsOrdered.col(0) = eigenVectors.col(2);
    eigenVectorsOrdered.col(1) = eigenVectors.col(1);
    eigenVectorsOrdered.col(2) = eigenVectors.col(0);
    eigenVectors = eigenVectorsOrdered;

    // 4. 此處僅考慮繞 Z 軸的旋轉（取 yaw 分量）
    Eigen::Vector3f eulerAngles = eigenVectors.eulerAngles(2, 1, 0);
    float yaw = eulerAngles[0];
    Eigen::AngleAxisf zRot(yaw, Eigen::Vector3f::UnitZ());

    // 5. 建立轉換矩陣：先平移到重心，再繞 Z 軸旋轉
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translate(centroid.head<3>());
    transform.rotate(zRot);

    // 6. 對點雲進行逆向轉換，將其轉到局部座標系
    pcl::PointCloud<PointType>::Ptr transformedCloud(new pcl::PointCloud<PointType>);
    pcl::transformPointCloud(*cloud_cluster, *transformedCloud, transform.inverse());
    
    // 7. 取得局部座標系下的最小/最大值，進而得到尺寸與局部中心
    PointType min_pt, max_pt;
    pcl::getMinMax3D(*transformedCloud, min_pt, max_pt);
    // 根據我們假設的直立場景，轉換後點雲的 X 軸對應前進方向（長度），Y 軸對應左右方向（寬度），Z 軸對應高度
    float length = max_pt.x - min_pt.x;  // 前進方向範圍
    float width  = max_pt.y - min_pt.y;  // 左右方向範圍
    float height = max_pt.z - min_pt.z;  // 垂直方向範圍
    //取得局部中心
    Eigen::Vector3f local_center((max_pt.x + min_pt.x) / 2.0f,(max_pt.y + min_pt.y) / 2.0f,(max_pt.z + min_pt.z) / 2.0f);

    // 8. 將局部中心轉回全域座標系
    Eigen::Vector3f global_center = transform * local_center;

    // 9. 儲存結果
    obb.center = global_center;
    obb.dimensions = Eigen::Vector3f(length, width, height);
    obb.orientation = Eigen::Quaternionf(zRot); // 僅用 Z 軸旋轉表示

    return obb;
}
