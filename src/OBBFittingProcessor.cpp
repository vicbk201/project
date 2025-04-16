
#include "OBBFittingProcessor.h"
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/common/centroid.h>
#include <pcl/common/eigen.h>
#include <iostream>
#include <cmath>

// 計算物體的有向包圍盒 (OBB)，可選擇性使用地面法向量校正點雲方向
OrientedBoundingBox OBBFittingProcessor::computeOBB(const pcl::PointCloud<PointType>::Ptr &cloud_cluster, const Eigen::Vector4f* ground_coeff_ptr)
{
    OrientedBoundingBox obb;
    if (cloud_cluster->empty())
    {
        std::cerr << "錯誤：輸入點雲為空！" << std::endl;
        return obb;
    }

    pcl::PointCloud<PointType>::Ptr cloud_aligned(new pcl::PointCloud<PointType>(*cloud_cluster));
    Eigen::Affine3f ground_correction = Eigen::Affine3f::Identity();

    if (ground_coeff_ptr)
    {
        Eigen::Vector3f ground_normal = ground_coeff_ptr->head<3>();
        if (ground_normal.norm() > 1e-6)
        {
            if (ground_normal.dot(Eigen::Vector3f::UnitZ()) < 0)
                ground_normal = -ground_normal;
            ground_normal.normalize();

            Eigen::Vector3f zAxis = Eigen::Vector3f::UnitZ();
            float angle = std::acos(ground_normal.dot(zAxis));
            Eigen::Vector3f axis = ground_normal.cross(zAxis);
            if (axis.norm() < 1e-6)
            {
                axis = Eigen::Vector3f::UnitX();
                angle = 0.0f;
            }
            else
            {
                axis.normalize();
            }
            ground_correction.rotate(Eigen::AngleAxisf(angle, axis));
            pcl::transformPointCloud(*cloud_cluster, *cloud_aligned, ground_correction);
        }
    }
    
    // 1. 計算點雲的幾何重心與中心
    // 重心：所有點的平均值 (用於計算協方差矩陣)
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_aligned, centroid);
    
    // 中心：點雲在各軸上最小值與最大值的平均值 (AABB 中心)
    PointType min_pt, max_pt;
    Eigen::Vector3f center;
    pcl::getMinMax3D(*cloud_aligned, min_pt, max_pt);
    center = (min_pt.getVector3fMap() + max_pt.getVector3fMap()) / 2.0f;
    
    // 2. 計算協方差矩陣 (必須使用重心)
    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*cloud_aligned, centroid, covariance);
    
    // 進行 PCA 計算特徵向量與特徵值
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();

    // 確保正交化：重新計算第三個軸
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
    eigenVectorsPCA.col(0) = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
    eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0));

    // 按照特徵值從大到小排列（預設 eigen_solver 返回的特徵向量是從小到大排列的）
    Eigen::Matrix3f eigenVectorsOrdered;
    eigenVectorsOrdered.col(0) = eigenVectorsPCA.col(2);
    eigenVectorsOrdered.col(1) = eigenVectorsPCA.col(1);
    eigenVectorsOrdered.col(2) = eigenVectorsPCA.col(0);
    eigenVectorsPCA = eigenVectorsOrdered;

    // 3. 計算變換矩陣：只考慮繞全局 Z 軸的旋轉（即只取 yaw）
    // 取得 Euler 角 (yaw, pitch, roll
    Eigen::Vector3f eulerAngles = eigenVectorsPCA.eulerAngles(2, 1, 0);
    float yaw = eulerAngles[0];
    // 只保留繞 Z 軸旋轉的部分
    Eigen::AngleAxisf keep_Z_Rot(yaw, Eigen::Vector3f::UnitZ());

    // 建立變換矩陣：先平移到「中心」(AABB 中心)，再旋轉 (順序不能顛倒)
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translate(center);
    transform.rotate(keep_Z_Rot);

    // 4. 將原始點雲轉到局部坐標系中，計算局部下的最小/最大值以得到包圍盒尺寸
    pcl::PointCloud<PointType>::Ptr transformedCloud(new pcl::PointCloud<PointType>);
    pcl::transformPointCloud(*cloud_aligned, *transformedCloud, transform.inverse());

    PointType min_pt_T, max_pt_T;
    pcl::getMinMax3D(*transformedCloud, min_pt_T, max_pt_T);
    float length = max_pt_T.x - min_pt_T.x;
    float width  = max_pt_T.y - min_pt_T.y;
    float height = max_pt_T.z - min_pt_T.z;

    // 計算局部中心 (局部 AABB 中心)
    Eigen::Vector3f local_center = (min_pt_T.getVector3fMap() + max_pt_T.getVector3fMap()) / 2.0f;
    // 5. 將局部中心轉回全局坐標
    Eigen::Vector3f global_center = transform * local_center;

    // 新增：依據局部 AABB 範圍篩選點雲，使 localCloud 僅包含位於 [min_pt_T, max_pt_T] 內的點
    pcl::PointCloud<PointType>::Ptr localCloudFiltered(new pcl::PointCloud<PointType>);

    for (const auto &pt : transformedCloud->points)
    {
        if (pt.x >= min_pt_T.x && pt.x <= max_pt_T.x &&
            pt.y >= min_pt_T.y && pt.y <= max_pt_T.y &&
            pt.z >= min_pt_T.z && pt.z <= max_pt_T.z)
        {
            localCloudFiltered->points.push_back(pt);
        }
    }

    
    // 5.將局部中心轉回全局 + 6.儲存結果
    obb.center = ground_correction.inverse() * global_center;
    obb.orientation = Eigen::Quaternionf(ground_correction.inverse().rotation()) 
                      * Eigen::Quaternionf(keep_Z_Rot);
    obb.dimensions = Eigen::Vector3f(length, width, height);


    // ========== 在這裡把 transformedCloud 存進去 ==========
    obb.localCloud = transformedCloud;

    return obb;
}   
