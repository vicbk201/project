#include "GroundRemovalProcessor.h"
#include <pcl/console/time.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <iostream>
#include <iomanip>

typedef pcl::PointXYZI PointTypeIO;

// 使用 RANSAC 自動擬合地面平面
GroundRemovalResult GroundRemovalProcessor::removeGround(
    const pcl::PointCloud<PointTypeIO>::Ptr &cloud)
{
    pcl::console::TicToc tt;
    tt.tic();

    pcl::PointCloud<PointTypeIO>::Ptr cloud_out(new pcl::PointCloud<PointTypeIO>(*cloud));
    pcl::PointCloud<PointTypeIO>::Ptr ground_cloud(new pcl::PointCloud<PointTypeIO>);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::SACSegmentation<PointTypeIO> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.08); // 可依點雲密度與雜訊微調

    int max_iterations = 5;
    const int min_inliers_threshold = 100;
    int iteration = 0;
    const float angle_threshold = 15.0f * M_PI / 180.0f; // 15度角容忍度

    Eigen::Vector4f ground_coeff(0, 0, 0, 0);
    bool groundFound = false;

    while (iteration < max_iterations)
    {
        seg.setInputCloud(cloud_out);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.empty() || inliers->indices.size() < min_inliers_threshold)
            break;

        Eigen::Vector3f current_normal(coefficients->values[0],
                                       coefficients->values[1],
                                       coefficients->values[2]);
        current_normal.normalize();

        if (!groundFound)
        {
            // 檢查此平面是否為近似水平面（Z 軸）
            if (std::fabs(current_normal.dot(Eigen::Vector3f::UnitZ())) < 0.9f)
                break; // 角度偏差太大

            ground_coeff << coefficients->values[0], coefficients->values[1],
                            coefficients->values[2], coefficients->values[3];
            groundFound = true;
        }
        else
        {
            // 後續擬合平面需與首次地面平面一致
            Eigen::Vector3f ground_normal = ground_coeff.head<3>().normalized();
            float angle = std::acos(current_normal.dot(ground_normal));
            if (angle > angle_threshold)
                break;
        }

        pcl::ExtractIndices<PointTypeIO> extract;
        extract.setInputCloud(cloud_out);
        extract.setIndices(inliers);

        // 提取地面點
        pcl::PointCloud<PointTypeIO>::Ptr current_ground(new pcl::PointCloud<PointTypeIO>());
        extract.setNegative(false);
        extract.filter(*current_ground);
        *ground_cloud += *current_ground;

        // 剩下的保留非地面點
        extract.setNegative(true);
        pcl::PointCloud<PointTypeIO>::Ptr cloud_no_ground(new pcl::PointCloud<PointTypeIO>());
        extract.filter(*cloud_no_ground);
        cloud_out = cloud_no_ground;

        iteration++;
    }

    double elapsed = tt.toc();

    GroundRemovalResult result;
    result.cloud = cloud_out;
    result.ground = ground_cloud;
    result.runtime_ms = elapsed;
    result.ground_coefficients = ground_coeff;
    return result;
}

// 使用指定的平面係數與距離閾值來分類地面點（固定平面方式）
GroundRemovalResult GroundRemovalProcessor::removeGroundWithPlane(
    const pcl::PointCloud<PointTypeIO>::Ptr &cloud,
    const pcl::ModelCoefficients::Ptr &plane_coeff,
    float threshold)
{
    pcl::console::TicToc tt;
    tt.tic();

    pcl::PointCloud<PointTypeIO>::Ptr cloud_out(new pcl::PointCloud<PointTypeIO>);
    pcl::PointCloud<PointTypeIO>::Ptr ground_cloud(new pcl::PointCloud<PointTypeIO>);

    // 平面方程式：Ax + By + Cz + D = 0
    float A = plane_coeff->values[0];
    float B = plane_coeff->values[1];
    float C = plane_coeff->values[2];
    float D = plane_coeff->values[3];

    float norm = std::sqrt(A*A + B*B + C*C);

    for (const auto &pt : cloud->points)
    {
        // 計算「有符號」距離
        float signed_d = (A*pt.x + B*pt.y + C*pt.z + D) / norm;

        // 如果在平面以下（signed_d<=0） 或 在平面上方但離地面<閾值（0<signed_d<=threshold）
        if (signed_d <= threshold)
        {
            ground_cloud->points.push_back(pt);  // 當地面／雜訊移除
        }
        else
        {
            cloud_out->points.push_back(pt);     // 真正高於閾值的點才保留
        }
    }
    
    /*
    for (const auto &pt : cloud->points)
    {
        float distance = std::fabs(A * pt.x + B * pt.y + C * pt.z + D) / norm;
        if (distance < threshold)
        {
            ground_cloud->points.push_back(pt); // 被視為地面
        }
        else
        {
            cloud_out->points.push_back(pt);    // 非地面
        }
    }
    */

    cloud_out->width = static_cast<uint32_t>(cloud_out->points.size());
    cloud_out->height = 1;
    cloud_out->is_dense = true;

    ground_cloud->width = static_cast<uint32_t>(ground_cloud->points.size());
    ground_cloud->height = 1;
    ground_cloud->is_dense = true;

    double elapsed = tt.toc();
    

    GroundRemovalResult result;
    result.cloud = cloud_out;
    result.ground = ground_cloud;
    result.runtime_ms = elapsed;
    result.ground_coefficients = Eigen::Vector4f(A, B, C, D);
    return result;
}
