#include "GroundRemovalProcessor.h"
#include <pcl/console/time.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/point_types.h>
#include <iostream>
#include <iomanip>

typedef pcl::PointXYZI PointTypeIO;

GroundRemovalResult GroundRemovalProcessor::removeGround(
    const pcl::PointCloud<PointTypeIO>::Ptr &cloud)
{
    pcl::console::TicToc tt;
    tt.tic();

    pcl::PointCloud<PointTypeIO>::Ptr cloud_out(new pcl::PointCloud<PointTypeIO>(*cloud));

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::SACSegmentation<PointTypeIO> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.3); // 根據點雲密度與感測器噪聲調整

    int max_iterations = 3;
    const int min_inliers_threshold = 100;
    int iteration = 0;
    while (iteration < max_iterations)
    {
        seg.setInputCloud(cloud_out);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.empty() || inliers->indices.size() < min_inliers_threshold)
        {
            break;
        }
        pcl::ExtractIndices<PointTypeIO> extract;
        extract.setInputCloud(cloud_out);
        extract.setIndices(inliers);
        extract.setNegative(true); // 移除 inliers
        pcl::PointCloud<PointTypeIO>::Ptr cloud_no_ground(new pcl::PointCloud<PointTypeIO>());
        extract.filter(*cloud_no_ground);
        cloud_out = cloud_no_ground;
        iteration++;
    }

    double elapsed = tt.toc();

    GroundRemovalResult result;
    result.cloud = cloud_out;
    result.runtime_ms = elapsed;
    return result;
}
