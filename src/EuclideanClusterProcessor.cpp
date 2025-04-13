#include "EuclideanClusterProcessor.h"
#include <pcl/common/io.h>
#include <pcl/console/time.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <cstdlib>
#include <cmath>

typedef pcl::PointXYZI PointTypeIO;
typedef pcl::PointXYZINormal PointTypeFull;

bool customRegionGrowing (const PointTypeFull& point_a, const PointTypeFull& point_b, float squared_distance)
{
    Eigen::Map<const Eigen::Vector3f> point_a_normal = point_a.getNormalVector3fMap(),
                                     point_b_normal = point_b.getNormalVector3fMap();
    if (squared_distance < 10000)
    {
        if (std::abs(point_a.intensity - point_b.intensity) < 8.0f)
            return true;
        if (std::abs(point_a_normal.dot(point_b_normal)) > std::cos(30.0f / 180.0f * static_cast<float>(M_PI)))
            return true;
    }
    else
    {
        if (std::abs(point_a.intensity - point_b.intensity) < 3.0f)
            return true;
    }
    return false;
}

EuclideanClusteringResult EuclideanClusterProcessor::clusterCloud(
    const pcl::PointCloud<PointTypeIO>::Ptr &cloud,
    double clusterTolerance)
{
    pcl::console::TicToc tt;
    tt.tic();
    
    // 1. 法向量估計：先將 cloud 複製為含法向量的 cloud_with_normals
    pcl::PointCloud<PointTypeFull>::Ptr cloud_with_normals(new pcl::PointCloud<PointTypeFull>);
    pcl::copyPointCloud(*cloud, *cloud_with_normals);
    
    pcl::NormalEstimation<PointTypeIO, PointTypeFull> ne;
    typename pcl::search::KdTree<PointTypeIO>::Ptr searchTree(new pcl::search::KdTree<PointTypeIO>);
    ne.setInputCloud(cloud);
    ne.setSearchMethod(searchTree);
    ne.setRadiusSearch(0.1);
    ne.compute(*cloud_with_normals);
    
    // 2. Conditional Euclidean Clustering
    pcl::IndicesClustersPtr clusters(new pcl::IndicesClusters);
    pcl::IndicesClustersPtr small_clusters(new pcl::IndicesClusters);
    pcl::IndicesClustersPtr large_clusters(new pcl::IndicesClusters);
    
    pcl::ConditionalEuclideanClustering<PointTypeFull> cec(true);
    cec.setInputCloud(cloud_with_normals);
    cec.setConditionFunction(&customRegionGrowing);
    cec.setClusterTolerance(clusterTolerance);
    cec.setMinClusterSize(50);
    cec.setMaxClusterSize(10000);
    cec.segment(*clusters);
    cec.getRemovedClusters(small_clusters, large_clusters);

    // 3. 使用 intensity 欄位進行視覺化：過小的叢集標記為 -2，過大的叢集標記為 +10，
    for (const auto& small_cluster : (*small_clusters))
        for (const auto& j : small_cluster.indices)
            (*cloud)[j].intensity = -2.0;
    for (const auto& large_cluster : (*large_clusters))
        for (const auto& j : large_cluster.indices)
            (*cloud)[j].intensity = +10.0;
    
    int label = 0;
    for (const auto& cluster : (*clusters))
    {
    // 為每個聚類分配一個唯一的標籤
    for (const auto& j : cluster.indices)
         (*cloud)[j].intensity = label;
    label++;
    }

    
    double elapsed = tt.toc();
    
    EuclideanClusteringResult result;
    result.cloud = cloud;
    result.runtime_ms = elapsed;
    return result;
}
