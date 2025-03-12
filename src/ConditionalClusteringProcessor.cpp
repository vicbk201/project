
#include "ConditionalClusteringProcessor.h"
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

ClusteringResult ConditionalClusteringProcessor::clusterCloud(
    const pcl::PointCloud<PointTypeIO>::Ptr &cloud)
{
    pcl::console::TicToc tt;  // 開始計時
    tt.tic();
    
    // 1. 法向量估計：先將 cloud_in 複製為 PointTypeFull
    pcl::PointCloud<PointTypeFull>::Ptr cloud_with_normals(new pcl::PointCloud<PointTypeFull>);
    pcl::copyPointCloud(*cloud, *cloud_with_normals);

    pcl::NormalEstimation<PointTypeIO, PointTypeFull> ne;
    typename pcl::search::KdTree<PointTypeIO>::Ptr searchTree(new pcl::search::KdTree<PointTypeIO>);
    ne.setInputCloud(cloud);
    ne.setSearchMethod(searchTree);
    ne.setRadiusSearch(0.3);
    ne.compute(*cloud_with_normals);

    // 2. Conditional Euclidean Clustering
    pcl::IndicesClustersPtr clusters(new pcl::IndicesClusters);
    pcl::IndicesClustersPtr small_clusters(new pcl::IndicesClusters);
    pcl::IndicesClustersPtr large_clusters(new pcl::IndicesClusters);
    pcl::ConditionalEuclideanClustering<PointTypeFull> cec(true);
    cec.setInputCloud(cloud_with_normals);
    cec.setConditionFunction(&customRegionGrowing);
    cec.setClusterTolerance(0.5);
    cec.setMinClusterSize(cloud_with_normals->size() / 1000);
    cec.setMaxClusterSize(cloud_with_normals->size() / 5);
    cec.segment(*clusters);
    cec.getRemovedClusters(small_clusters, large_clusters);
   
    // 使用 intensity 通道進行簡易視覺化：
    // 先設定太小的簇與太大的簇
    for (const auto& small_cluster : (*small_clusters))
      for (const auto& j : small_cluster.indices)
        (*cloud)[j].intensity = -2.0;
    for (const auto& large_cluster : (*large_clusters))
      for (const auto& j : large_cluster.indices)
        (*cloud)[j].intensity = +10.0;

    // 其他群集則使用隨機值（例如 0~7 之間）讓檢視器依 intensity 自行映射顏色
    for (const auto& cluster : (*clusters))
    {
      int label = rand() % 8;
      for (const auto& j : cluster.indices)
        (*cloud)[j].intensity = label;
    }

    double elapsed = tt.toc();  // 算法耗時 (毫秒)

    ClusteringResult result;
    result.cloud = cloud;
    result.runtime_ms = elapsed;
    return result;
}
