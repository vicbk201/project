#ifndef OCTREE_PROCESSOR_H
#define OCTREE_PROCESSOR_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/**
 * OctreeProcessor 使用自適應八叉樹的多層次與自適應特性來對點雲進行降採樣，
 * 使得在點密集區域保留更多細節，而點稀疏區域則採用較粗略的表示。
 *
 * 參數 resolution 用於設定最終期望的解析度（會根據點雲邊界計算出適當的最大深度）。
 */
class OctreeProcessor {
public:
    static pcl::PointCloud<pcl::PointXYZ>::Ptr processOctree(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, float resolution);
};

#endif // OCTREE_PROCESSOR_H
