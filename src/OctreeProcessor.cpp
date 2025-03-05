#include "OctreeProcessor.h"
#include <pcl/common/common.h>
#include <Eigen/Dense>
#include <vector>
#include <numeric>
#include <cmath>
#include <functional>
#include <iostream>

pcl::PointCloud<pcl::PointXYZ>::Ptr OctreeProcessor::processOctree(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, float resolution)
{
    // 計算點雲邊界
    Eigen::Vector4f minPt, maxPt;
    pcl::getMinMax3D(*cloud, minPt, maxPt);

    // 根據邊界最大尺寸計算最大層次 (maxDepth)，使最終解析度接近參數 resolution
    float maxDim = (maxPt - minPt).head<3>().maxCoeff();
    int maxDepth = static_cast<int>(std::ceil(std::log2(maxDim / resolution)));

    // 先印出計算的 maxDepth
    std::cout << "[OctreeProcessor] Calculated maxDepth = " << maxDepth << std::endl;

    // 記錄每層節點數量的容器 (0 ~ maxDepth 共 maxDepth+1 層)
    std::vector<int> nodeCountByLevel(maxDepth + 1, 0);

    // 可調整的密度閾值，決定節點中若超過多少點才繼續細分
    int densityThreshold = 50;

    // 定義一個節點結構，表示八叉樹中的一個區域
    struct Node {
        Eigen::Vector4f min_bound, max_bound;
        std::vector<int> indices;   // 區域內的點索引
        int level;                  // 當前層次
        std::vector<Node*> children; 
        Eigen::Vector3f centroid;   // 區域質心
        bool isLeaf;               // 是否為葉節點

        Node(const Eigen::Vector4f& min_b, const Eigen::Vector4f& max_b, int lvl)
            : min_bound(min_b), max_bound(max_b), level(lvl), isLeaf(true)
        {
            children.resize(8, nullptr);
        }
    };

    // 遞歸建立八叉樹的函式
    std::function<Node*(const std::vector<int>&, const Eigen::Vector4f&, const Eigen::Vector4f&, int)> buildTree;
    buildTree = [&](const std::vector<int>& idx, const Eigen::Vector4f& min_b, 
                    const Eigen::Vector4f& max_b, int lvl) -> Node* 
    {
        Node* node = new Node(min_b, max_b, lvl);
        // 統計當前層級的節點數量
        nodeCountByLevel[lvl]++;

        // 存下該區域內所有點的索引
        node->indices = idx;

        // 計算該區域的質心
        Eigen::Vector3f sum = Eigen::Vector3f::Zero();
        for (int i : idx) {
            sum += cloud->points[i].getVector3fMap();
        }
        if (!idx.empty()) {
            node->centroid = sum / static_cast<float>(idx.size());
        } else {
            node->centroid = Eigen::Vector3f::Zero();
        }

        // 若未達最大深度且該區域點數超過閾值，則繼續細分
        if (lvl < maxDepth && idx.size() > static_cast<size_t>(densityThreshold)) {
            node->isLeaf = false;
            // 計算當前區域的中點
            Eigen::Vector4f mid = (min_b + max_b) * 0.5f;

            // 根據中點將點分到 8 個子區域
            std::vector<int> child_indices[8];
            for (int i : idx) {
                const auto &pt = cloud->points[i];
                int childIdx = 0;
                if (pt.x > mid[0]) childIdx |= 1;
                if (pt.y > mid[1]) childIdx |= 2;
                if (pt.z > mid[2]) childIdx |= 4;
                child_indices[childIdx].push_back(i);
            }

            // 為每個非空子區域建立子節點
            for (int c = 0; c < 8; c++) {
                if (!child_indices[c].empty()) {
                    Eigen::Vector4f child_min = min_b;
                    Eigen::Vector4f child_max = max_b;
                    if (c & 1) child_min[0] = mid[0]; else child_max[0] = mid[0];
                    if (c & 2) child_min[1] = mid[1]; else child_max[1] = mid[1];
                    if (c & 4) child_min[2] = mid[2]; else child_max[2] = mid[2];

                    node->children[c] = buildTree(child_indices[c], child_min, child_max, lvl + 1);
                }
            }
        }

        return node;
    };

    // 建立八叉樹
    std::vector<int> allIndices(cloud->points.size());
    std::iota(allIndices.begin(), allIndices.end(), 0);
    Node* root = buildTree(allIndices, minPt, maxPt, 0);

    // 建立最終下採樣後的點雲
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampledCloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 遞歸遍歷八叉樹，蒐集代表點
    std::function<void(Node*)> traverseTree = [&](Node* node) {
        if (!node) return;
        if (node->isLeaf || node->indices.size() <= static_cast<size_t>(densityThreshold)) {
            pcl::PointXYZ pt;
            pt.x = node->centroid[0];
            pt.y = node->centroid[1];
            pt.z = node->centroid[2];
            downsampledCloud->points.push_back(pt);
        } else {
            for (Node* child : node->children) {
                traverseTree(child);
            }
        }
    };
    traverseTree(root);

    // 統計完成後，印出每個層級的節點數
    for (int d = 0; d <= maxDepth; d++) {
        std::cout << "[OctreeProcessor] Level " << d 
                  << " node count: " << nodeCountByLevel[d] << std::endl;
    }

    // 設定點雲維度
    downsampledCloud->width  = static_cast<uint32_t>(downsampledCloud->points.size());
    downsampledCloud->height = 1;
    downsampledCloud->is_dense = true;

    // 釋放動態配置的節點
    std::function<void(Node*)> deleteTree = [&](Node* node) {
        if (!node) return;
        for (Node* child : node->children) {
            deleteTree(child);
        }
        delete node;
    };
    deleteTree(root);

    return downsampledCloud;
}
