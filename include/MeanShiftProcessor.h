#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "OBBFittingProcessor.h"

/**
 * 用於二次分離多行人
 */
struct MeanShiftResult
{
    // 分離後更新 intensity 的點雲
    pcl::PointCloud<pcl::PointXYZI>::Ptr separatedCloud;
    double runtime_ms = 0.0;
};

class MeanShiftProcessor
{
public:
    /**
     * 針對給定 cluster 的 "局部點雲" (obb.localCloud)，檢查是否要二次分群。
     * @param obb: 已算好的 OBB (包含 localCloud)
     * @param bandwidth: Mean Shift 在 2D 上的核帶寬
     * @param minPoints: 分離子群小於此值就忽略或當噪點
     * @param newBaseLabel: 產生新的 cluster label 起始值
     * 
     * @return : (subLabels, newMaxLabel) 或你想要的資訊
     */
    static std::vector<int> separateClosePedestriansByLocalCloud(
        const OrientedBoundingBox &obb,
        float bandwidth,
        int minPoints,
        int newBaseLabel
    );

    /**
     * 這是簡化函式: 把 (x, y) 拿去 Mean Shift，回傳一個與該 cluster 各點對應的子群標籤
     * @param localPoints2D: 投影後的2D座標
     * @param bandwidth: Mean Shift半徑
     * @param maxIter: 迭代次數
     */
    static std::vector<int> runMeanShift2D(
        const std::vector<Eigen::Vector2f> &localPoints2D,
        float bandwidth,
        int maxIter = 50
    );
};