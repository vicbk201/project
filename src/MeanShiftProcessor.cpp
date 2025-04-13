#include "MeanShiftProcessor.h"
#include <pcl/console/time.h>
#include <cmath>
#include <map>
#include <queue>
#include <iostream>

std::vector<int> MeanShiftProcessor::runMeanShift2D(
    const std::vector<Eigen::Vector2f> &points2D,
    float bandwidth,
    int maxIter)
{
    std::vector<int> labels(points2D.size(), -1);
    std::vector<Eigen::Vector2f> modes; // 收斂峰值

    for (size_t i = 0; i < points2D.size(); ++i)
    {
        Eigen::Vector2f current = points2D[i];
        for (int iter = 0; iter < maxIter; ++iter)
        {
            Eigen::Vector2f numerator(0,0);
            float denominator = 0.0f;
            // 在帶寬 bandwidth 內做均值
            for (size_t j = 0; j < points2D.size(); ++j)
            {
                float dist = (points2D[j] - current).norm();
                if (dist < bandwidth)
                {
                    numerator += points2D[j];
                    denominator += 1.0f;
                }
            }
            if (denominator > 0.0f)
            {
                Eigen::Vector2f newCenter = numerator / denominator;
                if ((newCenter - current).norm() < 1e-3f)
                {
                    // 收斂
                    current = newCenter;
                    break;
                }
                current = newCenter;
            }
            else
            {
                // 沒鄰居 => 不動
                break;
            }
        }
        // 找到最終峰值 current，匹配看看是否跟 modes 的某個峰值靠近
        float mergeDist = bandwidth * 0.5f;
        int foundLabel = -1;
        for (size_t m = 0; m < modes.size(); ++m)
        {
            if ((modes[m] - current).norm() < mergeDist)
            {
                foundLabel = (int)m;
                break;
            }
        }
        if (foundLabel == -1)
        {
            foundLabel = (int)modes.size();
            modes.push_back(current);
        }
        labels[i] = foundLabel;
    }
    return labels;
}

std::vector<int> MeanShiftProcessor::separateClosePedestriansByLocalCloud(
    const OrientedBoundingBox &obb,
    float bandwidth,
    int minPoints,
    int newBaseLabel)
{
    auto localCloud = obb.localCloud;
    if (!localCloud || localCloud->empty()) {
        // 這個 cluster 可能是空的
        return {};
    }

    // 將 localCloud 投影到 2D => (x, y)
    std::vector<Eigen::Vector2f> points2D;
    points2D.reserve(localCloud->size());
    for (auto &pt : localCloud->points)
    {
        points2D.push_back(Eigen::Vector2f(pt.x, pt.y));
    }

    // 執行 MeanShift
    std::vector<int> subLabels = runMeanShift2D(points2D, bandwidth, 50);

    // 重新編碼 subLabels => subLabelRemap
    std::map<int, int> subRemap; 
    int nextID = 0;
    for (auto &sl : subLabels)
    {
        if (subRemap.find(sl) == subRemap.end())
        {
            subRemap[sl] = nextID++;
        }
    }
    // 計算每個子群大小
    std::vector<int> subCounts(nextID, 0);
    for (auto &sl : subLabels)
    {
        subCounts[subRemap[sl]]++;
    }

    // 真正的 label = newBaseLabel + subRemap[sl]
    // 如果該子群 < minPoints => 可視為噪點或合併
    // 返回一個對應 localCloud->points 的 "最終子標籤"
    std::vector<int> finalLabels(localCloud->size(), -1);
    for (size_t i = 0; i < localCloud->size(); ++i)
    {
        int sub = subRemap[subLabels[i]];
        if (subCounts[sub] < minPoints) {
            finalLabels[i] = -2; // 視為噪點
        } else {
            finalLabels[i] = (newBaseLabel + sub);
        }
    }
    return finalLabels;
}
