// FeatureExporter.h
#ifndef FEATURE_EXPORTER_H
#define FEATURE_EXPORTER_H

#include <vector>
#include <set>
#include <string>
#include <utility>
#include "FeatureExtractionProcessor.h"  // for ClusterFeatures

class FeatureExporter {
public:
    /**
     * @param featuresList 每個 pair.first = cluster label, pair.second = 計算好的特徵
     * @param csvPath      最終輸出 csv 的完整路徑（可包含 data/ 子目錄）
     * @param positiveSet  這些 label 將被標記為 IsHuman=1，其餘為 0
     */
    // 新的：前面多一個 filename 欄位
    static void exportToCSV(
        const std::vector<std::tuple<std::string,int,ClusterFeatures>>& featuresList,
        const std::string& csvPath,
        const std::set<int>& positiveSet);
};

#endif // FEATURE_EXPORTER_H