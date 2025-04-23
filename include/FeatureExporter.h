// FeatureExporter.h
#ifndef FEATURE_EXPORTER_H
#define FEATURE_EXPORTER_H

#include <vector>
#include <string>
#include <set>
#include "FeatureExtractionProcessor.h"  // 定義 ClusterFeatures

class FeatureExporter {
public:
    /**
     * 將一組 (label, 特徵) 輸出到 CSV。
     * @param featuresList  每筆資料對：first=ClusterLabel, second=ClusterFeatures
     * @param csvPath       輸出檔案路徑
     * @param labelSet      若非空，只有此集合內的 label 會被打上 LabelFlag=1
     */
    static void exportToCSV(const std::vector<std::pair<int, ClusterFeatures>>& featuresList,
                            const std::string& csvPath,
                            const std::set<int>& labelSet = {});
};

#endif // FEATURE_EXPORTER_H