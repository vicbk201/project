// FeatureExporter.cpp
#include "FeatureExporter.h"
#include <fstream>
#include <iostream>
#include <iomanip>
#include <filesystem>

void FeatureExporter::exportToCSV(
    const std::vector<std::pair<int, ClusterFeatures>>& featuresList,
    const std::string& csvPath,
    const std::set<int>& positiveSet)
{
    namespace fs = std::filesystem;
    bool exists = fs::exists(csvPath);
    // 建立目錄(若需要)
    fs::create_directories(fs::path(csvPath).parent_path());
    // 以 append 模式開啟檔案，若不存在則建立新檔
    std::ofstream file(csvPath, exists ? std::ios::app : std::ios::out);
    if (!file.is_open()) {
        std::cerr << "Error: 無法寫入 CSV 檔案 " << csvPath << std::endl;
        return;
    }

    // 若為新檔才寫標頭
    if (!exists) {
        file << "ClusterLabel,PointCount,CentroidDistance,AverageDistance,";
        file << "Cov_xx,Cov_xy,Cov_xz,Cov_yx,Cov_yy,Cov_yz,Cov_zx,Cov_zy,Cov_zz,";
        file << "Moment_xx,Moment_yy,Moment_zz,Moment_xy,Moment_xz,Moment_yz,";
        file << "OBB_Length,OBB_Width,OBB_Height,LW_Ratio,";
        file << "Eigen_0,Eigen_1,Eigen_2,";
        file << "IntensityMean,IntensityStdDev,LocalDensity,MainAxisAngle,IsHuman\n";
    }

    for (const auto& item : featuresList) {
        int label = item.first;
        const ClusterFeatures& f = item.second;
        int isHuman = positiveSet.count(label) ? 1 : 0;

        file << label << ","
             << f.pointCount << ","
             << f.centroidDistance << ","
             << f.averageDistance << ",";

        // 共變異數：xx,xy,xz,yx,yy,yz,zx,zy,zz
        file << f.covarianceMatrix[0] << "," << f.covarianceMatrix[1] << "," << f.covarianceMatrix[2] << ","
             << f.covarianceMatrix[3] << "," << f.covarianceMatrix[4] << "," << f.covarianceMatrix[5] << ","
             << f.covarianceMatrix[6] << "," << f.covarianceMatrix[7] << "," << f.covarianceMatrix[8] << ",";
        // 矩量：xx,yy,zz,xy,xz,yz
        file << f.normalizedMoments[0] << "," << f.normalizedMoments[1] << "," << f.normalizedMoments[2] << ","
             << f.normalizedMoments[3] << "," << f.normalizedMoments[4] << "," << f.normalizedMoments[5] << ",";

        file << f.obb_length << "," << f.obb_width << "," << f.obb_height << "," << f.lw_ratio << ",";

        file << f.eigenRatios[0] << "," << f.eigenRatios[1] << "," << f.eigenRatios[2] << ",";

        file << f.intensityMean << "," << f.intensityStdDev << "," << f.localDensity << ","
             << f.mainAxisAngle << "," << isHuman << "\n";
    }

    file.close();
    std::cout << "✅ 特徵已輸出至: " << csvPath << (exists ? " (已追加)" : " (新檔)") << std::endl;
}
