// FeatureExporter.cpp
#include "FeatureExporter.h"
#include <fstream>
#include <iostream>
#include <filesystem>
#include <iomanip>
#include <tuple>              


void FeatureExporter::exportToCSV(
    const std::vector<std::tuple<std::string,int,ClusterFeatures>>& featuresList,
    const std::string& csvPath,
    const std::set<int>& positiveSet)
{
    namespace fs = std::filesystem;
    fs::path path(csvPath);

    // 1. 如果父目錄不存在，先建立之
    fs::create_directories(path.parent_path());

    // 2. 檢查 CSV 檔案是否已存在
    bool exists = fs::exists(path);
    // 3. 以 append 模式開啟，若不存在則以 out 建檔
    std::ofstream file(csvPath, exists ? std::ios::app : std::ios::out);
    if (!file.is_open()) {
        std::cerr << "Error: 無法寫入 CSV 檔案 " << csvPath << std::endl;
        return;
    }

    // 4. 新檔才需要寫入欄位標頭
    if (!exists) {
        file << "Filename,ClusterLabel,PointCount,CentroidDistance,AverageDistance,"
             << "Cov_xx,Cov_xy,Cov_xz,Cov_yx,Cov_yy,Cov_yz,Cov_zx,Cov_zy,Cov_zz,"
             << "Moment_xx,Moment_yy,Moment_zz,Moment_xy,Moment_xz,Moment_yz,"
             << "OBB_Length,OBB_Width,OBB_Height,LW_Ratio,"
             << "Eigen_0,Eigen_1,Eigen_2,"
             << "IntensityMean,IntensityStdDev,LocalDensity,MainAxisAngle,IsHuman\n";
    }

    // 5. 逐筆寫入 featuresList
    for (auto const& [filename, label, f] : featuresList) {
        int isHuman = positiveSet.count(label) ? 1 : 0;

        // 寫入基本欄位
        file << filename << "," 
             << label << ","
             << f.pointCount << ","
             << f.centroidDistance << ","
             << f.averageDistance << ",";

        // Covariance（共變異數矩陣展開 xx,xy,xz,yx,yy,yz,zx,zy,zz）
        file << f.covarianceMatrix[0] << "," << f.covarianceMatrix[1] << "," << f.covarianceMatrix[2] << ","
             << f.covarianceMatrix[3] << "," << f.covarianceMatrix[4] << "," << f.covarianceMatrix[5] << ","
             << f.covarianceMatrix[6] << "," << f.covarianceMatrix[7] << "," << f.covarianceMatrix[8] << ",";

        // Normalized Moments（展開 xx,yy,zz,xy,xz,yz）
        file << f.normalizedMoments[0] << "," << f.normalizedMoments[1] << "," << f.normalizedMoments[2] << ","
             << f.normalizedMoments[3] << "," << f.normalizedMoments[4] << "," << f.normalizedMoments[5] << ",";

        // OBB 長寬高與 L/W 比
        file << f.obb_length << "," << f.obb_width << "," << f.obb_height << "," << f.lw_ratio << ",";

        // Eigen Ratios
        file << f.eigenRatios[0] << "," << f.eigenRatios[1] << "," << f.eigenRatios[2] << ",";

        // 其它統計量與 IsHuman 標記
        file << f.intensityMean << "," << f.intensityStdDev << "," << f.localDensity << ","
             << f.mainAxisAngle << "," << isHuman << "\n";
    }

    file.close();
    std::cout << " 特徵已輸出至: " << csvPath
              << (exists ? " (已追加)" : " (新檔)") << std::endl;
}
