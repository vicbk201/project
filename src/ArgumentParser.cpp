#include "ArgumentParser.h"
#include <iostream>
#include <stdexcept>
#include <cctype>

bool ArgumentParser::parseArguments(int argc, char** argv, std::string& cloudFile, float& resolution, std::string& method)
{
    // 設定預設解析度
    resolution = 0.2f;
    // 清空 method，要求使用者必須輸入
    method = "";

    // 從 argv[1] 開始解析參數
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--cloudfile") {
            if (i + 1 < argc) {
                cloudFile = argv[++i];
            } else {
                std::cerr << "Error: --cloudfile 需要一個檔案路徑" << std::endl;
                return false;
            }
        } else if (arg == "--method") {
            if (i + 1 < argc) {
                method = argv[++i];
                // 轉為小寫
                for (auto &c : method) {
                    c = std::tolower(c);
                }
                if (method != "octree" && method != "voxelgrid") {
                    std::cerr << "Error: --method 必須是 'octree' 或 'voxelgrid'" << std::endl;
                    return false;
                }
            } else {
                std::cerr << "Error: --method 需要一個值 (octree 或 voxelgrid)" << std::endl;
                return false;
            }
        } else if (arg == "--resolution" || arg == "--r") {
            if (i + 1 < argc) {
                try {
                    resolution = std::stof(argv[++i]);
                    if (resolution <= 0) {
                        throw std::invalid_argument("解析度必須是正數");
                    }
                } catch (const std::exception& e) {
                    std::cerr << "Error: 解析度錯誤: " << e.what() << std::endl;
                    return false;
                }
            } else {
                std::cerr << "Error: " << arg << " 需要一個數值" << std::endl;
                return false;
            }
        } else {
            std::cerr << "Warning: 未知的參數: " << arg << std::endl;
        }
    }

    // 檢查必要的參數是否提供
    if (cloudFile.empty()) {
        std::cerr << "Error: 必須提供 --cloudfile <PCD檔案路徑>" << std::endl;
        return false;
    }
    if (method.empty()) {
        std::cerr << "Error: 必須提供 --method <octree|voxelgrid>" << std::endl;
        return false;
    }

    return true;
}
