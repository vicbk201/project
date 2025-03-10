#ifndef ARGUMENT_PARSER_H
#define ARGUMENT_PARSER_H

#include <string>

class ArgumentParser {
public:
    // 解析命令列參數:
    // --cloudfile <PCD檔案路徑> --method <octree|voxelgrid> [--resolution <解析度>] 或 [--r <解析度>]
    // [--o | --output <完整輸出PCD檔案路徑>]
    // 若解析度未提供，則使用預設值 0.2；若未提供輸出檔案參數，則不儲存結果
    static bool parseArguments(int argc, char** argv, std::string& cloudFile, float& resolution, std::string& method, std::string& outputCloudFile);
};

#endif // ARGUMENT_PARSER_H
