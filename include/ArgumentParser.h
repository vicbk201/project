#ifndef ARGUMENT_PARSER_H
#define ARGUMENT_PARSER_H

#include <string>

class ArgumentParser {
public:
    // 解析命令列參數:
    // --cloudfile <PCD檔案路徑> --method <octree|voxelgrid> --cluster <euclidean|dbscan> [--resolution <解析度>] [--o | --output <完整輸出PCD檔案路徑>]
    static bool parseArguments(int argc, char** argv,
                               std::string& cloudFile,
                               float& resolution,
                               std::string& downsampleMethod,
                               std::string& clusterMethod,
                               std::string& outputCloudFile);
};

#endif // ARGUMENT_PARSER_H
