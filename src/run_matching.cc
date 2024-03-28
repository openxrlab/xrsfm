

#include <cctype>
#include <experimental/filesystem>
#include <regex>
#include <unordered_set>

#include "base/map.h"
#include "feature/feature_processor.h"
#include "utility/io_ecim.hpp"
#include "utility/timer.h"

int main(int argc, const char *argv[]) {
    google::InitGoogleLogging(argv[0]);

    using namespace xrsfm;
    std::string images_path, retrieval_path, matching_type, output_path;
    if (argc <= 2) {
        std::string config_path = "./config_mat.json";
        if (argc == 2) {
            config_path = argv[1];
        }
        auto config_json = LoadJSON(config_path);
        images_path = config_json["images_path"];
        retrieval_path = config_json["retrieval_path"];
        matching_type = config_json["matching_type"];
        output_path = config_json["output_path"];
    } else if (argc == 3) {
        images_path = argv[1];
        output_path = argv[2];
        matching_type = "sequential";
        retrieval_path = "";
    } else if (argc == 5) {
        images_path = argv[1];
        retrieval_path = argv[2];
        matching_type = argv[3];
        output_path = argv[4];
    } else {
        exit(-1);
    }

    FeatureProcessor feature_processor(images_path, output_path, matching_type,
                                       retrieval_path);
    feature_processor.Run();

    return 0;
}
