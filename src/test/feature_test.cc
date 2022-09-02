
#include "feature/feature_processing.h"
#include <gtest/gtest.h>

using namespace xrsfm;

TEST(FeatureTest, ExtractionAndMatching) {
    std::string image_dir_path = "/data/ECIM/UT/";
    std::vector<Frame> frames(2);
    frames[0].id = 0;
    frames[0].name = "000000.png";
    frames[1].id = 1;
    frames[1].name = "000001.png";
    FeatureExtract(image_dir_path, frames);

    std::vector<std::pair<int, int>> id_pairs = {{0, 1}};
    std::vector<FramePair> frame_pairs;
    FeatureMatching(frames, id_pairs, frame_pairs, true);
}

int main(int argc, char *argv[]) {
    testing::InitGoogleTest(&argc, argv);
    RUN_ALL_TESTS();
    return 0;
}
