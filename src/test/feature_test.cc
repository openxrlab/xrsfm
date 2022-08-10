
#include "feature/feature_processing.h"

using namespace xrsfm;

int main(int argc, char* argv[]) {
    std::string image_dir_path = "";
    std::vector<Frame> frames(2);
    frames[0].id = 0;
    frames[0].name = "";
    frames[1].id = 1;
    frames[1].name = "";
    FeatureExtract(image_dir_path, frames);
    std::vector<std::pair<int, int>> id_pairs = {{0, 1}};
    std::vector<FramePair> frame_pairs;
    FeatureMatching(frames, id_pairs, frame_pairs, true);
}