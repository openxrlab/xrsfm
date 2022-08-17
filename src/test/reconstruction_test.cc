#include <cctype>
#include <regex>
#include <unordered_set>

#include "feature/feature_processing.h"
#include "base/map.h"
#include "utility/io_ecim.hpp"
#include "utility/timer.h"

int main(int argc, char* argv[]) { 
    const std::string image_dir_path = "";
    const std::string retrival_path = "";
    const std::string matching_type = "";
    const std::string output_path = ""; 
    std::string ftr_path = output_path + "ftr.bin";
    std::string size_path = output_path + "size.bin";
    std::string fp_init_path = output_path + "fp_init.bin";
    std::string fp_path = output_path + "fp.bin";

    // 1.read images
    std::vector<std::string> image_names;
    LoadImageNames(image_dir_path, image_names); 
    std::map<std::string, int> name2id;
    const int num_image = image_names.size();
    std::vector<Frame> frames(num_image);
    for (int i = 0; i < num_image; ++i) {
        frames[i].id = i;
        frames[i].name = image_names[i];
        name2id[image_names[i]] = i;
    }
    std::cout << "Load Image Info Done.\n";

    // 2.feature extraction
    GetFeatures(image_dir_path, ftr_path, frames);
    std::cout << "Extract Features Done.\n";

    // 3.image matching
    std::map<int, std::vector<int>> id2rank;
    LoadRetrievalRank(retrival_path, name2id, id2rank);
    MatchingSeq(frames, fp_path, id2rank);  
    std::cout << "Match Features Done.\n";

    // 1. Map PreProcess
    Map map;
    PreProcess(bin_path, images_path, camera_path, map);
    std::cout << "PreProcess Done!" << std::endl;

    // 2. Map Reconstruction
    IncrementalMapper imapper;
    imapper.options.init_id1 = init_id1;
    imapper.options.init_id2 = init_id2;
    imapper.options.correct_pose = true;
    imapper.options.stop_when_register_fail = true;
    imapper.Reconstruct(map);
    std::cout << "Reconstruction Done!" << std::endl;

    // 3. Output
    WriteColMapDataBinary(output_path, map);

    // 1. refine with tag
    tag_refine(image_dir_path,output_path,output_path); 

    return 0;
}