#include <cctype>
#include <regex>
#include <unordered_set>

#include "feature/feature_processing.h"
#include "base/map.h"
#include "utility/io_ecim.hpp"
#include "utility/timer.h"
#include "3rdparty/json/json.hpp"
#include "base/map.h"
#include "mapper/incremental_mapper.h"
#include "utility/io_ecim.hpp"
#include "utility/io_feature.hpp"
#include "utility/timer.h"
#include "utility/viewer.h"
#include "tag/tag_extract.hpp"

using namespace xrsfm;

void MatchingSeq(std::vector<Frame>& frames, const std::string& fp_path, std::map<int, std::vector<int>>& id2rank) {
    std::set<std::pair<int, int>> set_pairs;
    for (int i = 0, num = frames.size(); i < num; ++i) {
        for (int k = 0; k < 20 && i + k < num; ++k) set_pairs.insert(std::pair<int, int>(i, i + k));
    }
    for (auto& [id1, vec] : id2rank) {
        if (id1 % 5 != 0) continue;
        for (auto& id2 : vec) {
            set_pairs.insert(std::pair<int, int>(id1, id2));
        }
    }
    std::vector<std::pair<int, int>> id_pairs;
    id_pairs.assign(set_pairs.begin(), set_pairs.end());

    std::vector<FramePair> frame_pairs;
    FeatureMatching(frames, id_pairs, frame_pairs, true);
    SaveFramePairs(fp_path, frame_pairs);
}

void PreProcess(const std::string dir_path, const std::string images_path,const std::string camera_path, Map& map) {
    std::vector<Frame> frames;
    std::vector<FramePair> frame_pairs;
    std::vector<std::string> image_names;
    ReadFeatures(dir_path + "ftr.bin", frames, true);
    ReadFramePairs(dir_path + "fp.bin", frame_pairs);
    LoadImageNames(images_path, image_names);

    // set cameras & image name
    std::vector<Camera> cameras;
    // TODO: ReadCamera
    // Camera seq(0, 1000, 1000, 640, 360, 0.0);
    // Camera seq =  Camera(0, 886.420017084725,881.278105028345, 479.5, 269.5, -0.004);
    Camera seq = ReadCameraIOSRecord(camera_path);
    seq.log();
    cameras.emplace_back(seq);
    for (auto& frame : frames) {
        frame.camera_id = 0;
        frame.name = image_names.at(frame.id);
    }

    // set points for reconstruction
    for (auto& frame : frames) {
        const int num_points = frame.keypoints_.size();
        frame.points.clear();
        frame.points_normalized.clear();
        frame.track_ids_.assign(num_points, -1);
        for (const auto& kpt : frame.keypoints_) {
            const auto& pt = kpt.pt;
            Eigen::Vector2d ept(pt.x, pt.y), eptn;
            ImageToNormalized(cameras[0], ept, eptn);
            frame.points.emplace_back(ept);
            frame.points_normalized.emplace_back(eptn);
        }
    }

    map.frames_ = frames;
    map.cameras_ = cameras;
    map.frame_pairs_ = frame_pairs;
    map.RemoveRedundancyPoints();
    map.Init();
}

int main(int argc, char* argv[]) {  
    google::InitGoogleLogging(argv[0]);
    const std::string dir_path = "/data1/XRSFM/2022-07-06T14-39-35/";
    const std::string image_dir_path = dir_path+"images/";
    const std::string camera_path = dir_path+"camera.txt";
    const std::string retrival_path = dir_path+"results/pairs-sfm.txt";
    const std::string output_path = dir_path+"ut/"; 
    const std::string output_path1 = dir_path+"ut1/"; 

    std::string ftr_path = output_path + "ftr.bin";
    std::string size_path = output_path + "size.bin";
    std::string fp_init_path = output_path + "fp_init.bin";
    std::string fp_path = output_path + "fp.bin";

    // // 1.read images
    // std::vector<std::string> image_names;
    // LoadImageNames(image_dir_path, image_names); 
    // std::map<std::string, int> name2id;
    // const int num_image = image_names.size();
    // std::vector<Frame> frames(num_image);
    // for (int i = 0; i < num_image; ++i) {
    //     frames[i].id = i;
    //     frames[i].name = image_names[i];
    //     name2id[image_names[i]] = i;
    // }
    // std::cout << "Load Image Info Done.\n";

    // // 2.feature extraction
    // FeatureExtract(image_dir_path, frames);
    // SaveFeatures(ftr_path, frames, true);
    // std::cout << "Extract Features Done.\n";

    // // 3.image matching
    // std::map<int, std::vector<int>> id2rank;
    // LoadRetrievalRank(retrival_path, name2id, id2rank);
    // MatchingSeq(frames, fp_path, id2rank);  
    // std::cout << "Match Features Done.\n";

    // 1. Map PreProcess
    // Map map;
    // PreProcess(output_path, image_dir_path, camera_path, map);
    // std::cout << "PreProcess Done!" << std::endl;

    // // 2. Map Reconstruction
    // IncrementalMapper imapper;
    // imapper.options.init_id1 = 0;
    // imapper.options.init_id2 = 10;
    // imapper.options.correct_pose = true;
    // imapper.options.only_with_sim3 = true;
    // imapper.options.stop_when_register_fail = true;
    // imapper.Reconstruct(map);
    // std::cout << "Reconstruction Done!" << std::endl;

    // // 3. Output
    // WriteColMapDataBinary(output_path, map);

    // 1. refine with tag
    tag_refine(image_dir_path,output_path,output_path1); 

    return 0;
}