
#include <fstream>

#include "3rdparty/json/json.hpp"
#include "base/map.h"
#include "mapper/incremental_mapper.h"
#include "utility/io_ecim.hpp"
#include "utility/io_feature.hpp"
#include "utility/timer.h"
#include "utility/viewer.h"

using namespace xrsfm;

void PreProcess(const std::string bin_path, Map& map) {
    std::vector<Frame> frames;
    std::vector<Camera> cameras;
    std::vector<FramePair> frame_pairs;
    std::map<std::string, int> name2cid;
    ReadFeatures(bin_path + "/ftr.bin", frames, true);
    ReadFramePairs(bin_path + "/fp.bin", frame_pairs);
    ReadCameraInfo(bin_path + "/camera_info.txt", name2cid, cameras);


    // assert(image_names.size() == frames.size());
    assert(cameras.size() == frames.size());
 
    for (int i = 0; i < frames.size(); ++i) {
        auto& frame = frames.at(i);
        frame.id = i;
        // frame.name = image_names.at(i);
        frame.camera_id = name2cid.at(frame.name);
        const auto& camera = cameras.at(frame.camera_id);
        const int num_points = frame.keypoints_.size();
        frame.points.clear();
        frame.points_normalized.clear();
        frame.uint_descs_.resize(0, 0);
        frame.track_ids_.assign(num_points, -1);
        for (const auto& kpt : frame.keypoints_) {
            const auto& pt = kpt.pt;
            Eigen::Vector2d ept(pt.x, pt.y), eptn;
            ImageToNormalized(camera, ept, eptn);
            frame.points.emplace_back(ept);
            frame.points_normalized.emplace_back(eptn);
        }
    } 

    for(int i = 0;i<cameras.size();++i){
        std::cout<<i<<" "<<cameras.size()<<std::endl;
        auto&camera = cameras[i];
        // camera.log();
        // if distortion parameters of the camera are not estimated, the camera parameters is invalid. 
        if(camera.distort_params[0]==0){
            camera.set_invalid();
        }
    }

    map.frames_ = frames;
    map.cameras_ = cameras;
    map.frame_pairs_ = frame_pairs;
    map.RemoveRedundancyPoints();
    map.Init();
    printf("Num Frames: %d Num Pairs %d\n", map.frames_.size(), map.frame_pairs_.size());
}

int main(int argc, char* argv[]) {
    google::InitGoogleLogging(argv[0]);
    // 1.Read Config
    std::string bin_dir_path,output_path;
    if (argc <= 2){
        std::string config_path = "./config_uno.json";
        if(argc == 2){
            config_path = argv[1];
        }
        auto config_json = LoadJSON(config_path);
        bin_dir_path = config_json["bin_dir_path"];
        output_path = config_json["output_path"]; 
    }else if (argc == 3){
        bin_dir_path = argv[1];
        output_path = argv[2];
    } 
    
    // 2.Initialize Program
    Map map;
    PreProcess(bin_dir_path, map);
    std::cout << "PreProcess Done!" << std::endl;

    // 3. Map Reconstruction
    IncrementalMapper imapper;
    imapper.options.th_rpe_gba = 4.0;
    imapper.options.th_angle_gba = 1.5;
    imapper.Reconstruct(map);
    std::cout << "Reconstruction Done!" << std::endl;

    WriteColMapDataBinary(output_path, map);

    return 0;
}
