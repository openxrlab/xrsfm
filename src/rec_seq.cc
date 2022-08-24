
#include <fstream>

#include "3rdparty/json/json.hpp"
#include "base/map.h"
#include "mapper/incremental_mapper.h"
#include "utility/io_ecim.hpp"
#include "utility/io_feature.hpp"
#include "utility/timer.h"
#include "utility/viewer.h"

using namespace xrsfm;

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
    // 1.Read Config
    std::string bin_path,images_path,camera_path,output_path;
    int init_id1,init_id2;
    if (argc <= 2){
        std::string config_path = "./config_seq.json";
        if(argc == 2){
            config_path = argv[1];
        }
        auto config_json = LoadJSON(config_path);
        bin_path = config_json["bin_path"];
        images_path = config_json["images_path"];
        camera_path = config_json["camera_path"];
        output_path = config_json["output_path"];
        init_id1 = config_json["init_id1"];
        init_id2 = config_json["init_id2"];
    }else if (argc == 7){
        bin_path = argv[1];
        images_path = argv[2];
        camera_path = argv[3];
        output_path = argv[4];
        init_id1 = std::stoi(argv[5]);
        init_id2 = std::stoi(argv[6]);
    }else{
        exit(-1);
    }
    std::cout << "Read Config Done!" << std::endl;

    // 2. Map PreProcess
    Map map;
    PreProcess(bin_path, images_path, camera_path, map);
    std::cout << "PreProcess Done!" << std::endl;

    // 3. Map Reconstruction
    IncrementalMapper imapper;
    imapper.options.init_id1 = init_id1;
    imapper.options.init_id2 = init_id2;
    imapper.options.correct_pose = true;
    imapper.options.stop_when_register_fail = true;
    imapper.Reconstruct(map);
    std::cout << "Reconstruction Done!" << std::endl;

    // 4. Output
    WriteColMapDataBinary(output_path, map);

    return 0;
}
