
#include <fstream>

#include "3rdparty/json/json.hpp"
#include "base/map.h"
#include "mapper/incremental_mapper.h"
#include "utility/io_ecim.hpp"
#include "utility/io_feature.hpp"
#include "utility/timer.h"
// #include "utility/viewer.h"

using namespace xrsfm;

void PreProcess(const std::string dir_path, const int camera_param_id,
                Map &map) {
    std::vector<Frame> frames;
    std::map<int, Camera> cameras;
    std::vector<FramePair> frame_pairs;
    ReadFeatures(dir_path + "ftr.bin", frames, true);
    ReadFramePairs(dir_path + "fp.bin", frame_pairs);

    // set cameras
    Camera seq;
    if (camera_param_id == 0) {
        seq = Camera(0, 718.856, 718.856, 607.1928, 185.27157, 0.0); // 00-02
    } else if (camera_param_id == 1) {
        seq = Camera(0, 721.5377, 721.5377, 609.5593, 172.854, 0.0); // 03
    } else if (camera_param_id == 2) {
        seq = Camera(0, 707.0912, 707.0912, 601.8873, 183.1104, 0.0); // 04-12
    }
    cameras[seq.id_] = seq;
    for (auto &frame : frames) {
        frame.camera_id = 0;
    }

    // set points for reconstruction
    for (auto &frame : frames) {
        const int num_points = frame.keypoints_.size();
        frame.points.clear();
        // frame.points_normalized.clear();
        frame.track_ids_.assign(num_points, -1);
        for (const auto &kpt : frame.keypoints_) {
            const auto &pt = kpt.pt;
            Eigen::Vector2d ept(pt.x, pt.y), eptn;
            // ImageToNormalized(cameras[0], ept, eptn);
            frame.points.emplace_back(ept);
            // frame.points_normalized.emplace_back(eptn);
        }
    }

    map.frames_ = frames;
    map.camera_map_ = cameras;
    map.frame_pairs_ = frame_pairs;
    map.RemoveRedundancyPoints();
    map.Init();
}

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    // 1.Read Config
    std::string bin_path, data_path, seq_name, output_path;
    int init_id1, init_id2;
    if (argc <= 2) {
        std::string config_path = "./config_kitti.json";
        if (argc == 2) {
            config_path = argv[1];
        }
        auto config_json = LoadJSON(config_path);
        bin_path = config_json["bin_path"];
        data_path = config_json["data_path"];
        seq_name = config_json["seq_name"];
        output_path = config_json["output_path"];

        init_id1 = config_json["init_id1"];
        init_id2 = config_json["init_id2"];
    } else if (argc == 7) {
        bin_path = argv[1];
        data_path = argv[2];
        seq_name = argv[3];
        output_path = argv[4];
        init_id1 = std::stoi(argv[5]);
        init_id2 = std::stoi(argv[6]);
    } else {
        exit(-1);
    }
    const std::string seq_path = data_path + seq_name + "/";
    std::map<std::string, int> name2camera_id = {
        {"00", 0}, {"01", 0}, {"02", 0}, {"03", 1}, {"04", 2}, {"05", 2},
        {"06", 2}, {"07", 2}, {"08", 2}, {"09", 2}, {"10", 2}};
    CHECK(name2camera_id.count(seq_name) != 0) << "NO suitable camera param.";
    const int camera_param_id = name2camera_id[seq_name];

    // 2. Map PreProcess
    Map map;
    PreProcess(bin_path, camera_param_id, map);
    std::cout << "PreProcess Done!" << std::endl;

    // 3. Map Reconstruction
    IncrementalMapper imapper;
    imapper.options.init_id1 = init_id1;
    imapper.options.init_id2 = init_id2;
    imapper.options.correct_pose = true;
    imapper.options.stop_when_register_fail = true;
    imapper.Reconstruct(map);
    std::cout << "Reconstruction Done!" << std::endl;

    // 4. Output Trajectory
    std::vector<double> timestamp_vec;
    LoadTimeStamp(seq_path + "times.txt", timestamp_vec);
    UpdateFrameTimeStamp(map.frames_, timestamp_vec);
    WriteTrajectory(map, output_path + seq_name + "_test.tum");
    WriteColMapDataBinary(output_path, map);

    return 0;
}
