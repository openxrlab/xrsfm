
#include <fstream>

#include "3rdparty/json/json.hpp"
#include "base/map.h"
#include "mapper/incremental_mapper.h"
#include "utility/io_ecim.hpp"
#include "utility/io_feature.hpp"
#include "geometry/epipolar_geometry.hpp"

using namespace xrsfm;

bool self_calibration(std::vector<Frame> &frames,
                      std::vector<FramePair> &frame_pairs, double w, double h,
                      double &estimated_focal) {
    std::map<float, float> focal_score_map;

    for (int k = 0; k < 100; ++k) {
        const double f = 0.25 * w + 2 * w * k / 100;
        focal_score_map[f] = 0;
    }

    int count = 0;
    for (const auto &fp : frame_pairs) {
        if (fp.inlier_num < 200)
            continue;

        std::vector<Eigen::Vector2d> points1, points2;
        for (const auto &match : fp.matches) {
            points1.push_back(frames.at(fp.id1).points[match.id1]);
            points2.push_back(frames.at(fp.id2).points[match.id2]);
        }
        double distance = 0;
        for (int i = 0; i < points1.size(); ++i) {
            points1.at(i).x() -= 0.5 * w;
            points1.at(i).y() -= 0.5 * h;
            points2.at(i).x() -= 0.5 * w;
            points2.at(i).y() -= 0.5 * h;
            distance = (points2.at(i) - points1.at(i)).norm();
        }
        distance /= points1.size();
        FramePair tmp_fp;
        SolveFundamentalCOLMAP(points1, points2, tmp_fp);

        for (int k = 0; k < 100; ++k) {
            const double f = 0.25 * w + 2 * w * k / 100;
            Eigen::Matrix3d A = tmp_fp.F;
            A(0, 2) /= f;
            A(1, 2) /= f;
            A(2, 0) /= f;
            A(2, 1) /= f;
            A(2, 2) /= (f * f);
            Eigen::JacobiSVD<Eigen::Matrix3d> svd(A, Eigen::ComputeFullU |
                                                         Eigen::ComputeFullV);
            Eigen::Vector3d s = svd.singularValues();
            s /= s(0);
            focal_score_map[f] += s(1);
        }
        count++;
    }

    std::vector<std::pair<float, float>> focal_vec(focal_score_map.begin(),
                                                   focal_score_map.end());
    std::sort(
        focal_vec.begin(), focal_vec.end(),
        [](const std::pair<float, float> &a, const std::pair<float, float> &b) {
            return a.second > b.second;
        });

    estimated_focal = focal_vec.at(0).first;
    std::cout << focal_vec.at(0).first << " " << focal_vec.at(0).second / count
              << std::endl;

    return true;
}

void PreProcess(const std::string dir_path, const std::string camera_path,
                Map &map) {
    std::vector<Frame> frames;
    std::vector<FramePair> frame_pairs;
    ReadFeatures(dir_path + "ftr.bin", frames, true);
    ReadFramePairs(dir_path + "fp.bin", frame_pairs);

    // convert keypoint to points(for reconstruction)
    for (auto &frame : frames) {
        const int num_points = frame.keypoints_.size();
        frame.points.clear();
        frame.track_ids_.assign(num_points, -1);
        for (const auto &kpt : frame.keypoints_) {
            const auto &pt = kpt.pt;
            Eigen::Vector2d ept(pt.x, pt.y), eptn;
            frame.points.push_back(ept);
        }
    }

    // set cameras & image name
    std::map<int, Camera> cameras;
    if (camera_path == "auto") {
        std::vector<ImageSize> image_size;
        LoadImageSize(dir_path + "size.bin", image_size);
        const int w = image_size.at(0).width;
        const int h = image_size.at(0).height;

        double focal = 0.5 * w;
        self_calibration(frames, frame_pairs, w, h, focal);

        Camera seq(0, "SIMPLE_RADIAL", w, h);
        seq.params_ = {focal, 0.5 * w, 0.5 * h, 0};
        cameras[0] = seq;
    } else {
        cameras = ReadCamerasText(camera_path);
    }

    CHECK_EQ(cameras.size(), 1);
    const int camera_id = cameras.begin()->first;

    for (auto &frame : frames) {
        frame.camera_id = camera_id;
    }

    // TODO check single camera

    map.camera_map_ = cameras;
    map.frames_ = frames;
    map.frame_pairs_ = frame_pairs;

    map.RemoveRedundancyPoints();
    map.Init();
}

int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    // 1.Read Config
    std::string bin_path, camera_path, output_path;
    int init_id1 = -1, init_id2 = -1;
    if (argc <= 2) {
        std::string config_path = "./config_seq.json";
        if (argc == 2) {
            config_path = argv[1];
        }
        auto config_json = LoadJSON(config_path);
        bin_path = config_json["bin_path"];
        camera_path = config_json["camera_path"];
        output_path = config_json["output_path"];
        init_id1 = config_json["init_id1"];
        init_id2 = config_json["init_id2"];
    } else if (argc >= 4 && argc <= 6) {
        bin_path = argv[1];
        camera_path = argv[2];
        output_path = argv[3];
        if (argc >= 5) {
            init_id1 = std::stoi(argv[4]);
        }
        if (argc == 6) {
            init_id2 = std::stoi(argv[5]);
        }
    } else {
        exit(-1);
    }
    std::cout << "Read Config Done!" << std::endl;

    // 2. Map PreProcess
    Map map;
    PreProcess(bin_path, camera_path, map);
    std::cout << "PreProcess Done!" << std::endl;

    // 3. Map Reconstruction
    IncrementalMapper imapper;
    imapper.options.init_id1 = init_id1;
    imapper.options.init_id2 = init_id2;
    imapper.options.correct_pose = false;
    imapper.options.stop_when_register_fail = true;
    imapper.Reconstruct(map);
    std::cout << "Reconstruction Done!" << std::endl;

    // 4. Output
    WriteColMapDataBinary(output_path, map);

    return 0;
}
