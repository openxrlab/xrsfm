//
// Created by SENSETIME\yezhichao1 on 2020/4/5.
//

#pragma once

#include <fcntl.h>
#include <glog/logging.h>
#include <sys/mman.h>
#include <sys/stat.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <string>

#include "base/map.h"
#include "io_feature.hpp"

namespace xrsfm {

inline Camera ReadCameraIOSRecord(const std::string &file_name) {
    Camera cam(0, 2);

    std::ifstream file(file_name, std::ios::out);
    std::string line;
    while (std::getline(file, line)) {
        if (line[0] == '#')
            continue;
        std::string image_name, model_name;
        std::stringstream ss(line);
        ss >> image_name >> model_name;
        ss >> cam.params_[0] >> cam.params_[0] >> cam.params_[1] >>
            cam.params_[2] >> cam.params_[3];
        break;
    }

    return cam;
}

inline void ReadCameraInfo(const std::string &file_name,
                           std::map<std::string, int> &name2cid,
                           std::map<int, Camera> &cameras) {
    std::ifstream file(file_name, std::ios::out | std::ios::binary);
    std::string line;
    while (std::getline(file, line)) {
        if (line[0] == '#')
            continue;
        if (line.size() < 10)
            continue;

        int w, h;
        std::string image_name, model_name;
        std::stringstream ss(line);
        ss >> image_name >> model_name >> w >> h;
        if (model_name == "SIMPLE_RADIAL") {
            const int camera_id = cameras.size();
            Camera cam(camera_id, 2);
            ss >> cam.params_[0] >> cam.params_[1] >> cam.params_[2] >>
                cam.params_[3];
            name2cid[image_name] = camera_id;
            cameras[camera_id] = cam;
        } else {
            CHECK(false);
        }
    }
}

inline void LoadTimeStamp(const std::string timestamp_path,
                          std::vector<double> &timestamp_vec) {
    std::ifstream file;
    file.open(timestamp_path);
    CHECK(file.is_open()) << timestamp_path;
    while (!file.eof()) {
        std::string s;
        std::getline(file, s);
        if (!s.empty()) {
            std::stringstream ss;
            ss << s;
            double t;
            ss >> t;
            timestamp_vec.push_back(t);
        }
    }
    return;
}

inline void UpdateFrameTimeStamp(std::vector<Frame> &frames,
                                 std::vector<double> &timestamp_Vec) {
    int step = int(1.0 * timestamp_Vec.size() / frames.size() + 0.1);
    printf("%d %zu %zu\n", step, timestamp_Vec.size(), frames.size());
    for (auto &f : frames) {
        f.timestamp = timestamp_Vec[step * f.id];
    }
    return;
}

inline void WriteTrajectory(const Map &map,
                            const std::string &trajectory_path) {
    std::ofstream trajectory_file(trajectory_path);
    for (auto &frame : map.frames_) {
        if (!frame.registered)
            continue;
        Eigen::Vector3d twc = frame.twc();
        Eigen::Quaterniond qwc = frame.qwc();
        trajectory_file << std::to_string(frame.timestamp) << " " << twc[0]
                        << " " << twc[1] << " " << twc[2] << " " << qwc.x()
                        << " " << qwc.y() << " " << qwc.z() << " " << qwc.w()
                        << "\n";
    }
    trajectory_file.close();
}

void ReadColMapDataBinary(const std::string &output_path, Map &map);

void WriteColMapDataBinary(const std::string &output_path, const Map &map);

void WriteColMapDataBinary2(const std::string &output_path, const Map &map);

void ReadImagesBinary(const std::string &path, std::map<int, Frame> &frames);

void ReadImagesBinaryForTriangulation(const std::string &path,
                                      std::map<int, Frame> &frames);

void ReadCamerasBinary(const std::string &path, std::map<int, Camera> &cameras);

void ReadFramePairBinaryForTriangulation(const std::string &path,
                                         std::vector<FramePair> &frame_pairs);
} // namespace xrsfm
