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

inline std::map<int, Camera> ReadCamerasText(const std::string &path) {
    std::map<int, Camera> cameras;

    std::ifstream file(path);
    CHECK(file.is_open()) << path;

    std::string line;
    std::string item;

    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }

        std::stringstream ss(line);
        int camera_id, width, height;
        std::string model_name;
        ss >> camera_id >> model_name >> width >> height;
        Camera camera(camera_id, model_name, width, height);

        for (int i = 0; i < camera.params_.size(); ++i) {
            ss >> camera.params_[i];
        }
        cameras.emplace(camera_id, camera);
    }
    file.close();
    return cameras;
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

bool ReadColMapDataBinary(const std::string &output_path, Map &map);

void WriteColMapDataBinary(const std::string &output_path, const Map &map);

void WriteColMapDataBinary2(const std::string &output_path, const Map &map);

void ReadImagesBinary(const std::string &path, std::map<int, Frame> &frames);

void ReadImagesBinaryForTriangulation(const std::string &path,
                                      std::map<int, Frame> &frames);

void ReadCamerasBinary(const std::string &path, std::map<int, Camera> &cameras);

void ReadFramePairBinaryForTriangulation(const std::string &path,
                                         std::vector<FramePair> &frame_pairs);
} // namespace xrsfm
