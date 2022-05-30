//
// Created by SENSETIME\yezhichao1 on 2020/4/5.
//

#pragma once

#include <glog/logging.h>

#include <chrono>
#include <experimental/filesystem>
#include <string>

#include "io_base.hpp"
#include "base/map.h"
#include "3rdparty/json/json.hpp"

inline nlohmann::json LoadJSON(const std::string config_path) {
  nlohmann::json config_json;
  std::ifstream ifs(config_path);
  ifs >> config_json;
  ifs.close();
  return config_json;
}

inline void LoadImageNames(const std::string &dir_path, std::vector<std::string> &image_names) {
  namespace fs = std::experimental::filesystem;
  for (const auto &entry : fs::directory_iterator(dir_path)) {
    const std::string filename = entry.path().filename();
    image_names.emplace_back(filename);
  }
  std::sort(image_names.begin(), image_names.end());
}

inline void LoadImages(const std::string common_path, const std::vector<std::string> &image_paths,
                       std::vector<cv::Mat> &images) {
  images.clear();
  images.reserve(image_paths.size());
  for (auto &image_path : image_paths) {
    cv::Mat image = cv::imread(common_path + "/" + image_path);
    images.emplace_back(image);
    if (image.empty()) {
      std::cout << "no image " + image_path << std::endl;
      break;
    }
  }
}

inline void ReadFeatures(const std::string &file_name, std::vector<Frame> &frames,bool init_frames = false) {
  std::ifstream file(file_name, std::ios::out | std::ios::binary);
  if (!file.is_open()) {
    std::cerr << "Error: can not open " << file_name << "\n";
    return;
  }
  // points track_ids_ keypoints_ uint_descs_
  int num_frames = -1;
  read_data(file, num_frames);
  
  if(init_frames){
    frames.resize(num_frames);
    for(int i = 0;i<num_frames;++i){
      frames[i].id = i;
    }
  }

  assert(num_frames == frames.size());
  for (auto &frame : frames) {
    int num_points = -1;
    read_data(file, num_points); 
    frame.keypoints_.resize(num_points);
    frame.uint_descs_.resize(num_points, 128);

    for (int i = 0; i < num_points; ++i) {
      read_data(file, frame.keypoints_[i].pt.x);
      read_data(file, frame.keypoints_[i].pt.y);
      read_data(file, frame.keypoints_[i].size);
      read_data(file, frame.keypoints_[i].angle);
    }
    read_data_vec(file, frame.uint_descs_.data(), num_points * 128);
  }
}

inline void SaveFeatures(const std::string &file_name, const std::vector<Frame> &frames,bool with_descs = false) {
  std::ofstream file(file_name, std::ios::out | std::ios::binary);
  if (!file.is_open()) {
    std::cerr << "Error: can not open " << file_name << "\n";
    return;
  }
  // points track_ids_ keypoints_ uint_descs_
  int num_frames = frames.size();
  write_data(file, num_frames);
  for (const auto &frame : frames) {
    const int num_points = frame.keypoints_.size();
    write_data(file, num_points);
    for (int i = 0; i < num_points; ++i) {
      write_data(file, frame.keypoints_[i].pt.x);
      write_data(file, frame.keypoints_[i].pt.y);
      write_data(file, frame.keypoints_[i].size);
      write_data(file, frame.keypoints_[i].angle);
    }
    if(with_descs) write_data_vec(file, frame.uint_descs_.data(), num_points * 128);
  }
}

inline void ReadFramePairs(const std::string &file_name, std::vector<FramePair> &frame_pairs) {
  std::ifstream file(file_name, std::ios::in | std::ios::binary);
  CHECK(file.is_open());
  size_t num_framepairs = read_data2<size_t>(file);
  frame_pairs.resize(num_framepairs);
  for (auto &frame_pair : frame_pairs) {
    read_data(file, frame_pair.id1);
    read_data(file, frame_pair.id2);
    // std::cout<<frame_pair.id1<<" "<<frame_pair.id2<<std::endl;
    size_t num_matches = read_data2<size_t>(file);
    frame_pair.matches.resize(num_matches);
    read_data_vec(file, &(frame_pair.matches[0]), num_matches);
    read_data(file, frame_pair.E);
    read_data(file, frame_pair.inlier_num);
    frame_pair.inlier_mask.resize(num_matches);
    read_data_vec(file, &(frame_pair.inlier_mask[0]), num_matches);
  }
  file.close();
}

inline void SaveFramePairs(const std::string &file_name,
                           const std::vector<FramePair> &frame_pairs) {
  std::ofstream file(file_name, std::ios::out | std::ios::binary);
  size_t num_framepairs = frame_pairs.size();
  write_data(file, num_framepairs);
  for (const auto &frame_pair : frame_pairs) {
    write_data(file, frame_pair.id1);
    write_data(file, frame_pair.id2);
    size_t num_matches = frame_pair.matches.size();
    write_data(file, num_matches);
    write_data_vec(file, &(frame_pair.matches[0]), num_matches);
    write_data(file, frame_pair.E);
    write_data(file, frame_pair.inlier_num);
    write_data_vec(file, &(frame_pair.inlier_mask[0]), num_matches);
  }
  file.close();
}

inline void ReadCameraInfo(const std::string &file_name, 
  std::map<std::string,int> &name2cid,std::vector<Camera> &cameras){

  std::ifstream file(file_name, std::ios::out | std::ios::binary);
  std::string line;
  while (std::getline(file, line)) {
    if (line[0] == '#') continue;
    if (line.size()<10) continue;
    Camera cam;
    int w,h;
    std::string image_name,model_name;
    std::stringstream ss(line);
    ss>>image_name>>model_name>>w>>h;
    ss>>cam.camera_params[0]>>cam.camera_params[2]>>cam.camera_params[3]>>cam.distort_params[0];
    cam.camera_params[1] = cam.camera_params[0];
    // std::cout<<image_name<<" ";
    // cam.log();
    const int id = cameras.size();
    cam.id = name2cid[image_name] = id;
    cameras.emplace_back(cam);
  }   
}

inline void LoadImageSize(const std::string &file_name, std::vector<ImageSize> &image_size) {
  std::ifstream file(file_name, std::ios::out | std::ios::binary);
  if (!file.is_open()) {
    std::cerr << "Error: can not open " << file_name << "\n";
    return;
  }
  int num_frames = -1;
  read_data(file, num_frames);
  image_size.resize(num_frames);
  for (auto &size : image_size) {
    read_data(file, size.width);
    read_data(file, size.height);
  }
}

inline void LoadRetrievalRank(const std::string &file_path,
                              const std::map<std::string, int> &name_map,
                              std::map<int, std::vector<int>> &id2rank) {
  std::ifstream infile(file_path.c_str());

  std::string line;
  std::set<std::string> missing_image_names;
  while (getline(infile, line)) {
    if (line == "") continue;
    std::istringstream s1(line);
    std::string image_name1, image_name2;
    s1 >> image_name1 >> image_name2;
    if (name_map.count(image_name1) == 0) {  //
      // printf("Warning : missing %s in name map\n",image_name1.c_str());
      missing_image_names.insert(image_name1);
      continue;
    }
    if (name_map.count(image_name2) == 0) {
      missing_image_names.insert(image_name2);
      continue;
    }
    const int id1 = name_map[image_name1], id2 = name_map[image_name2];
    id2rank[id1].emplace_back(id2);
  }

  for (const auto name : missing_image_names) {
    printf("Warning : missing %s in name map\n", name.c_str());
  }
}

inline void SaveImageSize(const std::string &file_name, const std::vector<ImageSize> &image_size) {
  std::ofstream file(file_name, std::ios::out | std::ios::binary);
  if (!file.is_open()) {
    std::cerr << "Error: can not open " << file_name << "\n";
    return;
  }
  int num_frames = image_size.size();
  write_data(file, num_frames);
  for (auto &size : image_size) {
    write_data(file, size.width);
    write_data(file, size.height);
  }
}

