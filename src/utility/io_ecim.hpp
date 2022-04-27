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

using Clock = std::chrono::high_resolution_clock;

inline double tloop(const Clock::time_point &t_start, const Clock::time_point &t_end) {
  return std::chrono::duration_cast<std::chrono::duration<double>>(t_end - t_start).count();
}

inline void SaveCameras(const std::string &file_name, const std::vector<Camera> &cameras) {
  std::ofstream file(file_name, std::ios::out | std::ios::binary);
  size_t num_camera = cameras.size();
  write_data(file, num_camera);
  for (const auto &camera : cameras) {
    write_data(file, camera.camera_model);
    write_data(file, camera.camera_params);
    write_data(file, camera.distort_params);
  }
  file.close();
}

inline void ReadCameras(const std::string &file_name, std::vector<Camera> &cameras) {
  std::ifstream file(file_name, std::ios::in | std::ios::binary);
  CHECK(file.is_open()) << file_name;
  size_t num_camera;
  read_data(file, num_camera);
  cameras.resize(num_camera);
  for (auto &camera : cameras) {
    read_data(file, camera.camera_model);
    read_data(file, camera.camera_params);
    read_data(file, camera.camera_model);
  }
  for (size_t i = 0; i < 4; ++i) {
    std::cout << cameras[0].camera_params[i] << " ";
  }
  std::cout << std::endl;
}

inline void SaveFrames(const std::string &file_name, const std::vector<Frame> &frames) {
  std::ofstream file(file_name, std::ios::out | std::ios::binary);
  size_t num_frames = frames.size();
  write_data(file, num_frames);
  for (const auto &frame : frames) {
    write_data(file, frame.id);
    write_data(file, frame.Tcw.q);
    write_data(file, frame.Tcw.t);
    size_t num_points = frame.points.size();
    //        std::cout<<num_points<<std::endl;
    write_data(file, num_points);
    write_data_vec(file, &(frame.points[0]), num_points);
    write_data_vec(file, &(frame.points_normalized[0]), num_points);
    write_data_vec(file, &(frame.track_ids_[0]), num_points);
  }
  file.close();
}

inline void ReadFrames(const std::string &file_name, std::vector<Frame> &frames) {
  std::ifstream file(file_name, std::ios::in | std::ios::binary);
  CHECK(file.is_open()) << file_name;
  size_t num_frames = read_data2<size_t>(file);
  frames.resize(num_frames);
  for (auto &frame : frames) {
    read_data(file, frame.id);
    read_data(file, frame.Tcw.q);
    read_data(file, frame.Tcw.t);
    size_t num_points = read_data2<size_t>(file);
    // std::cout << frame.id<<" "<<num_points << std::endl;
    frame.points.resize(num_points);
    frame.points_normalized.resize(num_points);
    frame.track_ids_.resize(num_points);
    read_data_vec(file, &(frame.points[0]), num_points);
    read_data_vec(file, &(frame.points_normalized[0]), num_points);
    read_data_vec(file, &(frame.track_ids_[0]), num_points);
  }
  file.close();
}

inline void SaveMap(const std::string &path, const Map &map) {
  printf("Store Map ...\n");

  auto t_start = Clock::now();
  SaveCameras(path + "map_cameras.bin", map.cameras_);
  auto t_end = Clock::now();
  printf("Time %.3lfs\n", tloop(t_start, t_end));

  t_start = Clock::now();
  SaveFrames(path + "map_frames.bin", map.frames_);
  t_end = Clock::now();
  printf("Time %.3lfs\n", tloop(t_start, t_end));

  t_start = Clock::now();
  SaveFramePairs(path + "map_pairs.bin", map.frame_pairs_);
  t_end = Clock::now();
  printf("Time %.3lfs\n", tloop(t_start, t_end));

  printf("Done!\n");
}

inline void LoadMap(const std::string &path, Map &map) {
  printf("Loading Map ...\n");

  auto t_start = Clock::now();
  ReadCameras(path + "map_cameras.bin", map.cameras_);
  auto t_end = Clock::now();
  //    printf("Time %.3lfs\n", tloop(t_start, t_end));

  t_start = Clock::now();
  ReadFrames(path + "map_frames.bin", map.frames_);
  t_end = Clock::now();
  //    printf("Time %.3lfs\n", tloop(t_start, t_end));

  t_start = Clock::now();
  ReadFramePairs(path + "map_pairs.bin", map.frame_pairs_);
  t_end = Clock::now();
  //    printf("Time %.3lfs\n", tloop(t_start, t_end));

  printf("%zu Frames Done!\n", map.frames_.size());
}

inline void ReadImage(const std::string &dir_path, const int step, std::vector<cv::Mat> &images) {
  std::string filename = dir_path + "rgb.txt";
  std::vector<std::string> image_names;
  std::ifstream file(filename);
  CHECK(file.is_open());
  std::string line;
  int count = 0;
  while (std::getline(file, line)) {
    if (line[0] == '#') continue;
    std::stringstream ss(line);
    double timestamp = 0;
    std::string name;
    if (!(ss >> timestamp >> name)) {
      break;
    }
    if (count++ % step == 0) {
      image_names.emplace_back(name);
      cv::Mat image = cv::imread(dir_path + name);
      images.push_back(image);
    }
  }
  file.close();
}

inline void ReadAssociate(const std::string &dir_path, const int step, std::vector<double> &time_vec,
                          std::vector<cv::Mat> &images) {
  std::string filename = dir_path + "associate.txt";
  std::vector<std::string> image_names;
  std::ifstream file(filename);
  CHECK(file.is_open());
  std::string line;
  int count = 0;
  while (std::getline(file, line)) {
    if (line[0] == '#') continue;
    std::stringstream ss(line);
    double timestamp_rgb = 0;
    std::string name_rgb;
    double timestamp_dpt = 0;
    std::string name_dpt;
    if (!(ss >> timestamp_rgb >> name_rgb >> timestamp_dpt >> name_dpt)) break;
    if (count++ % step == 0) {
      time_vec.emplace_back(timestamp_rgb);
      image_names.emplace_back(name_rgb);
      cv::Mat image = cv::imread(dir_path + name_rgb);
      images.emplace_back(image);
    }
  }
  file.close();
}

inline void LoadImageKITTI(const std::string &dir_path, const int step, std::vector<cv::Mat> &images) {
  images.clear();
  char image_path[512];
  const int max_num = 100000;
  for (int i = 0; i < max_num; i += step) {
    sprintf(image_path, (dir_path + "%06d.png").c_str(), i);
    cv::Mat image = cv::imread(image_path);
    if (image.empty()) break;
    images.emplace_back(image);
  }
}

inline void OutputTumResult(const std::string &bin_path, const std::vector<double> &time_vec, Map &map) {
  std::ofstream outfile;
  outfile.open(bin_path + "result.txt");
  outfile.setf(std::ios::fixed, std::ios::floatfield);
  for (int id = 0; id < time_vec.size(); ++id) {
    if (!map.frames_[id].registered) continue;
    outfile << time_vec[id];
    const auto &pose = map.frames_[id].Tcw;
    Eigen::Quaterniond qwc = pose.q.inverse();
    Eigen::Vector3d twc = -(pose.q.inverse() * pose.t);
    for (int i = 0; i < 3; ++i) outfile << " " << twc(i);
    for (int i = 0; i < 4; ++i) outfile << " " << qwc.coeffs()(i);
    outfile << std::endl;
  }
  outfile.close();
}

inline void SaveSfMState(const std::string &file_name, const int &iter, Map &map) {
  std::ofstream file(file_name, std::ios::out | std::ios::binary);
  write_data(file, iter);

  for (const auto &frame : map.frames_) {
    int tmp = frame.registered ? 1 : 0;
    write_data(file, tmp);
    tmp = frame.is_keyframe ? 1 : 0;
    write_data(file, tmp);
    write_data(file, frame.num_neighbors_registered);
    write_data(file, frame.num_visible_points3D_);
    write_data(file, frame.hierarchical_level);
    write_data(file, frame.Tcw.q);
    write_data(file, frame.Tcw.t);
    size_t num_points = frame.track_ids_.size();
    write_data_vec(file, &(frame.track_ids_[0]), num_points);
    write_data_vec(file, &(frame.num_correspondences_have_point3D_[0]), num_points);
  }

  size_t num_track = map.tracks_.size();
  write_data(file, num_track);
  for (const auto &track : map.tracks_) {
    int tmp = track.outlier ? 1 : 0;
    write_data(file, tmp);
    tmp = track.is_keypoint ? 1 : 0;
    write_data(file, tmp);
    write_data(file, track.point3d_);
    size_t num_obs = track.observations_.size();
    write_data(file, num_obs);
    for (const auto &[t_frame_id, t_p2d_id] : track.observations_) {
      write_data(file, t_frame_id);
      write_data(file, t_p2d_id);
    }
  }

  size_t num_covisible = map.frameid2covisible_frameids_.size();
  write_data(file, num_covisible);
  for (auto &[t_frame_id, frame_ids] : map.frameid2covisible_frameids_) {
    write_data(file, t_frame_id);
    size_t num_vec = frame_ids.size();
    write_data(file, num_vec);
    for (auto &id : frame_ids) {
      write_data(file, id);
    }
  }
  file.close();
}

inline void LoadSfMState(const std::string &file_name, int &iter, Map &map) {
  std::ifstream file(file_name, std::ios::in | std::ios::binary);
  CHECK(file.is_open());
  if(file.is_open()){
    std::cout<<"ok";
  }
  read_data(file, iter);

  for (auto &frame : map.frames_) {
    int tmp = 0;
    read_data(file, tmp);
    frame.registered = tmp == 1;
    read_data(file, tmp);
    frame.is_keyframe = tmp == 1;
    read_data(file, frame.num_neighbors_registered);
    read_data(file, frame.num_visible_points3D_);
    read_data(file, frame.hierarchical_level);
    read_data(file, frame.Tcw.q);
    read_data(file, frame.Tcw.t);
    // std::cout<<frame.Tcw.t.transpose()<<std::endl;
    size_t num_points = frame.track_ids_.size();
    read_data_vec(file, &(frame.track_ids_[0]), num_points);
    read_data_vec(file, &(frame.num_correspondences_have_point3D_[0]), num_points);
  }

  size_t num_track = 0;
  read_data(file, num_track);
  std::cout<<num_track<<std::endl;
  map.tracks_.resize(num_track);
  for (auto &track : map.tracks_) {
    int tmp = 0;
    read_data(file, tmp);
    track.outlier = tmp == 1;
    read_data(file, tmp);
    track.is_keypoint = tmp == 1;
    read_data(file, track.point3d_);
    size_t num_obs;
    read_data(file, num_obs);
    for (int i = 0; i < num_obs; ++i) {
      int frame_id = 0, p2d_id = 0;
      read_data(file, frame_id);
      read_data(file, p2d_id);
      track.observations_[frame_id] = p2d_id;
    }
  }

  size_t num_covisible = 0;
  read_data(file, num_covisible);
  for (int i = 0; i < num_covisible; ++i) {
    int frame_id = 0;
    std::vector<int> vec_frame_id;
    read_data(file, frame_id);
    size_t num_vec;
    read_data(file, num_vec);
    vec_frame_id.resize(num_vec);
    for (auto &id : vec_frame_id) {
      read_data(file, id);
    }
    map.frameid2covisible_frameids_[frame_id] = vec_frame_id;
  }
}

inline void LoadTum(std::string filename, std::vector<double> &time_vec, std::vector<Pose> &pose_vec) {
  time_vec.clear();
  pose_vec.clear();
  std::ifstream file(filename);
  CHECK(file.is_open());
  std::string line;
  while (std::getline(file, line)) {
    if (line[0] == '#') continue;
    std::stringstream ss(line);
    double time = 0;
    char tmp;
    Pose pose;
    ss >> time >> pose.t.x() >> pose.t.y() >> pose.t.z();
    ss >> pose.q.x() >> pose.q.y() >> pose.q.z() >> pose.q.w();
    //        ss >> time >> tmp >> pose.t.x() >> tmp >> pose.t.y() >> tmp >>
    //        pose.t.z() >> tmp; ss >> pose.q.x() >> tmp >> pose.q.y() >> tmp >>
    //        pose.q.z() >> tmp >> pose.q.w();
    time_vec.emplace_back(time);
    pose_vec.emplace_back(pose);
  }
  file.close();
}

inline void SaveTum(std::string filename, std::vector<double> &time_vec, std::vector<Pose> &pose_vec) {
  std::ofstream file(filename);
  for (int i = 0; i < pose_vec.size(); ++i) {
    auto &time = time_vec[i];
    auto &pose = pose_vec[i];
    file << time << " " << pose.t.x() << " " << pose.t.y() << " " << pose.t.z() << " ";
    file << pose.q.x() << " " << pose.q.y() << " " << pose.q.z() << " " << pose.q.w() << "\n";
  }
  file.close();
}

inline void LoadTwc(std::string filename, std::vector<Pose> &pose_vec) {
  pose_vec.clear();
  std::ifstream file(filename);
  CHECK(file.is_open());
  std::string line;
  while (std::getline(file, line)) {
    if (line[0] == '#') continue;
    std::stringstream ss(line);
    Eigen::Matrix<double, 3, 4, Eigen::RowMajor> Twc;
    for (int i = 0; i < 12; ++i) {
      ss >> Twc(i);
    }
    Eigen::Matrix3d R = Twc.leftCols<3>();
    Eigen::Vector3d t = Twc.rightCols<1>();
    //        std::cout<<Twc<<std::endl;
    pose_vec.emplace_back(Pose(Eigen::Quaterniond(R), t));
  }
  file.close();
}

inline void ReadCameraColmapTXT(const std::string &file_name, std::vector<Camera> &cameras) {
  cameras.clear();
  std::ifstream file(file_name);
  CHECK(file.is_open()) << file_name;
  std::string line;
  while (std::getline(file, line)) {
    if (line[0] == '#') continue;
    std::stringstream ss(line);
    int id, width, height;
    //        char model_name[128];
    std::string model_name;
    double f, cx, cy, k;
    ss >> id >> model_name >> width >> height >> f >> cx >> cy >> k;
    Camera camera;
    camera.id = id;
    camera.camera_params = {f, f, cx, cy};
    camera.distort_params = {k, 0, 0, 0, 0};
    cameras.emplace_back(camera);
  }
}

inline void ReadImageColmapTXT(const std::string &file_name, std::map<int, std::string> &image_names) {
  std::ifstream file(file_name);
  CHECK(file.is_open());
  std::string line;
  while (std::getline(file, line)) {
    if (line[0] == '#') continue;
    std::stringstream ss(line);
    int id, c_id;
    Pose p;
    std::string name;
    ss >> id;
    ss >> p.q.w() >> p.q.x() >> p.q.y() >> p.q.z() >> p.t(0) >> p.t(1) >> p.t(2) >> c_id >> name;
    image_names[id] = name;
    std::getline(file, line);
  }
}

inline void ReadCamerasColmap(const std::string &file_name, std::vector<Camera> &cameras) {
  std::ifstream file(file_name, std::ios::in | std::ios::binary);
  CHECK(file.is_open());
  size_t num_camera;
  read_data(file, num_camera);
  cameras.resize(num_camera);
  int count = 0;
  for (auto &camera : cameras) {
    int id = 0, model_id = 0, flag = 0;
    read_data(file, id);
    read_data(file, model_id);
    size_t num_param = 0;
    read_data(file, num_param);
    std::vector<double> params(num_param);
    read_data_vec(file, params.data(), num_param);
    read_data(file, flag);
    if (model_id == 2) {
      camera.id = id;
      camera.camera_model = Camera::CameraModel::SIMPLE_RADIAL;
      CHECK(num_param == 4);
      camera.camera_params = {params[0], params[0], params[1], params[2]};
      camera.distort_params = {params[3], 0, 0, 0, 0};
    }
    count++;
    // printf("%d %d %lf %lf %lf %lf %d\n", id, count, camera.fx(), camera.cx(), camera.cy(),
    //        camera.distort_params[0], flag);
  }
  std::cout << std::endl;
}

inline void LoadTimeStamp(const std::string timestamp_path, std::vector<double> &timestamp_vec) {
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

inline void UpdateFrameTimeStamp(std::vector<Frame> &frames, std::vector<double> &timestamp_Vec) {
  int step = int(1.0 * timestamp_Vec.size() / frames.size() + 0.1);
  printf("%d %zu %zu\n", step, timestamp_Vec.size(), frames.size());
  for (auto &f : frames) {
    f.time = timestamp_Vec[step * f.id];
  }
  return;
}

inline void ReadExtraColmap(const std::string &file_name, std::vector<Frame> &frames,
                            std::map<int, std::string> &image_names) {
  image_names.clear();
  std::ifstream file(file_name, std::ios::in | std::ios::binary);
  CHECK(file.is_open());
  size_t num_frames = read_data2<size_t>(file);
  CHECK(frames.size() == num_frames);
  for (size_t i = 0; i < num_frames; ++i) {
    int id1, id2;
    char buf[128];
    read_data(file, id1);
    read_data(file, id2);
    size_t num_name = read_data2<size_t>(file);
    read_data_vec(file, buf, num_name);
    buf[num_name] = '\0';
    std::string name = buf;
    // printf("%d %s %s %zu\n", i, name.c_str(), buf, num_name);
    CHECK(frames[i].id == id1);
    frames[i].camera_id = id2;
    image_names[id1] = name;
  }
  file.close();
}

inline void ReadExtraColmap(const std::string &file_name, std::vector<std::tuple<std::string, int, int>> &image_infos) {
  image_infos.clear();
  std::ifstream file(file_name, std::ios::in | std::ios::binary);
  CHECK(file.is_open()) << file_name;
  size_t num_frames = read_data2<size_t>(file);
  for (size_t i = 0; i < num_frames; ++i) {
    int id_image, id_camera;
    char buf[128];
    read_data(file, id_image);
    read_data(file, id_camera);
    size_t num_name = read_data2<size_t>(file);
    read_data_vec(file, buf, num_name);
    buf[num_name] = '\0';
    std::string name = buf;
    image_infos.emplace_back(name, id_image, id_camera);
  }
  file.close();
}

inline void ReadCameraColmapTXT(const std::string &file_name, std::map<int, Camera> &cameras) {
  // cameras.clear();
  std::ifstream file(file_name);
  CHECK(file.is_open()) << file_name;
  std::string line;
  while (std::getline(file, line)) {
    if (line[0] == '#') continue;
    std::stringstream ss(line);
    int id, width, height;
    //        char model_name[128];
    std::string model_name;
    double f, cx, cy, k;
    ss >> id >> model_name >> width >> height >> f >> cx >> cy >> k;
    Camera camera;
    camera.id = id;
    camera.camera_params = {f, f, cx, cy};
    camera.distort_params = {k, 0, 0, 0, 0};
    cameras[id] = camera;
  }
}

inline void WriteTrajectory(const Map &map, const std::string &trajectory_path) {
  std::ofstream trajectory_file(trajectory_path);
  for (auto &frame : map.frames_) {
    if (!frame.registered) continue;
    // if (!frame.is_keyframe) continue;
    Eigen::Vector3d twc = frame.twc();
    Eigen::Quaterniond qwc = frame.qwc();
    trajectory_file << std::to_string(frame.time) << " " << twc[0] << " " << twc[1] << " " << twc[2] << " " << qwc.x()
                    << " " << qwc.y() << " " << qwc.z() << " " << qwc.w() << "\n";
  }
  trajectory_file.close();
}

inline void ReadPoseColmapTXT(const std::string &file_name, std::map<int, Pose> &pose_map) {
  std::ifstream file(file_name);
  CHECK(file.is_open());
  std::string line;
  while (std::getline(file, line)) {
    if (line[0] == '#') continue;
    std::stringstream ss(line);
    int id, c_id;
    Pose p;
    std::string name;
    ss >> id;
    ss >> p.q.w() >> p.q.x() >> p.q.y() >> p.q.z() >> p.t(0) >> p.t(1) >> p.t(2) >> c_id >> name;
    pose_map[id] = p;
    // image_names[id] = name;
    std::getline(file, line);
  }
}

void WriteColMapDataBinary(const std::string &output_path, const Map &map);

void ReadColMapDataBinary(const std::string &output_path, Map &map);