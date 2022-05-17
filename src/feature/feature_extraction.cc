//
// Created by yzc on 19-4-8.
//

#include <x86intrin.h>

#include <algorithm>
#include <numeric>

#include "feature_processing.h"
#include "geometry/colmap/estimators/fundamental_matrix.h"
#include "geometry/essential.h"
#include "base/map.h"
#include "optim/loransac.h"
#include "optim/ransac.h"
#include "orb_extractor.h"
#include "sift_extractor.h"
#include "utility/io_ecim.hpp"
#include "utility/timer.h"

void FeatureExtract(const std::string &dir_path, const Camera &camera,
                    const std::vector<cv::Mat> &images, std::vector<Frame> &frames) {
#ifndef USE_ORB
  SiftExtractor sift(8192);
#else
  const int _feature_num = 2048;
  const float _lever_ratio = 1.2;
  const int _lever_num = 8;
  const int _initTh = 20;
  const int _minTh = 7;
  ORB_SLAM2::OrbExtractor orb(_feature_num, _lever_ratio, _lever_num, _initTh, _minTh);
#endif

  int id = 0;
  for (const auto &image : images) {
    Frame frame;
    frame.id = id++;
#ifndef USE_ORB
    sift.ExtractUINT8(image, frame.keypoints_, frame.uint_descs_);
#else
    orb(image, cv::Mat(), frame.keypoints_, frame.orb_descs_);
#endif

    frame.points_normalized.clear();
    size_t num_points = frame.keypoints_.size();
    for (size_t i = 0; i < num_points; ++i) {
      const auto &pt = frame.keypoints_[i].pt;
      frame.points.emplace_back(pt.x, pt.y);
      frame.points_normalized.emplace_back((pt.x - camera.cx()) / camera.fx(),
                                           (pt.y - camera.cy()) / camera.fy());
    }
    frame.track_ids_.resize(num_points, -1);
    frames.emplace_back(frame);
    printf("#feature(%d): %d\n", frame.id, num_points);
    // DrawFeature(image,frame.keypoints_);
  }
}

void FeatureExtract(const std::vector<cv::Mat> &images, std::vector<Frame> &frames) {
#ifndef USE_ORB
  const int _feature_num = 8192;
  std::cout << "FeatureExtract: _feature_num = " << _feature_num << "\n";
  SiftExtractor sift(_feature_num);
#else
  const int _feature_num = 2048;
  const float _lever_ratio = 1.2;
  const int _lever_num = 8;
  const int _initTh = 20;
  const int _minTh = 7;
  ORB_SLAM2::OrbExtractor orb(_feature_num, _lever_ratio, _lever_num, _initTh, _minTh);
#endif
  assert(images.size() == frames.size());
  for (int i = 0; i < images.size(); i++) {
    const auto &image = images[i];
    Frame &frame = frames[i];
    frame.keypoints_.clear();
#ifndef USE_ORB
    sift.ExtractUINT8(image, frame.keypoints_, frame.uint_descs_);
    std::cout << i << " " << frame.uint_descs_.rows() << std::endl;
#else
    orb(image, cv::Mat(), frame.keypoints_, frame.orb_descs_);
#endif
    // frame.points_normalized.clear();
    // size_t num_points = frame.keypoints_.size();
    // for (size_t j = 0; j < num_points; ++j) {
    //   const auto &pt = frame.keypoints_[j].pt;
    //   frame.points.emplace_back(pt.x, pt.y);
    // }
    // frame.track_ids_.resize(num_points, -1);
  }
}

void FeatureExtract(const std::string &image_dir_path, std::vector<Frame> &frames) {
#ifndef USE_ORB
  const int _feature_num = 8192;
  std::cout << "FeatureExtract:  feature number = " << _feature_num << "\n";
  SiftExtractor sift(_feature_num);
#else
  const int _feature_num = 2048;
  const float _lever_ratio = 1.2;
  const int _lever_num = 8;
  const int _initTh = 20;
  const int _minTh = 7;
  ORB_SLAM2::OrbExtractor orb(_feature_num, _lever_ratio, _lever_num, _initTh, _minTh);
#endif
  for (int i = 0; i < frames.size(); i++) {
    Frame &frame = frames[i];
    frame.keypoints_.clear();
    if (frame.name == "140983607_f7b7142b1b_o.jpg") continue;
    const cv::Mat image = cv::imread(image_dir_path + frame.name);
    if (image.rows == 0) {
      std::cout << "Can't read " << image_dir_path + frame.name << std::endl;
      exit(0);
    }
#ifndef USE_ORB
    std::cout << image_dir_path + frame.name << std::endl;
    sift.ExtractUINT8(image, frame.keypoints_, frame.uint_descs_);
    std::cout << i << " " << frame.uint_descs_.rows() << std::endl;
#else
    orb(image, cv::Mat(), frame.keypoints_, frame.orb_descs_);
#endif
  }
}

void FeatureExtractMixData(const std::vector<Camera> &cameras,
                           std::map<int, int> &seqindex2cameraindex,
                           const std::vector<cv::Mat> &images, std::vector<Frame> &frames) {
#ifndef USE_ORB
  SiftExtractor sift(2048);
#else
  const int _feature_num = 2048;
  const float _lever_ratio = 1.2;
  const int _lever_num = 8;
  const int _initTh = 20;
  const int _minTh = 7;
  ORB_SLAM2::OrbExtractor orb(_feature_num, _lever_ratio, _lever_num, _initTh, _minTh);
#endif

  int id = 0;
  for (const auto &image : images) {
    Frame frame;
    Camera camera = cameras[seqindex2cameraindex[id]];
    frame.id = id++;
    frame.camera_id = seqindex2cameraindex[id];
#ifndef USE_ORB
    sift.ExtractUINT8(image, frame.keypoints_, frame.uint_descs_);
#else
    orb(image, cv::Mat(), frame.keypoints_, frame.orb_descs_);
#endif

    frame.points_normalized.clear();
    size_t num_points = frame.keypoints_.size();
    for (size_t i = 0; i < num_points; ++i) {
      const auto &pt = frame.keypoints_[i].pt;
      frame.points.emplace_back(pt.x, pt.y);
      frame.points_normalized.emplace_back((pt.x - camera.cx()) / camera.fx(),
                                           (pt.y - camera.cy()) / camera.fy());
    }
    frame.track_ids_.resize(num_points, -1);
    frames.emplace_back(frame);
  }
}
