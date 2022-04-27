//
// Created by yzc on 19-4-2.
//

#ifndef WSFM_PNP_H
#define WSFM_PNP_H

#include <Eigen/Eigen>
#include <vector>

#include "base/map.h"

bool ComputeRegisterInlierLoop(const int next_frame_id, LoopInfo &loop_info, Map &map);

bool RegisterNextImageLoop(const int next_frame_id, LoopInfo &loop_info, Map &map);

bool RegisterImage(const int next_frame_id, Map &map);

bool RegisterNextImage(const int next_frame_id, Map &map, Pose &tcw, std::vector<std::pair<int, int>> &cor_2d_3d_ids);

bool RegisterNextImageLocal(const int next_frame_id, const std::set<int> cor_set, Map &map, Pose &tcw,
                            std::vector<std::pair<int, int>> &cor_2d_3d_ids);

int RegisterNextImage1(const int next_frame_id, Map &map);

bool RegisterNextImageLocal(const int next_frame_id, const std::set<int> cor_set, Map &map);

// bool SolvePnP(Camera cam, std::vector<Eigen::Vector2d> cor_points2ds, std::vector<Eigen::Vector3d> cor_points3ds,
//               Frame &frame, std::vector<int> &inlier);

bool SolvePnP_colmap(const std::vector<Eigen::Vector2d> &cor_points2ds,
                     const std::vector<Eigen::Vector3d> &cor_points3ds, double max_error, Pose &tcw,
                     std::vector<char> &inlier_mask);

#endif  // WSFM_PNP_H
