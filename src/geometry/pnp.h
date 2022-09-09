//
// Created by yzc on 19-4-2.
//

#ifndef XRSFM_SRC_GEOMETRY_PNP_H
#define XRSFM_SRC_GEOMETRY_PNP_H

#include <Eigen/Eigen>
#include <vector>

#include "base/map.h"

namespace xrsfm {
bool ComputeRegisterInlierLoop(const int next_frame_id, LoopInfo &loop_info,
                               Map &map);

bool RegisterNextImageLoop(const int next_frame_id, LoopInfo &loop_info,
                           Map &map);

bool RegisterImage(const int next_frame_id, Map &map);

bool RegisterNextImage(const int next_frame_id, Map &map, Pose &tcw,
                       std::vector<std::pair<int, int>> &cor_2d_3d_ids);

bool RegisterNextImageLocal(const int next_frame_id,
                            const std::set<int> cor_set, Map &map, Pose &tcw,
                            std::vector<std::pair<int, int>> &cor_2d_3d_ids);

int RegisterNextImage1(const int next_frame_id, Map &map);

bool RegisterNextImageLocal(const int next_frame_id,
                            const std::set<int> cor_set, Map &map);

bool SolvePnP_colmap(const std::vector<Eigen::Vector2d> &cor_points2ds,
                     const std::vector<Eigen::Vector3d> &cor_points3ds,
                     double max_error, Pose &tcw,
                     std::vector<char> &inlier_mask);
} // namespace xrsfm
#endif // XRSFM_SRC_GEOMETRY_PNP_H
