//
// Created by yzc on 19-4-2.
//

#ifndef XRSFM_SRC_GEOMETRY_PNP_H
#define XRSFM_SRC_GEOMETRY_PNP_H

#include <Eigen/Eigen>
#include <vector>

#include "base/map.h"

namespace xrsfm {
bool RegisterImage(const int next_frame_id, Map &map);

bool RegisterImageLocal(const int next_frame_id, const std::set<int> cor_set,
                        Map &map);

bool SolvePnP_colmap(const std::vector<Eigen::Vector2d> &cor_points2ds,
                     const std::vector<Eigen::Vector3d> &cor_points3ds,
                     double max_error, Pose &tcw,
                     std::vector<char> &inlier_mask);
} // namespace xrsfm
#endif // XRSFM_SRC_GEOMETRY_PNP_H
