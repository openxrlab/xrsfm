//
// Created by yzc on 19-4-2.
//

#ifndef XRSFM_SRC_GEOMETRY_PNP_H
#define XRSFM_SRC_GEOMETRY_PNP_H

#include <Eigen/Eigen>
#include <vector>

#include "base/map.h"

namespace xrsfm {

struct RegisterResult {
    bool success = false;
    int frame_id;
    Pose tcw;
    std::vector<std::pair<int, int>> correspondences_2d3d;
    int num_inlier = 0;
    int num_correspondences = 0;
};

bool RegisterImage(const int next_frame_id, Map &map);

bool RegisterImageLocal(const int next_frame_id, const std::set<int> cor_set,
                        Map &map);

RegisterResult RegisterImageLocal1(const int frame_id,
                                   const std::set<int> references, Map &map);

} // namespace xrsfm

#endif // XRSFM_SRC_GEOMETRY_PNP_H
