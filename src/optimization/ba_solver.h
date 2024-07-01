//
// Created by SENSETIME\yezhichao1 on 2020/10/21.
//

#ifndef XRSFM_SRC_OPTIMIZATION_BA_SOLVER_H
#define XRSFM_SRC_OPTIMIZATION_BA_SOLVER_H

#include <ceres/ceres.h>

#include "base/map.h"
#include "utility/timer.h"

namespace xrsfm {

struct BaOptions {
    bool fix_camera_parameter = true;
    bool fix_frame_poses = false;
    bool is_accurate_mode = true;
};

class BASolver {
  public:
    BASolver() {}

    void ScalePoseGraphUnorder(const LoopInfo &loop_info, Map &map,
                               bool use_key = false);
    void ScalePoseGraphUnorder1(const LoopInfo &loop_info, Map &map);
    void KGBA(Map &map, const std::vector<int> fix_key_frame_ids,
              const bool is_sequential_data);
    void GBA(Map &map, BaOptions ba_options = BaOptions());
    void LBA(int frame_id, Map &map);

  private:
    ceres::Solver::Options InitSolverOptions();
    void SetUp(ceres::Problem &problem, Map &map, Frame &frame);
    void SetUpLBA(ceres::Problem &problem, Map &map, Frame &frame,
                  int frame_id);
};
} // namespace xrsfm
#endif // XRSFM_SRC_OPTIMIZATION_BA_SOLVER_H
