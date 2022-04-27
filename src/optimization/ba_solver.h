//
// Created by SENSETIME\yezhichao1 on 2020/10/21.
//

#ifndef ECIM_BA_SOLVER_H
#define ECIM_BA_SOLVER_H

#include <ceres/ceres.h>

#include "base/map.h"
#include "utility/timer.h"

class BASolver {
 public:
  BASolver() {
    lba1 = Timer("Time lba1 %.6lfs\n");
    lba2 = Timer("Time lba2 %.6lfs\n");
    lba3 = Timer("Time lba3 %.6lfs\n");
  }

  void ScalePoseGraphUnorder(const LoopInfo &loop_info, Map &map, bool use_key = false);
  void KGBA(Map &map, const std::vector<int> fix_key_frame_ids, const bool order_frames);
  void GBA(Map &map, bool accurate = true, bool fix_all_frames = false);
  void LBA(int frame_id, Map &map);

  Timer lba1, lba2, lba3;
  bool set_all_pose_const = false;
  int num_frames_set_const_ = -1;

 private:
  ceres::Solver::Options InitSolverOptions();
  void SetUp(ceres::Problem &problem, Map &map, Frame &frame);
  void SetUpLBA(ceres::Problem &problem, Map &map, Frame &frame, int frame_id);
};

#endif  // ECIM_BA_SOLVER_H
