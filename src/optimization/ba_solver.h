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
  BASolver() {}

  void ScalePoseGraphUnorder(const LoopInfo &loop_info, Map &map, bool use_key = false);
  void KGBA(Map &map, const std::vector<int> fix_key_frame_ids, const bool order_frames); 
  void GBA(Map &map, bool accurate = true, bool fix_all_frames = false); 
  void LBA(int frame_id, Map &map); 

 private:
  ceres::Solver::Options InitSolverOptions();
  void SetUp(ceres::Problem &problem, Map &map, Frame &frame);
  void SetUpLBA(ceres::Problem &problem, Map &map, Frame &frame, int frame_id);
};

#endif  // ECIM_BA_SOLVER_H
