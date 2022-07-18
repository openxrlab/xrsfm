//
// Created by SENSETIME\yezhichao1 on 2020/4/28.
//

#ifndef WARG_LINEAR_SOLVER_H
#define WARG_LINEAR_SOLVER_H

#include "ba_block.h"

// clang-format off
#include <ceres/ceres.h>
#include "ceres/compressed_row_sparse_matrix.h"
#include "ceres/sparse_cholesky.h"
#include "ceres/triplet_sparse_matrix.h"
// clang-format on

namespace mgba {

#define residual_size 3
using RES_BLOCK = matrix<residual_size, 1>;
using JAC_BLOCK_CAM = matrix<residual_size, 6>;
using JAC_BLOCK_POINT = matrix<residual_size, 3>;
using SparseBlockStorage = std::vector<std::map<int, matrix6>>;

class linear_solver_refine {
 public:
  void init(int num_res, int num_cam, int num_point);

  void solve_linear_problem(double radius);

  void compute_cost(double &cost);

  void compute_model_cost_change();

  void compute_relative_decrease();

  void compute_step_norm_x_norm(double &step_norm, double &x_norm);

  void param_plus_delta();

  void param_update();

  void log();

  double model_cost_change_ = 0.0;
  double current_cost_ = 0.0;
  double candidate_cost_ = 0.0;
  double relative_decrease_ = 0.0;

  double radius_ = 0;

  using CamraId = int;
  using PointId = int;
  using ResidualId = int;

  int num_res_, num_cam_, num_point_;
  std::vector<std::pair<CamraId, PointId>> edge_vec;
  std::vector<ParamBlock<default_float>> param_blocks;
  std::vector<ResidualBlock<default_float>> residual_blocks;

  // Storage Data Association
  std::vector<bool> const_cam_flags;
  std::vector<bool> const_pt_flags;
  std::vector<ResidualId> cam_res_size_;
  std::vector<std::vector<ResidualId>> pid2rid_;
  std::vector<std::vector<ResidualId>> pid2rid_nonconst_;
  // cam_pair2res_pair_[i][j] corresponding the BLOCK_ij and store the residual pairs
  std::vector<std::map<CamraId, std::vector<std::pair<ResidualId, ResidualId>>>> cam_pair2res_pair_;

  // record real block info(removing const block)
  int num_block_row_;
  std::vector<int> id_origin2real;


  std::vector<RES_BLOCK> residuals;
  std::vector<JAC_BLOCK_CAM> jc_blocks;
  std::vector<JAC_BLOCK_POINT> jp_blocks;
  std::vector<vector3> factors_v3;
  std::vector<vector4> factors_v4;
  std::vector<vector5> factors_v5;

  std::vector<matrix6x3> etfs;
  std::vector<matrix6x3> etf_etei_s;
  std::vector<matrix3> etes;
  std::vector<vector3> etbs;

  vectorX x_f;
  vectorX x_e;

  SparseBlockStorage real_lhs;
  vectorX real_rhs;
  vectorX real_dx;

  std::string message;
  std::unique_ptr<ceres::internal::SparseCholesky> sparse_cholesky_;

  void linearization();
  void schur_complement();
  void compute_delta();

  int solver_type = 1;//0 dense 1 sparse 2 pcg
};

}  // namespace mgba

#endif  // WARG_LINEAR_SOLVER_H
