//
// Created by SENSETIME\yezhichao1 on 2020/4/28.
//
#include "linear_solver.h"

#include <Eigen/Eigen>
#include <iostream>

#include "../timer.h"
#include "../type_mg.h"

#include "ceres/block_random_access_diagonal_matrix.h"
#include "ceres/block_random_access_sparse_matrix.h"
#include "ceres/conjugate_gradients_solver.h"

namespace ceres::internal {
using namespace mgba;
class BlockRandomAccessSparseMatrixAdapter : public LinearOperator {
 public:
  explicit BlockRandomAccessSparseMatrixAdapter(const BlockRandomAccessSparseMatrix &m) : m_(m) {}

  virtual ~BlockRandomAccessSparseMatrixAdapter() {}

  // y = y + Ax;
  void RightMultiply(const double *x, double *y) const final { m_.SymmetricRightMultiply(x, y); }

  // y = y + A'x;
  void LeftMultiply(const double *x, double *y) const final { m_.SymmetricRightMultiply(x, y); }

  int num_rows() const final { return m_.num_rows(); }
  int num_cols() const final { return m_.num_rows(); }

 private:
  const BlockRandomAccessSparseMatrix &m_;
};

class BlockRandomAccessDiagonalMatrixAdapter : public LinearOperator {
 public:
  explicit BlockRandomAccessDiagonalMatrixAdapter(const BlockRandomAccessDiagonalMatrix &m) : m_(m) {}

  virtual ~BlockRandomAccessDiagonalMatrixAdapter() {}

  // y = y + Ax;
  void RightMultiply(const double *x, double *y) const final { m_.RightMultiply(x, y); }

  // y = y + A'x;
  void LeftMultiply(const double *x, double *y) const final { m_.RightMultiply(x, y); }

  int num_rows() const final { return m_.num_rows(); }
  int num_cols() const final { return m_.num_rows(); }

 private:
  const BlockRandomAccessDiagonalMatrix &m_;
};

inline void ConvertToBRSM(SparseBlockStorage &lhs, BlockRandomAccessSparseMatrix *sc) {
  const int num_blocks = lhs.size();
  for (int id_row_block = 0; id_row_block < num_blocks; ++id_row_block) {
    for (auto &[id_col_block, block_matrix] : lhs[id_row_block]) {
      int row, col, rs, cs;
      CellInfo *cell_info = sc->GetCell(id_row_block, id_col_block, &row, &col, &rs, &cs);
      mgba::map<mgba::matrix6> cell(cell_info->values);
      cell = block_matrix.transpose();
    }
  }
}
}

namespace mgba {

Timer timer_linearization("Time Linearization %.7lfs\n");
Timer timer_schur_complement("Time schur complement %.7lfs\n");
Timer timer_compute_delta("Time compute delta %.7lfs\n");
Timer timer1("Time 1 %.7lfs\n");
Timer timer2("Time 2 %.7lfs\n");
Timer timer3("Time 3 %.7lfs\n");
std::vector<Timer *> timer_vec = {
    &timer_linearization, &timer_schur_complement, &timer_compute_delta};

 
void solve_linear_system_pcg(SparseBlockStorage &lhs, double *rhs, double *dx) {
  using namespace ceres::internal;
  using MatrixRef = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>;
  using VectorRef = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 1>>;

  std::unique_ptr<BlockRandomAccessSparseMatrix> sc; 
  {
    const int num_blocks = lhs.size();
    std::vector<int> blocks_(num_blocks, 6);
    std::set<std::pair<int, int>> block_pairs;
    for (int id_row_block = 0; id_row_block < num_blocks; ++id_row_block) {
      for (auto &[id_col_block, block_matrix] : lhs[id_row_block]) {
        block_pairs.insert(std::make_pair(id_row_block, id_col_block));
      }
    }
    sc.reset(new BlockRandomAccessSparseMatrix(blocks_, block_pairs));
  }
  ConvertToBRSM(lhs, sc.get());

  const int num_blocks = lhs.size();
  const int num_rows = 6 * num_blocks;
  std::vector<int> blocks_(num_blocks, 6);
  // Size of the blocks in the Schur complement.
  std::unique_ptr<BlockRandomAccessDiagonalMatrix> preconditioner_;
  preconditioner_.reset(new BlockRandomAccessDiagonalMatrix(blocks_));

  // Extract block diagonal from the Schur complement to construct the
  // schur_jacobi preconditioner.
  for (int i = 0; i < blocks_.size(); ++i) {
    const int block_size = blocks_[i];
    int sc_r, sc_c, sc_row_stride, sc_col_stride;
    CellInfo *sc_cell_info = sc->GetCell(i, i, &sc_r, &sc_c, &sc_row_stride, &sc_col_stride);
    CHECK(sc_cell_info != nullptr);
    MatrixRef sc_m(sc_cell_info->values, sc_row_stride, sc_col_stride);
    int pre_r, pre_c, pre_row_stride, pre_col_stride;
    CellInfo *pre_cell_info = preconditioner_->GetCell(i, i, &pre_r, &pre_c, &pre_row_stride, &pre_col_stride);
    CHECK(pre_cell_info != nullptr);
    MatrixRef pre_m(pre_cell_info->values, pre_row_stride, pre_col_stride);
    pre_m.block(pre_r, pre_c, block_size, block_size) = sc_m.block(sc_r, sc_c, block_size, block_size);
  }
  preconditioner_->Invert();

  VectorRef(dx, num_rows).setZero();

  std::unique_ptr<LinearOperator> lhs_adapter(new BlockRandomAccessSparseMatrixAdapter(*sc));
  std::unique_ptr<LinearOperator> preconditioner_adapter(new BlockRandomAccessDiagonalMatrixAdapter(*preconditioner_));

  LinearSolver::Options cg_options;
  cg_options.min_num_iterations = 0;
  cg_options.max_num_iterations = 100;
  ConjugateGradientsSolver cg_solver(cg_options);

  LinearSolver::PerSolveOptions cg_per_solve_options;
  cg_per_solve_options.r_tolerance = -1;
  cg_per_solve_options.q_tolerance = 0.1;
  cg_per_solve_options.preconditioner = preconditioner_adapter.get();

  cg_solver.Solve(lhs_adapter.get(), rhs, cg_per_solve_options, dx);
}

inline void equal_33_l(const default_float *ptr, const matrix3 &H, matrix6x3 &r) {
  // fte*H
  // e = a'x f = A,a'x
  // fte = a'x,-a'x*a'x
  const_map<vector3> a(ptr);
  const default_float h2 = ptr[3];
  for (int i = 0; i < 3; ++i) {
    r.col(i).head<3>().noalias() = a.cross(h2*H.col(i));
    r.col(i).tail<3>().noalias() = -a.cross(a.cross(h2*H.col(i)));
  }
}

inline void sub_63_r(const default_float *ptr, const matrix6x3 &H, matrix6 &r) {
  // -H*etf
  // -etf = a'x,a'x*a'x
  const_map<vector3> a(ptr);
  const default_float h2 = ptr[3];
  for (int i = 0; i < 6; ++i) {
    vector3 tmp = a.cross(h2*H.row(i));
    r.row(i).head<3>().noalias() -= tmp;
    r.row(i).tail<3>().noalias() += a.cross(tmp);
  }
}

inline void sub_61_l(const default_float *ptr, const vector6 &H, vector3 &r) {
  const_map<vector3> a(ptr);
  const default_float h2 = ptr[3];
  r.noalias() += h2*a.cross(H.head<3>() + a.cross(H.tail<3>()));
}

void linear_solver_refine::init(int num_res, int num_cam, int num_point) {
  num_res_ = num_res, num_cam_ = num_cam, num_point_ = num_point;

  factors_v4.assign(num_res_, vector4::Zero());
  factors_v5.assign(num_res_, vector5::Zero());
  residuals.assign(num_res_, RES_BLOCK::Zero());
  etf_etei_s.assign(num_res_, matrix6x3::Zero());
  etes.assign(num_point_, matrix3::Zero());
  etbs.assign(num_point_, vector3::Zero());
  x_f.resize(6 * num_cam_);
  x_e.resize(3 * num_point_);

  num_block_row_ = num_cam_;
  if(const_cam_flags.empty()){
    const_cam_flags.assign(num_cam_, false);
  }else{
    for(auto flag:const_cam_flags){
      if(flag)num_block_row_--;
    }
  }
  id_origin2real.assign(num_cam_, -1);
  int id_block = 0;
  for (int i = 0; i < num_cam_; ++i) {
    if (const_cam_flags[i]) continue;
    id_origin2real[i] = id_block;
    id_block++;
  }
  if(const_pt_flags.empty()){
    const_pt_flags.assign(num_point_, false);
  }

  real_lhs.clear();
  real_lhs.resize(num_block_row_);
  real_rhs.resize(num_block_row_ * 6);
  real_dx.resize(num_block_row_ * 6);

  pid2rid_.assign(num_point_, std::vector<int>(0));
  cam_res_size_.resize(num_cam_ + 1, 0);
  cam_pair2res_pair_.clear();
  cam_pair2res_pair_.resize(num_cam_);

  for (int id_res = 0; id_res < edge_vec.size(); ++id_res) {
    const auto &[id_cam, id_pt] = edge_vec[id_res];
    pid2rid_[id_pt].emplace_back(id_res);
    if ((id_res + 1 == edge_vec.size()) || (id_cam != edge_vec[id_res + 1].first)) {
      cam_res_size_[id_cam + 1] = id_res + 1;
    }
  }

  for (int id_cam1 = 0; id_cam1 < num_cam_; ++id_cam1) {  // TODO speed up
    if (const_cam_flags[id_cam1]) continue;
    const int id_res_b = cam_res_size_[id_cam1], id_res_e = cam_res_size_[id_cam1 + 1];
    for (int id_res1 = id_res_b; id_res1 < id_res_e; ++id_res1) {
      const int id_pt = edge_vec[id_res1].second;
      if(const_pt_flags[id_pt])continue;
      for (auto &id_res2 : pid2rid_[id_pt]) {
        if (id_res2 < id_res1) continue;
        const int id_cam2 = edge_vec[id_res2].first;
        if (const_cam_flags[id_cam2]) continue;
        if (cam_pair2res_pair_[id_cam1].count(id_cam2) == 0)
          cam_pair2res_pair_[id_cam1][id_cam2] = std::vector<std::pair<int, int>>(0);
        cam_pair2res_pair_[id_cam1][id_cam2].emplace_back(std::pair<int, int>(id_res1, id_res2));
      }
    }
  }

  if(num_block_row_>=100){
    solver_type = 2;
  }
  if(solver_type==1){
    using namespace ceres::internal;
    LinearSolver::Options sparse_cholesky_options;
    sparse_cholesky_options.use_postordering = true;
    sparse_cholesky_ = SparseCholesky::Create(sparse_cholesky_options); 
  }
}

void linear_solver_refine::linearization() {
  // for every residual block compute {Jacobian, residual}
  for (int i = 0; i < num_res_; ++i) {
    const auto &rb = residual_blocks[i];
    const std::array<default_float *, 2> factors_ptr = {factors_v4[i].data(), factors_v5[i].data()};
    rb.factor->Evaluate_Fac(rb.param_blocks.data(), residuals[i].data(), factors_ptr.data());
  }
}

void linear_solver_refine::schur_complement() {
  // lhs += sum_i f_i^T * f_i - e_t_f^T * e_t_e_inverse * e_t_f
  // rhs += f_t_b - e_t_f^T * e_t_e_inverse * e_t_b

  // step1: compute ete, etb
  const default_float diag_val = 1.0 / radius_;
  matrix3 diag_radius = matrix3::Zero();
  diag_radius.diagonal().setConstant(diag_val);
  for (int id_pt = 0; id_pt < num_point_; ++id_pt) {
    matrix3 ete = diag_radius;
    vector3 etb = vector3::Zero();
    for (const int id_res : pid2rid_[id_pt]) {
      const_map<vector3> a(factors_v5[id_res].data());
      const default_float s = factors_v5[id_res](3);
      const default_float h = factors_v5[id_res](4);
      const default_float data[] = {1.0 - a(0) * a(0), -a(0) * a(1), -a(0) * a(2), -a(1) * a(0),     1.0 - a(1) * a(1),
                                    -a(1) * a(2),      -a(2) * a(0), -a(2) * a(1), 1.0 - a(2) * a(2)};
      const_map<matrix3> A(data);
      ete.noalias() += (h * h*s *s) * A;//f2
      etb.noalias() += s * h * (residuals[id_res] - (a.dot(residuals[id_res])) * a);// f2
    }
    etes[id_pt] = ete.inverse();
    etbs[id_pt] = etb;
  }

  // step2: compute etf_etei = e_t_f^T * e_t_e_inverse
  for (int id_res = 0; id_res < num_res_; ++id_res) {
    const int id_pt = edge_vec[id_res].second;
    equal_33_l(factors_v4[id_res].data(), etes[id_pt], etf_etei_s[id_res]);
  }
  // step3: compute -etf_etei * e_t_f  & -etf_etei * e_t_b
  // #pragma omp parallel for default(shared) schedule(dynamic, 10)
  for (int id_cam = 0; id_cam < num_cam_; ++id_cam) {
    if (const_cam_flags[id_cam]) continue;
    const int id_row = id_origin2real[id_cam];
    for (const auto &[id_cam2, id_res_pairs] : cam_pair2res_pair_[id_cam]) {
      matrix6 tmp = matrix6::Zero();
      for (const auto &[id_res1, id_res2] : id_res_pairs) {
        sub_63_r(factors_v4[id_res2].data(), etf_etei_s[id_res1], tmp); 
      }
      const int id_col = id_origin2real[id_cam2];
      real_lhs[id_row][id_col] = tmp;
    }
    vector6 tmpv = vector6::Zero();
    const int id_res0 = cam_res_size_[id_cam], id_res1 = cam_res_size_[id_cam + 1];
    for (int id_res = id_res0; id_res < id_res1; ++id_res) {
      const int id_pt = edge_vec[id_res].second;
      if(const_pt_flags[id_pt])continue;
      tmpv.noalias() -= etf_etei_s[id_res] * etbs[id_pt];
    }
    real_rhs.segment<6>(id_row * 6).noalias() = tmpv;
  }

  // step4:  compute ftf, ftb
  for (int id_cam = 0; id_cam < num_cam_; ++id_cam) {
    if (const_cam_flags[id_cam]) continue;
    const int id_res0 = cam_res_size_[id_cam], id_res1 = cam_res_size_[id_cam + 1];
    matrix3 ftf1 = diag_radius, ftf2 = diag_radius;
    vector3 ftf3 = vector3::Zero();
    vector6 ftb = vector6::Zero();
    for (int id_res = id_res0; id_res < id_res1; ++id_res) {
      const_map<vector3> a(factors_v5[id_res].data());
      const default_float &s = factors_v5[id_res](3);
      const default_float &h = factors_v5[id_res](4);
      const default_float data[] = {1 - a(0) * a(0), -a(0) * a(1), -a(0) * a(2), -a(1) * a(0),   1 - a(1) * a(1),
                                    -a(1) * a(2),    -a(2) * a(0), -a(2) * a(1), 1 - a(2) * a(2)};
      const_map<matrix3> A(data);

      ftf1.noalias() += h*h*A;//f2
      ftf2.noalias() += h*h*(s * s) * A;//f2
      ftf3.noalias() += h*h*s * a;//f2

      ftb.head<3>().noalias() += h*a.cross(residuals[id_res]);//f2
      ftb.tail<3>().noalias() += h*s * (residuals[id_res] - a.dot(residuals[id_res]) * a);//f2
    }

    matrix6 ftf;
    ftf.topLeftCorner<3, 3>().noalias() = ftf1;
    ftf.topRightCorner<3, 3>().noalias() = skewSymmetric(ftf3);
    ftf.bottomRightCorner<3, 3>().noalias() = ftf2;
    if(solver_type==2){
      ftf.bottomLeftCorner<3, 3>().noalias() = -skewSymmetric(ftf3);
    }
    const int id_row = id_origin2real[id_cam];
    real_lhs[id_row][id_row] += ftf;
    real_rhs.segment<6>(id_row * 6).noalias() += ftb;
  }
}

using namespace ceres::internal;
inline CompressedRowSparseMatrix *ConvertToCRSM(const int num_block, const SparseBlockStorage &A) {
  const int num_rows = 6 * num_block, num_cols = 6 * num_block;
  int num_nozero_block_ = 0;
  for (int id_row_block = 0; id_row_block < num_block; ++id_row_block) {
    num_nozero_block_ += A[id_row_block].size();
  }
  CompressedRowSparseMatrix *output = new CompressedRowSparseMatrix(num_rows, num_cols, num_nozero_block_ * 36);
  int *output_rows = output->mutable_rows();
  int *output_cols = output->mutable_cols();
  double *output_values = output->mutable_values();

  int id = 0;
  output_rows[0] = 0;
  for (int id_row_block = 0; id_row_block < num_block; ++id_row_block) {
    for (int r = 0; r < 6; ++r) {
      const int row_id = 6 * id_row_block + r;
      for (auto &[id_col_block, block_matrix] : A[id_row_block]) {
        for (int c = 0; c < 6; ++c) {
          const int col_id = 6 * id_col_block + c;
          output_cols[id] = col_id;
          output_values[id] = block_matrix(r, c);
          id++;
        }
      }
      output_rows[row_id + 1] = output_rows[row_id] + A[id_row_block].size() * 6;
    }
  }
  return output;
}

inline void solve_sparse_linear_system(const int num_block, const SparseBlockStorage &lhs, vectorX &rhs, vectorX &dx, SparseCholesky* sparse_cholesky_) {
  std::unique_ptr<CompressedRowSparseMatrix> _lhs;
  _lhs.reset(ConvertToCRSM(num_block, lhs));
  _lhs->set_storage_type(CompressedRowSparseMatrix::UPPER_TRIANGULAR);
  std::string message;
  sparse_cholesky_->FactorAndSolve(_lhs.get(), rhs.data(), dx.data(), &message);  // 1000 4s
}

inline void solve_dense_linear_system(const int num_block, const SparseBlockStorage &lhs, vectorX &rhs, vectorX &dx) {
  matrixX dense_lhs = matrixX::Zero(num_block * 6, num_block * 6);
  for (int id_row_block = 0; id_row_block < num_block; ++id_row_block) {
    for (auto &[id_col_block, block_matrix] : lhs[id_row_block]) {
      dense_lhs.block<6, 6>(id_row_block * 6, id_col_block * 6) = block_matrix;
    }
  }
  Eigen::LLT<matrixX, Eigen::Upper> llt = dense_lhs.selfadjointView<Eigen::Upper>().llt();
  dx = llt.solve(rhs);
}

void linear_solver_refine::compute_delta() {
  // slove Ax = b
  if(solver_type==0){
    solve_dense_linear_system(num_block_row_, real_lhs, real_rhs, real_dx);
  }else if(solver_type==1){
    solve_sparse_linear_system(num_block_row_, real_lhs, real_rhs, real_dx,sparse_cholesky_.get());
  }else if(solver_type==2){
    solve_linear_system_pcg( real_lhs, real_rhs.data(), real_dx.data());
  }
  x_f.setZero();
  int id_real = 0;
  for (int id_cam = 0; id_cam < num_cam_; ++id_cam) {
    if (const_cam_flags[id_cam]) continue;
    x_f.segment<6>(id_cam*6) = real_dx.segment<6>(id_real*6);
    id_real++;
  }

  // x_e_i = e_t_e_inverse * sum_i e_i^T * (b_i - f_i * x_f);
  // x_e_i = e_t_e_inverse * sum_i (e_t_b - e_t_f * x_f);
  for (int id_pt = 0; id_pt < num_point_; ++id_pt) {
    if(const_pt_flags[id_pt]){
      x_e.segment<3>(3 * id_pt).setZero();
    }else{
      vector3 tmp = etbs[id_pt];
      for (const int id_res : pid2rid_[id_pt]) {
        const int id_cam = edge_vec[id_res].first; 
        sub_61_l(factors_v4[id_res].data(), x_f.segment<6>(6 * id_cam),  tmp);
      }
      x_e.segment<3>(3 * id_pt) = etes[id_pt] * tmp;
    }
  }
  x_f = -x_f;
  x_e = -x_e;
}

void linear_solver_refine::solve_linear_problem(double radius) {
  radius_ = radius;
  timer_linearization.resume();
  linearization();
  timer_linearization.stop();

  timer_schur_complement.resume();
  schur_complement();
  timer_schur_complement.stop();

  timer_compute_delta.resume();
  compute_delta();
  timer_compute_delta.stop();
}

void linear_solver_refine::compute_model_cost_change() {
  // new_model_cost
  //  = 1/2 [f + J * step]^2
  //  = 1/2 [ f'f + 2f'J * step + step' * J' * J * step ]
  // model_cost_change
  //  = cost - new_model_cost
  //  = f'f/2  - 1/2 [ f'f + 2f'J * step + step' * J' * J * step]
  //  = -f'J * step - step' * J' * J * step / 2
  //  = -(J * step)'(f + J * step / 2)
  default_float _model_cost_change = 0;
  for (int id_res = 0; id_res < num_res_; ++id_res) {
    const auto &[id_cam, id_pt] = edge_vec[id_res]; 
    const_map<vector3> a(factors_v5[id_res].data());
    const default_float s = factors_v5[id_res].data()[3];
    const default_float h = factors_v5[id_res].data()[4];
    const vector3 b = x_f.segment<3>(6 * id_cam + 3) + x_e.segment<3>(3 * id_pt);
    vector<residual_size> model_residual = h*((s * (b - a.dot(b) * a)) - a.cross(x_f.segment<3>(6 * id_cam)));
    vector<residual_size> tmp = model_residual / 2 + residuals[id_res];
    _model_cost_change -= model_residual.dot(tmp);
  }
  model_cost_change_ = _model_cost_change;
  // std::cout << "model cost change: " << model_cost_change_ << std::endl;
}

void linear_solver_refine::compute_cost(double &cost) {
  cost = 0;
  for (int i = 0; i < num_res_; ++i) {
    cost += residuals[i].squaredNorm();
  }
  cost /= 2;
}

void linear_solver_refine::compute_relative_decrease() {
  compute_cost(current_cost_);
  // compute candidate param
  param_plus_delta();
  // compute relative_decrease
  candidate_cost_ = 0;
  for (int i = 0; i < num_res_; ++i) {
    RES_BLOCK res;
    const auto &rb = residual_blocks[i];
    rb.factor->EvaluateRes(rb.param_blocks_candidate.data(), res.data());
    candidate_cost_ += res.squaredNorm();
  }
  candidate_cost_ /= 2;
  relative_decrease_ = (current_cost_ - candidate_cost_) / model_cost_change_;
}

void linear_solver_refine::compute_step_norm_x_norm(double &step_norm, double &x_norm) {
  step_norm = x_norm = 0;
  for (int id_cam = 0; id_cam < num_cam_; ++id_cam) {
    if (const_cam_flags[id_cam]) continue;
    step_norm += param_blocks[id_cam * 2].step_square_norm();
    step_norm += param_blocks[id_cam * 2 + 1].step_square_norm();
    x_norm += param_blocks[id_cam * 2].x_square_norm();
    x_norm += param_blocks[id_cam * 2 + 1].x_square_norm();
  }
  step_norm = sqrt(step_norm);
  x_norm = sqrt(x_norm);
}

void linear_solver_refine::param_plus_delta() {
  for (int id_cam = 0; id_cam < num_cam_; ++id_cam) {
    // if (const_cam_flags[id_cam]) continue;
    default_float *delta_x = x_f.block<6, 1>(6 * id_cam, 0).data();
    param_blocks[id_cam * 2].plus_delta(delta_x);
    param_blocks[id_cam * 2 + 1].plus_delta(delta_x + 3);
  }
  for (int id_pt = 0; id_pt < num_point_; ++id_pt) {
    default_float *delta_x = x_e.block<3, 1>(3 * id_pt, 0).data();
    param_blocks[num_cam_ * 2 + id_pt].plus_delta(delta_x);
  }
}

void linear_solver_refine::param_update() {
  for (int id_cam = 0; id_cam < num_cam_; ++id_cam) {
    // if (const_cam_flags[id_cam]) continue;
    param_blocks[id_cam * 2].update();
    param_blocks[id_cam * 2 + 1].update();
  }
  for (int id_pt = 0; id_pt < num_point_; ++id_pt) {
    param_blocks[num_cam_ * 2 + id_pt].update();
  }
}

void linear_solver_refine::log() {
  for (auto &timer_ptr : timer_vec) timer_ptr->log();
}

}  // namespace mgba
