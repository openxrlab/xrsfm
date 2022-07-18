//
// Created by SENSETIME\yezhichao1 on 2020/4/19.
//
#pragma once
#ifndef WARG_BA_SLOVER_H
#define WARG_BA_SLOVER_H

#include <iostream>
#include <memory>

#include "local_parameterization.h"
#include "../matrix_math.h"
#include "../type_mg.h"

namespace mgba {

// template <typename D>
// class ProjectionFactor {
//  public:
//   ProjectionFactor(const vector2 &mea) : mea3d((mea.homogeneous()).normalized()) {}

//   inline bool Evaluate_Jac(D const *const *const parameters, D *const residuals, D *const *const jacobians) const {
//     const_map<quaternion> Q(parameters[0]);
//     const_map<vector3> P(parameters[1]);
//     const_map<vector3> pt_w(parameters[2]);
//     map<vector3> res(residuals);

//     const vector3 a = (pt_w + P).normalized();
//     const D scale = 1.0f / (pt_w + P).norm();
//     res.noalias() = a - Q.conjugate() * mea3d;

//     map<matrix<3, 6>> j_cam(jacobians[0]);
//     map<matrix<3, 3>> j_point(jacobians[1]);
//     j_cam.leftCols<3>() = -skewSymmetric(a);
//     j_cam.rightCols<3>() = j_point = scale * (matrix3::Identity() - a * a.transpose());
//     return true;
//   }

//   inline bool Evaluate_Fac(D const *const *const parameters, D *const residuals, D *const *const factors) const {
//     const_map<quaternion> Q(parameters[0]);
//     const_map<vector3> P(parameters[1]);
//     const_map<vector3> pt_w(parameters[2]);
//     map<vector3> res(residuals);

//     const vector3 a = (pt_w + P).normalized();
//     const D scale = 1.0f / (pt_w + P).norm();
//     res.noalias() = a - Q.conjugate() * mea3d;

//     map<vector3> factor_v3(factors[0]);
//     map<vector4> factor_v4(factors[1]);
//     factor_v3.noalias() = scale * a;
//     factor_v4 << a, scale;

//     return true;
//   }

//   inline bool EvaluateRes(D const *const *parameters, D *const residuals) const {
//     const_map<quaternion> Q(parameters[0]);
//     const_map<vector3> P(parameters[1]);
//     const_map<vector3> pt_w(parameters[2]);
//     map<vector3> res(residuals);

//     vector3 a = (pt_w + P).normalized();
//     vector3 b = Q.conjugate() * mea3d;
//     res.noalias() = a - b;

//     return true;
//   }
//   vector3 mea3d;
// };


template <typename D>
class ProjectionFactor{
 public:
  ProjectionFactor(const vector2 &mea,const double _w = 1.0) : mea3d((mea.homogeneous()).normalized()),w(_w) {}

  inline bool Evaluate_Fac(D const *const *const parameters, D *const residuals, D *const *const factors) const {
    const_map<quaternion> Q(parameters[0]);
    const_map<vector3> P(parameters[1]);
    const_map<vector3> pt_w(parameters[2]);
    map<vector3> res(residuals);

    const vector3 a = (pt_w + P).normalized();
    const D scale = 1.0f / (pt_w + P).norm();
    vector3 b = Q.conjugate() * mea3d;
    res.noalias() = w*(a - b);

    map<vector4> factor_v4(factors[0]);
    map<vector5> factor_v5(factors[1]);
    factor_v4 << scale * a,w*w;
    factor_v5 << a, scale,w;
    return true;
  }

  inline bool EvaluateRes(D const *const *parameters, D *const residuals) const {
    const_map<quaternion> Q(parameters[0]);
    const_map<vector3> P(parameters[1]);
    const_map<vector3> pt_w(parameters[2]);
    map<vector3> res(residuals);

    vector3 a = (pt_w + P).normalized();
    vector3 b = Q.conjugate() * mea3d;
    res.noalias() = w*(a - b);
    return true;
  }
  
  vector3 mea3d;
  double w;
};



template <typename D>
struct ParamBlock {
  ParamBlock(D *_ptr, int _size, LocalParameterization *_p) {
    param_ptr = _ptr;
    size = _size;
    parameterization = _p;
    param_new.resize(_size);
    for (int i = 0; i < size; ++i) param_new[i] = param_ptr[i];
  }

  ParamBlock(D *_ptr, int _size) {
    param_ptr = _ptr;
    size = _size;
    parameterization = nullptr;
    param_new.resize(_size);
    for (int i = 0; i < size; ++i) param_new[i] = param_ptr[i];
  }

  void set_constant() { constant = true; }

  void plus_delta(D *param_ptr_delta) {
    D *param_ptr_new = param_new.data();
    if (parameterization) {
      parameterization->Plus(param_ptr, param_ptr_delta, param_ptr_new);
    } else {
      for (int i = 0; i < size; ++i) param_ptr_new[i] = param_ptr[i] + param_ptr_delta[i];
    }
  }

  void update() {
    for (int i = 0; i < size; ++i) param_ptr[i] = param_new[i];
  }

  double step_square_norm() {
    double sn = 0;
    for (int i = 0; i < size; ++i) sn += (param_new[i] - param_ptr[i]) * (param_new[i] - param_ptr[i]);
    return sn;
  }

  double x_square_norm() {
    double sn = 0;
    for (int i = 0; i < size; ++i) sn += param_ptr[i] * param_ptr[i];
    return sn;
  }

  int size;
  D *param_ptr;
  std::vector<D> param_new;
  LocalParameterization *parameterization;
  bool constant;
};

template <typename D>
struct ResidualBlock {
  std::array<D *,3> param_blocks;
  std::array<D *,3> param_blocks_candidate;
  std::unique_ptr<ProjectionFactor<D>> factor;

  void add_paramblock(ParamBlock<D> &pb) {
    param_blocks.emplace_back(pb.param_ptr);
    param_blocks_candidate.emplace_back(pb.param_new.data());
  }
};

}  // namespace mgba

#endif  // WARG_BA_SLOVER_H