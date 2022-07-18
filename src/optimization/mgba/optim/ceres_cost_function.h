//
// Created by SENSETIME\yezhichao1 on 2020/4/15.
//
#pragma once
#ifndef WARG_PROJECTION_FACTOR_H
#define WARG_PROJECTION_FACTOR_H

#endif  // WARG_PROJECTION_FACTOR_H

#include <ceres/ceres.h>
#include <iostream>
#include <ceres/rotation.h>
#include "../matrix_math.h"

class ProjectionCostAuto {
 public:
  explicit ProjectionCostAuto(const Eigen::Vector2d &p2d,double _w) : mea_(p2d),w(_w){}

  static ceres::CostFunction *Create(const Eigen::Vector2d &p2d, double _w) {
    return (new ceres::AutoDiffCostFunction<ProjectionCostAuto, 2, 4, 3, 3>(new ProjectionCostAuto(p2d, _w)));
  }

  template <typename T>
  bool operator()(const T *const quat_ptr, const T *const tvec_ptr, const T *const point3d_ptr,
                  T *residuals_ptr) const { 
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> P(tvec_ptr); 
    Eigen::Map<Eigen::Matrix<T, 2, 1>> residuals(residuals_ptr);
    Eigen::Matrix<T, 3, 1> projects;

    ceres::UnitQuaternionRotatePoint(quat_ptr, point3d_ptr, projects.data());
    projects += P;
    
    residuals = w*(projects.hnormalized() - mea_);
    return true;
  }

 private:
  const Eigen::Vector2d mea_; 
  double w;
};
 
class ProjectionCostAuto1 {
 public:
  explicit ProjectionCostAuto1(const Eigen::Vector2d &p2d,double _w) : mea_(p2d),w(_w){}

  static ceres::CostFunction *Create(const Eigen::Vector2d &p2d, double _w) {
    return (new ceres::AutoDiffCostFunction<ProjectionCostAuto, 2, 4, 3, 3>(new ProjectionCostAuto(p2d, _w)));
  }

  template <typename T>
  bool operator()(const T *const quat_ptr, const T *const tvec_ptr, const T *const point3d_ptr,
                  T *residuals_ptr) const { 
    Eigen::Map<const Eigen::Quaternion<T>> Q(quat_ptr);
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> P(tvec_ptr); 
    Eigen::Map<const Eigen::Matrix<T, 3, 1>> pt_w(point3d_ptr);
    Eigen::Map<Eigen::Matrix<T, 2, 1>> residuals(residuals_ptr);
    Eigen::Matrix<T, 3, 1> projects;

    projects +=  projects = Q * (P + pt_w);
    residuals = w*(projects.hnormalized() - mea_);
    return true;
  }

 private:
  const Eigen::Vector2d mea_; 
  double w;
};
 

constexpr bool compute_dq = false;

class ProjectionDepthCost : public ceres::SizedCostFunction<2, 4, 3, 4, 3, 1> {
 public:
  ProjectionDepthCost(const Eigen::Vector2d &p2d1, const Eigen::Vector2d &p2d2) : mea1_(p2d1), mea2_(p2d2) {}

  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    Eigen::Map<const Eigen::Quaterniond> Q1(parameters[0]);
    Eigen::Map<const Eigen::Vector3d> P1(parameters[1]);
    Eigen::Map<const Eigen::Quaterniond> Q2(parameters[2]);
    Eigen::Map<const Eigen::Vector3d> P2(parameters[3]);
    const double depth = parameters[4][0];
    Eigen::Map<Eigen::Vector2d> residual(residuals);

    Eigen::Matrix3d R2 = Q2.toRotationMatrix();
    Eigen::Vector3d ray_vec = Q1.inverse() * mea1_.homogeneous();
    Eigen::Vector3d pt_w = depth * ray_vec - P1;
    Eigen::Vector3d pt_c = R2 * (pt_w + P2);
    double dep = pt_c.z();
    // std::cout << dep << std::endl;
    residual = pt_c.hnormalized() - mea2_;

    if (jacobians) {
      Eigen::Matrix<double, 2, 3> reduce;
      reduce << 1. / dep, 0.0, -pt_c(0) / (dep * dep), 0.0, 1. / dep, -pt_c(1) / (dep * dep);
      auto &reduceR = reduce * R2;
      auto &dr_dpt_w = reduceR;
      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> jacobian_Q1(jacobians[0]);
        jacobian_Q1.leftCols<3>() = reduceR * skewSymmetric(depth * ray_vec);
        jacobian_Q1.rightCols<1>().setZero();
      }
      if (jacobians[1]) {
        Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> jacobian_P1(jacobians[1]);
        jacobian_P1 = -dr_dpt_w;
      }
      if (jacobians[2]) {
        Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> jacobian_Q2(jacobians[2]);
        jacobian_Q2.leftCols<3>() = -dr_dpt_w * skewSymmetric(pt_w + P2);
        jacobian_Q2.rightCols<1>().setZero();
      }
      if (jacobians[3]) {
        Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> jacobian_P2(jacobians[3]);
        jacobian_P2 = dr_dpt_w;
      }
      if (jacobians[4]) {
        Eigen::Map<Eigen::Matrix<double, 2, 1>> jacobian_d(jacobians[4]);
        jacobian_d = dr_dpt_w * ray_vec;
      }
    }
    return true;
  }

  Eigen::Vector2d mea1_, mea2_;
};

class ProjectionCost : public ceres::SizedCostFunction<2, 4, 3, 3> {
 public:
  ProjectionCost(const Eigen::Vector2d &p2d) : mea(p2d) {}

  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    Eigen::Map<const Eigen::Quaterniond> Q(parameters[0]);
    Eigen::Map<const Eigen::Vector3d> P(parameters[1]);
    Eigen::Map<const Eigen::Vector3d> pt_w(parameters[2]);
    Eigen::Map<Eigen::Vector2d> residual(residuals);

    /// project point
    Eigen::Matrix3d R = Q.toRotationMatrix();
    Eigen::Vector3d pt_c = R * (pt_w + P);
    double dep = pt_c.z();
    residual = pt_c.hnormalized() - mea;

    if (jacobians) {
      Eigen::Matrix<double, 2, 3> reduce;
      reduce << 1. / dep, 0.0, -pt_c(0) / (dep * dep), 0.0, 1. / dep, -pt_c(1) / (dep * dep);
      auto reduceR = reduce * R;
      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> jacobian_Q(jacobians[0]);
        if (compute_dq) {
          Eigen::Matrix<double, 3, 4, Eigen::RowMajor> dp_dq;
          double w = Q.w();
          Eigen::Vector3d v(Q.x(), Q.y(), Q.z());
          Eigen::Vector3d a = pt_w;
          dp_dq.leftCols<1>() = v.cross(a);
          dp_dq.rightCols<3>() =
              v.dot(a) * Eigen::Matrix3d::Identity() + v * a.transpose() - 2 * a * v.transpose() - w * skewSymmetric(a);
          jacobian_Q = 2 * reduce * dp_dq;
        } else {
          jacobian_Q.leftCols<3>() = -reduceR * skewSymmetric(pt_w + P);
          jacobian_Q.rightCols<1>().setZero();
        }
      }
      if (jacobians[1]) {
        Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> jacobian_P(jacobians[1]);
        jacobian_P = reduceR;
      }
      if (jacobians[2]) {
        Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> jacobian_p3d(jacobians[2]);
        jacobian_p3d = reduceR;
      }
    }
    return true;
  }

  Eigen::Vector2d mea;
};

class UnitVectorCost : public ceres::SizedCostFunction<3, 4, 3, 3> {
 public:
  UnitVectorCost(const Eigen::Vector2d &p2d) : mea(p2d.homogeneous().normalized()) {}

  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    Eigen::Map<const Eigen::Quaterniond> Q(parameters[0]);
    Eigen::Map<const Eigen::Vector3d> P(parameters[1]);
    Eigen::Map<const Eigen::Vector3d> pt_w(parameters[2]);
    Eigen::Map<Eigen::Vector3d> residual(residuals);

    /// project point
    Eigen::Matrix3d R = Q.toRotationMatrix();
    Eigen::Vector3d a1 = pt_w + P;
    double scale = 1.0 / a1.norm();
    Eigen::Vector3d a = a1.normalized();
    Eigen::Vector3d b = mea;
    residual = R * a - b;

    if (jacobians) {
      Eigen::Matrix3d dr_da = scale * (Eigen::Matrix3d::Identity() - a * a.transpose());
      Eigen::Matrix<double, 3, 3> dr_daR = R * dr_da;

      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> jacobian_Q(jacobians[0]);
        jacobian_Q.setZero();
        jacobian_Q.leftCols<3>() = -R * skewSymmetric(a);  // It looks interesting
      }
      if (jacobians[1]) {
        Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jacobian_P(jacobians[1]);
        jacobian_P = dr_daR;
      }
      if (jacobians[2]) {
        Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jacobian_p3d(jacobians[2]);
        jacobian_p3d = dr_daR;
      }
    }
    return true;
  }

  Eigen::Vector3d mea;
};

class UnitVectorCostW : public ceres::SizedCostFunction<3, 4, 3, 3> {
 public:
  UnitVectorCostW(const Eigen::Vector2d &p2d,double _w) : mea(p2d.homogeneous().normalized()),w(_w) {}

  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    Eigen::Map<const Eigen::Quaterniond> Q(parameters[0]);
    Eigen::Map<const Eigen::Vector3d> P(parameters[1]);
    Eigen::Map<const Eigen::Vector3d> pt_w(parameters[2]);
    Eigen::Map<Eigen::Vector3d> residual(residuals);

    /// project point
    Eigen::Matrix3d R = Q.toRotationMatrix();
    Eigen::Vector3d a1 = pt_w + P;
    double scale = 1.0 / a1.norm();
    Eigen::Vector3d a = a1.normalized();
    Eigen::Vector3d b = mea;
    residual = w*(R * a - b);

    if (jacobians) {
      Eigen::Matrix3d dr_da = scale * (Eigen::Matrix3d::Identity() - a * a.transpose());
      Eigen::Matrix<double, 3, 3> dr_daR = w*(R * dr_da);

      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> jacobian_Q(jacobians[0]);
        jacobian_Q.setZero();
        jacobian_Q.leftCols<3>() = -w*R * skewSymmetric(a);  // It looks interesting
      }
      if (jacobians[1]) {
        Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jacobian_P(jacobians[1]);
        jacobian_P = dr_daR;
      }
      if (jacobians[2]) {
        Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jacobian_p3d(jacobians[2]);
        jacobian_p3d = dr_daR;
      }
    }
    return true;
  }

  Eigen::Vector3d mea;
  double w;
};


class UnitVectorCost1 : public ceres::SizedCostFunction<3, 4, 3, 3> {
 public:
  UnitVectorCost1(const Eigen::Vector2d &p2d) : mea(p2d.homogeneous().normalized()) {}

  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    Eigen::Map<const Eigen::Quaterniond> Q(parameters[0]);
    Eigen::Map<const Eigen::Vector3d> P(parameters[1]);
    Eigen::Map<const Eigen::Vector3d> pt_w(parameters[2]);
    Eigen::Map<Eigen::Vector3d> residual(residuals);

    /// project point
    Eigen::Matrix3d R = Q.toRotationMatrix();
    Eigen::Vector3d a1 = pt_w + P;
    double scale = 1.0 / a1.norm();
    Eigen::Vector3d a = scale * a1;
    Eigen::Vector3d b = R.transpose() * mea;
    residual = a - b;

    if (jacobians) {
      Eigen::Matrix3d dr_da = scale * (Eigen::Matrix3d::Identity() - a * a.transpose());

      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> jacobian_Q(jacobians[0]);
        jacobian_Q.setZero();
        jacobian_Q.leftCols<3>() = -skewSymmetric(b);  // It looks interesting
      }
      if (jacobians[1]) {
        Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jacobian_P(jacobians[1]);
        jacobian_P = dr_da;
      }
      if (jacobians[2]) {
        Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jacobian_p3d(jacobians[2]);
        jacobian_p3d = dr_da;
        ;
      }
    }
    return true;
  }

  Eigen::Vector3d mea;
};

class CrossCost : public ceres::SizedCostFunction<3, 4, 3, 3> {
 public:
  explicit CrossCost(const Eigen::Vector2d &p2d) : mea(p2d.homogeneous().normalized()) {}

  bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const override {
    Eigen::Map<const Eigen::Quaterniond> Q(parameters[0]);
    Eigen::Map<const Eigen::Vector3d> P(parameters[1]);
    Eigen::Map<const Eigen::Vector3d> pt_w(parameters[2]);
    Eigen::Map<Eigen::Vector3d> residual(residuals);

    /// project point
    Eigen::Matrix3d R = Q.toRotationMatrix();
    Eigen::Vector3d a1 = pt_w + P;
    double scale = 1.0 / a1.norm();
    Eigen::Vector3d a = a1.normalized();
    Eigen::Vector3d b = R.transpose() * mea;
    residual = b.cross(a);

    if (jacobians) {
      Eigen::Matrix3d dr_dsa = skewSymmetric(b);
      Eigen::Matrix3d dsa_da = scale * (Eigen::Matrix3d::Identity() - a * a.transpose());
      Eigen::Matrix<double, 3, 3> dr_da = dr_dsa * dsa_da;

      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> jacobian_Q(jacobians[0]);
        jacobian_Q.leftCols<3>() = -skewSymmetric(a) * skewSymmetric(b);
        jacobian_Q.rightCols<1>().setZero();
      }
      if (jacobians[1]) {
        Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jacobian_P(jacobians[1]);
        jacobian_P = dr_da;
      }
      if (jacobians[2]) {
        Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jacobian_p3d(jacobians[2]);
        jacobian_p3d = dr_da;
        ;
      }
    }
    return true;
  }

  Eigen::Vector3d mea;
};

class CrossTangentNew : public ceres::SizedCostFunction<2, 4, 3, 3> {
 public:
  explicit CrossTangentNew(const Eigen::Vector2d &p2d) : mea(p2d.homogeneous().normalized()) {
    local_tangent.leftCols<2>() = s2_tangential_basis(mea);
    local_tangent.rightCols<1>() = mea;
  }

  bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const override {
    Eigen::Map<const Eigen::Quaterniond> Q(parameters[0]);
    Eigen::Map<const Eigen::Vector3d> P(parameters[1]);
    Eigen::Map<const Eigen::Vector3d> pt_w(parameters[2]);
    Eigen::Map<Eigen::Vector2d> residual(residuals);

    /// project point
    Eigen::Matrix3d R = Q.toRotationMatrix();
    Eigen::Vector3d a1 = pt_w + P;
    Eigen::Vector3d a = R * a1;
    Eigen::Vector3d b = local_tangent.transpose() * a;
    residual = b.hnormalized();

    if (jacobians) {
      Eigen::Matrix<double, 2, 3> dr_db;
      dr_db << 1. / b.z(), 0.0, -b.x() / (b.z() * b.z()), 0.0, 1. / b.z(), -b.y() / (b.z() * b.z());

      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> jacobian_Q(jacobians[0]);
        jacobian_Q.setZero();
        jacobian_Q.leftCols<3>() = -dr_db * local_tangent.transpose() * R * skewSymmetric(a1);
      }
      if (jacobians[1]) {
        Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> jacobian_P(jacobians[1]);
        jacobian_P.setZero();
        jacobian_P = dr_db * local_tangent.transpose() * R;
      }
      if (jacobians[2]) {
        Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> jacobian_p3d(jacobians[2]);
        jacobian_p3d.setZero();
        jacobian_p3d = dr_db * local_tangent.transpose() * R;
      }
    }
    return true;
  }

  Eigen::Vector3d mea;
  Eigen::Matrix3d local_tangent;
};

class CrossTangentOld : public ceres::SizedCostFunction<2, 4, 3, 3> {
 public:
  explicit CrossTangentOld(const Eigen::Vector2d &p2d) : mea(p2d.homogeneous().normalized()) {
    local_tangent.leftCols<2>() = s2_tangential_basis(mea);
    local_tangent.rightCols<1>() = mea;
  }

  bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const override {
    Eigen::Map<const Eigen::Quaterniond> Q(parameters[0]);
    Eigen::Map<const Eigen::Vector3d> P(parameters[1]);
    Eigen::Map<const Eigen::Vector3d> pt_w(parameters[2]);
    Eigen::Map<Eigen::Vector2d> residual(residuals);

    /// project point
    Eigen::Matrix3d R = Q.toRotationMatrix();
    Eigen::Vector3d a = R * pt_w + P;
    Eigen::Vector3d b = local_tangent.transpose() * a;
    residual = b.hnormalized();

    if (jacobians) {
      Eigen::Matrix<double, 2, 3> dr_db;
      dr_db << 1. / b.z(), 0.0, -b.x() / (b.z() * b.z()), 0.0, 1. / b.z(), -b.y() / (b.z() * b.z());

      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 2, 4, Eigen::RowMajor>> jacobian_Q(jacobians[0]);
        jacobian_Q.setZero();
        jacobian_Q.leftCols<3>() = -dr_db * local_tangent.transpose() * R * skewSymmetric(pt_w);
      }
      if (jacobians[1]) {
        Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> jacobian_P(jacobians[1]);
        jacobian_P.setZero();
        jacobian_P = dr_db * local_tangent.transpose();
      }
      if (jacobians[2]) {
        Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> jacobian_p3d(jacobians[2]);
        jacobian_p3d.setZero();
        jacobian_p3d = dr_db * local_tangent.transpose() * R;
      }
    }
    return true;
  }

  Eigen::Vector3d mea;
  Eigen::Matrix3d local_tangent;
};

class CrossCostAppr : public ceres::SizedCostFunction<3, 4, 3, 3> {
 public:
  CrossCostAppr(const Eigen::Vector2d &p2d) : mea(p2d.homogeneous().normalized()) {}

  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    Eigen::Map<const Eigen::Quaterniond> Q(parameters[0]);
    Eigen::Map<const Eigen::Vector3d> P(parameters[1]);
    Eigen::Map<const Eigen::Vector3d> pt_w(parameters[2]);
    Eigen::Map<Eigen::Vector3d> residual(residuals);

    /// project point
    Eigen::Vector3d a1 = pt_w + P;
    Eigen::Vector3d b1 = mea;
    double scale = 1.0 / a1.norm();
    Eigen::Vector3d a = scale * a1;
    Eigen::Vector3d b = Q.inverse() * b1;
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

    residual = b.cross(a);

    if (jacobians) {
      Eigen::Matrix3d dr_da = scale * skewSymmetric(a);

      if (jacobians[0]) {
        Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> jacobian_Q(jacobians[0]);
        jacobian_Q.setZero();
        jacobian_Q.leftCols<3>() = I - a * a.transpose();
      }
      if (jacobians[1]) {
        Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jacobian_P(jacobians[1]);
        jacobian_P = dr_da;
      }
      if (jacobians[2]) {
        Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> jacobian_p3d(jacobians[2]);
        jacobian_p3d = dr_da;
      }
    }
    return true;
  }

  Eigen::Vector3d mea;
};

class EigenQuatParam : public ceres::LocalParameterization {
 public:
  bool Plus(const double *x, const double *delta, double *x_plus_delta) const override {
    Eigen::Map<const Eigen::Quaterniond> _q(x);
    Eigen::Map<const Eigen::Vector3d> theta(delta);
    Eigen::Map<Eigen::Quaterniond> q(x_plus_delta);

    // double norm = theta.norm();
    // double t = asin(norm);
    // double cos_half_t = cos(t / 2);
    // Eigen::Vector3d vec = theta / (2 * cos_half_t);
    // Eigen::Quaterniond dq(cos_half_t, vec(0), vec(1), vec(2));
    // dq = dq.normalized();

    Eigen::Quaterniond dq = expmap(theta);  // delta_q(theta);
    q = (_q * dq).normalized();

    return true;
  }

  bool ComputeJacobian(const double *x, double *jacobian) const override {
    Eigen::Map<Eigen::Matrix<double, 4, 3, Eigen::RowMajor>> j(jacobian);
    j.setIdentity();
    return true;
  }

  int GlobalSize() const override { return 4; };

  int LocalSize() const override { return 3; };
};
