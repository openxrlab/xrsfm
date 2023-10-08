//
// Created by SENSETIME\yezhichao1 on 2020/9/14.
//

#ifndef XRSFM_SRC_OPTIMIZATION_COST_FACTOR_CERES_H
#define XRSFM_SRC_OPTIMIZATION_COST_FACTOR_CERES_H

#include <ceres/ceres.h>

#include "lie_algebra.h"
#include "utility/global.h"

namespace xrsfm {

template <typename CameraModel> class ReProjectionCost {
  public:
    ReProjectionCost(Eigen::Vector2d uv) : uv_(uv) {}

    template <typename T>
    bool operator()(const T *const _qcw, const T *const _tcw,
                    const T *const _pw, const T *const _camera_param,
                    T *_residuals) const {
        const_map<Eigen::Quaternion<T>> qcw(_qcw);
        const_map<vector<3, T>> tcw(_tcw);
        const_map<vector<3, T>> pw(_pw);
        map<vector<2, T>> residuals(_residuals);

        vector<3, T> pc = qcw * pw + tcw;
        if (pc.z() < 0.1) {
            residuals = vector<2, T>(12.0, 12.0);
        } else {
            vector<2, T> xy = pc.hnormalized();
            vector<2, T> uv_estimated;
            CameraModel::WorldToImage(_camera_param, xy.data(),
                                      uv_estimated.data());
            residuals = uv_estimated - uv_;
        }
        return true;
    }

    static ceres::CostFunction *Create(Eigen::Vector2d uv) {
        return (new ceres::AutoDiffCostFunction<ReProjectionCost, 2, 4, 3, 3,
                                                CameraModel::kNumParams>(
            new ReProjectionCost(uv)));
    }

  private:
    const Eigen::Vector2d uv_;
};

inline static ceres::CostFunction *ReProjectionCostCreate(const int model_id,
                                                          Eigen::Vector2d uv) {
    if (model_id == 2)
        return ReProjectionCost<SimpleRadialCameraModel>::Create(uv);
    if (model_id == 4)
        return ReProjectionCost<OpenCVCameraModel>::Create(uv);

    return ReProjectionCost<OpenCVCameraModel>::Create(uv);
}

class ProjectionCost : public ceres::SizedCostFunction<2, 4, 3, 3> {
  public:
    ProjectionCost(const vector2 _obs, const double _sigma = 5.99 / 700,
                   const double _weight = 1.0)
        : obs(_obs), sigma(_sigma), weight(_weight) {}

    virtual bool Evaluate(double const *const *parameters, double *residuals,
                          double **jacobians) const {
        const_map<quaternion> Q(parameters[0]);
        const_map<vector3> P(parameters[1]);
        const_map<vector3> P_w(parameters[2]);
        map<vector2> residual(residuals);

        const matrix3 R = Q.toRotationMatrix();
        const vector3 p_c = R * P_w + P;
        const double z = p_c.z();

        residual = p_c.head<2>() / z - obs;
        const double r2 = residual.squaredNorm();
        const double huber_factor =
            r2 > sigma * sigma ? sqrt(sigma / sqrt(r2)) : 1;
        residual = weight * huber_factor * residual;

        if (jacobians) {
            matrix<2, 3> dr_dpc;
            dr_dpc << 1.0 / z, 0.0, -p_c.x() / (z * z), 0.0, 1.0 / z,
                -p_c.y() / (z * z);
            dr_dpc = weight * huber_factor * dr_dpc;

            if (jacobians[0]) {
                map<matrix<2, 4, true>> j_q(jacobians[0]);
                j_q.setZero();
                j_q.leftCols<3>() = -dr_dpc * R * skewSymmetric(P_w);
            }
            if (jacobians[1]) {
                map<matrix<2, 3, true>> j_p(jacobians[1]);
                j_p = dr_dpc;
            }
            if (jacobians[2]) {
                map<matrix<2, 3, true>> j_pw(jacobians[2]);
                j_pw = dr_dpc * R;
            }
        }
        return true;
    }

    double sigma;
    double weight;
    vector2 obs;
};

class PoseGraphCost : public ceres::SizedCostFunction<8, 4, 3, 4, 3, 1, 1> {
  public:
    PoseGraphCost(quaternion _q_mea, vector3 _p_mea, double _weight_o = 0)
        : q_mea(_q_mea), p_mea(_p_mea), weight_o(_weight_o){};

    virtual bool Evaluate(double const *const *parameters, double *residuals,
                          double **jacobians) const {
        quaternion q1(parameters[0]);
        vector3 p1(parameters[1]);
        quaternion q2(parameters[2]);
        vector3 p2(parameters[3]);
        double s1 = parameters[4][0];
        double s2 = parameters[5][0];
        map<Eigen::Vector<double, 8>> residual(residuals);

        quaternion q1_inverse = q1.inverse();
        quaternion q12_estimated = q1_inverse * q2;
        vector3 p12_estimated = (q1_inverse * (p2 - p1));

        double weight_q = 1.0;
        double weight_p = 1.0;
        residual.head<3>() = weight_q * logmap(q_mea * q12_estimated.inverse());
        residual.tail<3>() = weight_p * (p12_estimated - s1 * p_mea);

        // 1.0 00 seq be better
        // 0.5 02 seq be better
        double weight_s = 1.0;
        residual(3) = weight_s * (s1 / s2 - 1);

        // double weight_o = 0.1;  // 0.1 bad in seq_4761_478
        if (s1 < 1) {
            residual(4) = weight_o * (s1 - 1);
        } else {
            residual(4) = weight_o * (1.0 / s1 - 1);
        }

        if (jacobians) {
            matrix3 R1 = q1.toRotationMatrix();
            matrix3 JRI = jri(residual.head<3>());
            if (jacobians[0]) {
                map<matrix<8, 4, true>> j_q1(jacobians[0]);
                j_q1.setZero();
                j_q1.topLeftCorner<3, 3>() = weight_q * JRI;
                j_q1.bottomLeftCorner<3, 3>() =
                    weight_p * skewSymmetric(p12_estimated);
            }
            if (jacobians[1]) {
                map<matrix<8, 3, true>> j_p1(jacobians[1]);
                j_p1.setZero();
                j_p1.bottomLeftCorner<3, 3>() = -weight_p * R1.transpose();
            }
            if (jacobians[2]) {
                map<matrix<8, 4, true>> j_q2(jacobians[2]);
                j_q2.setZero();
                j_q2.topLeftCorner<3, 3>() =
                    -weight_q * JRI * q12_estimated.toRotationMatrix();
            }
            if (jacobians[3]) {
                map<matrix<8, 3, true>> j_p2(jacobians[3]);
                j_p2.setZero();
                j_p2.bottomLeftCorner<3, 3>() = weight_p * R1.transpose();
            }
            if (jacobians[4]) {
                map<matrix<8, 1>> j_s1(jacobians[4]);
                j_s1.setZero();
                j_s1.tail<3>() = -weight_p * p_mea;
                j_s1(3) = weight_s * (1.0 / s2);
                j_s1(4) = s1 < 1 ? weight_o : -weight_o / (s1 * s1);
            }
            if (jacobians[5]) {
                map<matrix<8, 1>> j_s2(jacobians[5]);
                j_s2.setZero();
                j_s2(3) = weight_s * (-s1 / (s2 * s2));
            }
        }
        return true;
    }

    quaternion q_mea;
    vector3 p_mea;
    double weight_o;
};

class ScaleCost : public ceres::SizedCostFunction<1, 1, 1> {
  public:
    ScaleCost(double _s12) : s12(_s12){};

    virtual bool Evaluate(double const *const *parameters, double *residuals,
                          double **jacobians) const {
        double s1 = parameters[0][0];
        double s2 = parameters[1][0];
        double weight = 10;
        residuals[0] = weight * (s1 / (s12 * s2) - 1);
        if (jacobians) {
            if (jacobians[0]) {
                jacobians[0][0] = weight * 1.0 / (s12 * s2);
            }
            if (jacobians[1]) {
                jacobians[1][0] = -weight * s1 / (s12 * s2 * s2);
            }
        }
        return true;
    }
    double s12;
};

class TagCost : public ceres::SizedCostFunction<3, 4, 3, 1, 3> {
  public:
    TagCost(vector3 _p3d_ori, double _w) : p3d_ori(_p3d_ori), w(_w){};

    virtual bool Evaluate(double const *const *parameters, double *residuals,
                          double **jacobians) const {
        const_map<quaternion> q(parameters[0]);
        const_map<vector3> t(parameters[1]);
        double s = parameters[2][0];
        const_map<vector3> p(parameters[3]);
        map<vector3> residual(residuals);

        matrix3 R = q.toRotationMatrix();
        residual = w * (p - (R * (s * p3d_ori) + t));
        if (jacobians) {
            if (jacobians[0]) {
                map<matrix<3, 4, true>> j_q(jacobians[0]);
                j_q.setZero();
                j_q.leftCols<3>() = w * (R * skewSymmetric(s * p3d_ori));
            }
            if (jacobians[1]) {
                map<matrix<3, 3, true>> j_t(jacobians[1]);
                j_t = w * -matrix3::Identity();
            }
            if (jacobians[2]) {
                map<matrix<3, 1, true>> j_s(jacobians[2]);
                j_s = -w * R * p3d_ori;
            }
            if (jacobians[3]) {
                map<matrix<3, 3, true>> j_p(jacobians[3]);
                j_p = w * matrix3::Identity();
            }
        }
        return true;
    }
    double w;
    vector3 p3d_ori;
};

 
class QuatParam : public ceres::LocalParameterization {
  public:
    bool Plus(const double *x, const double *delta,
              double *x_plus_delta) const override {
        map<quaternion> q(x_plus_delta);
        const_map<quaternion> _q(x);
        const_map<vector3> theta(delta);
        quaternion dq = expmap(theta);
        q = (_q * dq).normalized();
        return true;
    }

    bool ComputeJacobian(const double *x, double *jacobian) const override {
        map<matrix<4, 3, true>> j(jacobian);
        j.setIdentity();
        return true;
    }

    int GlobalSize() const override { return 4; }
    int LocalSize() const override { return 3; }
};
} // namespace xrsfm

#endif // XRSFM_SRC_OPTIMIZATION_COST_FACTOR_CERES_H
