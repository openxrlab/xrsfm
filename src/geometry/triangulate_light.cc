// Copyright (c) 2019, SenseTime Group.
// All rights reserved.

#include "triangulate_light.h"

#include <Eigen/Geometry>
#include <unordered_set>

#include "ransac/combination_sampler.h"
#include "ransac/loransac.h"

namespace xrsfm {

inline Eigen::Matrix3d hat(const Eigen::Vector3d &vector) {
    Eigen::Matrix3d matrix;
    matrix << 0, -vector(2), vector(1), vector(2), 0, -vector(0), -vector(1),
        vector(0), 0;
    return matrix;
}

inline Eigen::Vector3d
MultiPointEstimate(const std::vector<Eigen::Vector3d> &points,
                   const std::vector<Eigen::Vector3d> &poses) {
    Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
    Eigen::Vector3d b = Eigen::Vector3d::Zero();

    for (int i = 0; i < points.size(); ++i) {
        Eigen::Matrix3d J = hat(points.at(i));
        Eigen::Vector3d r = (points.at(i)).cross(poses.at(i));
        A += J.transpose() * J;
        b += J.transpose() * r;
    }

    Eigen::LLT<Eigen::Matrix3d> llt = A.llt();
    return llt.solve(b);
}

std::vector<Eigen::Vector3d>
TriLightEstimator::Estimate(const std::vector<Eigen::Vector3d> &points,
                            const std::vector<Eigen::Vector3d> &poses) const {
    CHECK_GE(points.size(), 2);
    CHECK_EQ(points.size(), poses.size());
    const double max_cos_theta = std::cos(min_tri_angle_);

    const M_t xyz = MultiPointEstimate(points, poses);

    for (int i = 0; i < points.size(); ++i) {
        if (points.at(i).dot(xyz - poses.at(i)) < 0)
            return std::vector<M_t>();
    }

    for (size_t i = 0; i < points.size() - 1; ++i) {
        const Eigen::Vector3d ray_i = (xyz - poses.at(i)).normalized();
        for (size_t j = i + 1; j < points.size(); ++j) {
            const Eigen::Vector3d ray_j = (xyz - poses.at(j)).normalized();
            const double cos_theta = ray_i.dot(ray_j);
            if (cos_theta < max_cos_theta) {
                return std::vector<M_t>{xyz};
            }
        }
    }

    return std::vector<M_t>();
}

inline double caculate_ray_error(const Eigen::Vector3d &ray1,
                                 const Eigen::Vector3d &ray2) {
    double cos_theta = ray1.dot(ray2);
    cos_theta = std::acos(std::max(-1.0, std::min(1.0, cos_theta)));
    return cos_theta;
}

void TriLightEstimator::Residuals(const std::vector<Eigen::Vector3d> &points,
                                  const std::vector<Eigen::Vector3d> &poses,
                                  const Eigen::Vector3d &xyz,
                                  std::vector<double> *residuals) const {
    residuals->resize(points.size());

    for (size_t i = 0; i < points.size(); ++i) {
        const Eigen::Vector3d ray1 = points.at(i);
        const Eigen::Vector3d ray2 = (xyz - poses.at(i)).normalized();
        const double angular_error = caculate_ray_error(ray1, ray2);
        residuals->at(i) = angular_error * angular_error;
    }
}

bool EstimatePointLight(const colmap::RANSACOptions &ransac_options,
                        const double min_tri_angle,
                        const std::vector<Eigen::Vector3d> &point_data,
                        const std::vector<Eigen::Vector3d> &pose_data,
                        std::vector<char> &inlier_mask, Eigen::Vector3d &xyz) {
    CHECK_GE(point_data.size(), 2);
    CHECK_EQ(point_data.size(), pose_data.size());

    colmap::LORANSAC<TriLightEstimator, TriLightEstimator,
                     colmap::InlierSupportMeasurer, colmap::CombinationSampler>
        ransac(ransac_options);
    ransac.estimator.min_tri_angle_ = ransac.local_estimator.min_tri_angle_ =
        min_tri_angle;
    const auto report = ransac.Estimate(point_data, pose_data);

    if (!report.success) {
        return false;
    }

    inlier_mask = report.inlier_mask;
    xyz = report.model;

    return report.success;
}

double CalculateTriangulationAngleLight(const Eigen::Vector3d &view_point0,
                                        const Eigen::Vector3d &view_point1,
                                        const Eigen::Vector3d &point3d) {
    Eigen::Vector3d ray0 = point3d - view_point0;
    Eigen::Vector3d ray1 = point3d - view_point1;
    if (ray0.norm() == 0 || ray1.norm() == 0)
        return 0;
    ray0 = ray0.normalized();
    ray1 = ray1.normalized();
    const double cos_theta = ray0.dot(ray1);
    return std::acos(cos_theta);
}

std::vector<double> CalculateTriangulationAnglesLight(
    const Eigen::Vector3d &view_point0, const Eigen::Vector3d &view_point1,
    const std::vector<Eigen::Vector3d> &points3d) {
    const int num_points = points3d.size();
    std::vector<double> angles(num_points);
    for (int i = 0; i < num_points; ++i) {
        angles.at(i) = CalculateTriangulationAngleLight(
            view_point0, view_point1, points3d.at(i));
    }
    return angles;
}

} // namespace xrsfm
