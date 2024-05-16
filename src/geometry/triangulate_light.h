// Copyright (c) 2019, SenseTime Group.
// All rights reserved.

#ifndef SENSEMAP_ESTIMATORS_TRIANGULATION_LIGHT_H_
#define SENSEMAP_ESTIMATORS_TRIANGULATION_LIGHT_H_

#include <Eigen/Core>
#include <vector>

#include "ransac/ransac.h"

namespace xrsfm {

inline double DegToRad(const double deg) {
    return deg * 0.0174532925199432954743716805978692718781530857086181640625;
}

class TriLightEstimator {
  public:
    typedef Eigen::Vector3d X_t; // ray
    typedef Eigen::Vector3d Y_t; // camera position
    typedef Eigen::Vector3d M_t; // point position

    // The minimum number of samples needed to estimate a model.
    static constexpr int kMinNumSamples = 2;

    std::vector<M_t> Estimate(const std::vector<X_t> &point_data,
                              const std::vector<Y_t> &pose_data) const;
    void Residuals(const std::vector<X_t> &point_data,
                   const std::vector<Y_t> &pose_data, const M_t &xyz,
                   std::vector<double> *residuals) const;

    double min_tri_angle_ = 0.0;
};

bool EstimatePointLight(const colmap::RANSACOptions &ransac_options,
                        const double min_tri_angle,
                        const std::vector<Eigen::Vector3d> &point_data,
                        const std::vector<Eigen::Vector3d> &pose_data,
                        std::vector<char> &inlier_mask, Eigen::Vector3d &xyz);

double CalculateTriangulationAngleLight(const Eigen::Vector3d &view_point0,
                                        const Eigen::Vector3d &view_point1,
                                        const Eigen::Vector3d &point3d);

std::vector<double>
CalculateTriangulationAnglesLight(const Eigen::Vector3d &view_point0,
                                  const Eigen::Vector3d &view_point1,
                                  const std::vector<Eigen::Vector3d> &points3d);

} // namespace xrsfm

#endif // SENSEMAP_ESTIMATORS_TRIANGULATION_LIGHT_H_
