
#ifndef XRSFM_SRC_GEOMETRY_EPIPOLAR_GEOMETRY_HPP
#define XRSFM_SRC_GEOMETRY_EPIPOLAR_GEOMETRY_HPP

#include "estimators/fundamental_matrix.h"
#include "optim/loransac.h"

namespace xrsfm {
inline void SolveFundamnetalCOLMAP(const std::vector<Eigen::Vector2d> &points1,
                                   const std::vector<Eigen::Vector2d> &points2,
                                   FramePair &frame_pair) {
    colmap::Options option;
    option.ransac_options.max_error = 4.0;
    option.ransac_options.max_num_trials = 10000;
    option.ransac_options.min_num_trials = 100;
    option.ransac_options.confidence = 0.999;
    option.ransac_options.min_inlier_ratio = 0.25;
    colmap::LORANSAC<colmap::FundamentalMatrixSevenPointEstimator,
                     colmap::FundamentalMatrixEightPointEstimator>
        F_ransac(option.ransac_options);
    const auto F_report = F_ransac.Estimate(points1, points2);
    Eigen::Matrix3d F = F_report.model;
    frame_pair.inlier_num = F_report.support.num_inliers;
    frame_pair.inlier_mask = F_report.inlier_mask;
    frame_pair.F = F;
}
} // namespace xrsfm
#endif