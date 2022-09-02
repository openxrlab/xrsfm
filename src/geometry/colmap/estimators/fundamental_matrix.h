// Copyright (c) 2018, ETH Zurich and UNC Chapel Hill.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//
//     * Neither the name of ETH Zurich and UNC Chapel Hill nor the names of
//       its contributors may be used to endorse or promote products derived
//       from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: Johannes L. Schoenberger (jsch-at-demuc-dot-de)

#ifndef COLMAP_SRC_ESTIMATORS_FUNDAMENTAL_MATRIX_H_
#define COLMAP_SRC_ESTIMATORS_FUNDAMENTAL_MATRIX_H_

#include <vector>

#include <Eigen/Core>

//#include "estimators/homography_matrix.h"
//#include "util/alignment.h"
#include "util/types.h"
#include "optim/ransac.h"

namespace colmap {

struct Options {
    // Minimum number of inliers for non-degenerate two-view geometry.
    size_t min_num_inliers = 15;

    // In case both cameras are calibrated, the calibration is verified by
    // estimating an essential and fundamental matrix and comparing their
    // fractions of number of inliers. If the essential matrix produces
    // a similar number of inliers (`min_E_F_inlier_ratio * F_num_inliers`),
    // the calibration is assumed to be correct.
    double min_E_F_inlier_ratio = 0.95;

    // In case an epipolar geometry can be verified, it is checked whether
    // the geometry describes a planar scene or panoramic view (pure rotation)
    // described by a homography. This is a degenerate case, since epipolar
    // geometry is only defined for a moving camera. If the inlier ratio of
    // a homography comes close to the inlier ratio of the epipolar geometry,
    // a planar or panoramic configuration is assumed.
    double max_H_inlier_ratio = 0.8;

    // In case of valid two-view geometry, it is checked whether the geometry
    // describes a pure translation in the border region of the image. If more
    // than a certain ratio of inlier points conform with a pure image
    // translation, a watermark is assumed.
    double watermark_min_inlier_ratio = 0.7;

    // Watermark matches have to be in the border region of the image. The
    // border region is defined as the area around the image borders and
    // is defined as a fraction of the image diagonal.
    double watermark_border_size = 0.1;

    // Whether to enable watermark detection. A watermark causes a pure
    // translation in the image space with inliers in the border region.
    bool detect_watermark = true;

    // Whether to ignore watermark models in multiple model estimation.
    bool multiple_ignore_watermark = true;

    // Options used to robustly estimate the geometry.
    RANSACOptions ransac_options;

    void Check() const {
        CHECK_GE(min_num_inliers, 0);
        CHECK_GE(min_E_F_inlier_ratio, 0);
        CHECK_LE(min_E_F_inlier_ratio, 1);
        CHECK_GE(max_H_inlier_ratio, 0);
        CHECK_LE(max_H_inlier_ratio, 1);
        CHECK_GE(watermark_min_inlier_ratio, 0);
        CHECK_LE(watermark_min_inlier_ratio, 1);
        CHECK_GE(watermark_border_size, 0);
        CHECK_LE(watermark_border_size, 1);
        ransac_options.Check();
    }
};

// Fundamental matrix estimator from corresponding point pairs.
//
// This algorithm solves the 7-Point problem and is based on the following
// paper:
//
//    Zhengyou Zhang and T. Kanade, Determining the Epipolar Geometry and its
//    Uncertainty: A Review, International Journal of Computer Vision, 1998.
//    http://citeseerx.ist.psu.edu/viewdoc/summary?doi=10.1.1.33.4540
class FundamentalMatrixSevenPointEstimator {
  public:
    typedef Eigen::Vector2d X_t;
    typedef Eigen::Vector2d Y_t;
    typedef Eigen::Matrix3d M_t;

    // The minimum number of samples needed to estimate a model.
    static const int kMinNumSamples = 7;

    // Estimate either 1 or 3 possible fundamental matrix solutions from a set
    // of corresponding points.
    //
    // The number of corresponding points must be exactly 7.
    //
    // @param points1  First set of corresponding points.
    // @param points2  Second set of corresponding points
    //
    // @return         Up to 4 solutions as a vector of 3x3 fundamental
    // matrices.
    static std::vector<M_t> Estimate(const std::vector<X_t> &points1,
                                     const std::vector<Y_t> &points2);

    // Calculate the residuals of a set of corresponding points and a given
    // fundamental matrix.
    //
    // Residuals are defined as the squared Sampson error.
    //
    // @param points1    First set of corresponding points as Nx2 matrix.
    // @param points2    Second set of corresponding points as Nx2 matrix.
    // @param F          3x3 fundamental matrix.
    // @param residuals  Output vector of residuals.
    static void Residuals(const std::vector<X_t> &points1,
                          const std::vector<Y_t> &points2, const M_t &F,
                          std::vector<double> *residuals);
};

// Fundamental matrix estimator from corresponding point pairs.
//
// This algorithm solves the 8-Point problem based on the following paper:
//
//    Hartley and Zisserman, Multiple View Geometry, algorithm 11.1, page 282.
class FundamentalMatrixEightPointEstimator {
  public:
    typedef Eigen::Vector2d X_t;
    typedef Eigen::Vector2d Y_t;
    typedef Eigen::Matrix3d M_t;

    // The minimum number of samples needed to estimate a model.
    static const int kMinNumSamples = 8;

    // Estimate fundamental matrix solutions from a set of corresponding points.
    //
    // The number of corresponding points must be at least 8.
    //
    // @param points1  First set of corresponding points.
    // @param points2  Second set of corresponding points
    //
    // @return         Single solution as a vector of 3x3 fundamental matrices.
    static std::vector<M_t> Estimate(const std::vector<X_t> &points1,
                                     const std::vector<Y_t> &points2);

    // Calculate the residuals of a set of corresponding points and a given
    // fundamental matrix.
    //
    // Residuals are defined as the squared Sampson error.
    //
    // @param points1    First set of corresponding points as Nx2 matrix.
    // @param points2    Second set of corresponding points as Nx2 matrix.
    // @param F          3x3 fundamental matrix.
    // @param residuals  Output vector of residuals.
    static void Residuals(const std::vector<X_t> &points1,
                          const std::vector<Y_t> &points2, const M_t &F,
                          std::vector<double> *residuals);
};

// Calculate the residuals of a set of corresponding points and a given
// fundamental or essential matrix.
//
// Residuals are defined as the squared Sampson error.
//
// @param points1     First set of corresponding points as Nx2 matrix.
// @param points2     Second set of corresponding points as Nx2 matrix.
// @param E           3x3 fundamental or essential matrix.
// @param residuals   Output vector of residuals.
void ComputeSquaredSampsonError(const std::vector<Eigen::Vector2d> &points1,
                                const std::vector<Eigen::Vector2d> &points2,
                                const Eigen::Matrix3d &E,
                                std::vector<double> *residuals);

// Center and normalize image points.
//
// The points are transformed in a two-step procedure that is expressed
// as a transformation matrix. The matrix of the resulting points is usually
// better conditioned than the matrix of the original points.
//
// Center the image points, such that the new coordinate system has its
// origin at the centroid of the image points.
//
// Normalize the image points, such that the mean distance from the points
// to the coordinate system is sqrt(2).
//
// @param points          Image coordinates.
// @param normed_points   Transformed image coordinates.
// @param matrix          3x3 transformation matrix.
void CenterAndNormalizeImagePoints(const std::vector<Eigen::Vector2d> &points,
                                   std::vector<Eigen::Vector2d> *normed_points,
                                   Eigen::Matrix3d *matrix);

} // namespace colmap

#endif // COLMAP_SRC_ESTIMATORS_FUNDAMENTAL_MATRIX_H_
