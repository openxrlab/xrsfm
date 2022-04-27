/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this
license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                          License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
// Copyright (C) 2009, Willow Garage Inc., all rights reserved.
// Copyright (C) 2013, OpenCV Foundation, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without
modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright
notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote
products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is"
and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are
disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any
direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

#pragma once

#include "opencv2/opencv.hpp"

namespace cv {

//! the algorithm for finding fundamental matrix
// enum { FM_7POINT = 1, //!< 7-point algorithm
//        FM_8POINT = 2, //!< 8-point algorithm
//        FM_LMEDS  = 4, //!< least-median algorithm. 7-point algorithm is used.
//        FM_RANSAC = 8  //!< RANSAC algorithm. It needs at least 15 points.
//        7-point algorithm is used.
//      };

/** @brief Calculates a fundamental matrix from the corresponding points in two
images.

@param points1 Array of N points from the first image. The point coordinates
should be floating-point (single or double precision).
@param points2 Array of the second image points of the same size and format as
points1 .
@param method Method for computing a fundamental matrix.
-   **CV_FM_7POINT** for a 7-point algorithm. \f$N = 7\f$
-   **CV_FM_8POINT** for an 8-point algorithm. \f$N \ge 8\f$
-   **CV_FM_RANSAC** for the RANSAC algorithm. \f$N \ge 8\f$
-   **CV_FM_LMEDS** for the LMedS algorithm. \f$N \ge 8\f$
@param ransacReprojThreshold Parameter used only for RANSAC. It is the maximum
distance from a point to an epipolar line in pixels, beyond which the point is
considered an outlier and is not used for computing the final fundamental
matrix. It can be set to something like 1-3, depending on the accuracy of the
point localization, image resolution, and the image noise.
@param confidence Parameter used for the RANSAC and LMedS methods only. It
specifies a desirable level of confidence (probability) that the estimated
matrix is correct.
@param mask

The epipolar geometry is described by the following equation:

\f[[p_2; 1]^T F [p_1; 1] = 0\f]

where \f$F\f$ is a fundamental matrix, \f$p_1\f$ and \f$p_2\f$ are corresponding
points in the first and the second images, respectively.

The function calculates the fundamental matrix using one of four methods listed
above and returns the found fundamental matrix. Normally just one matrix is
found. But in case of the 7-point algorithm, the function may return up to 3
solutions ( \f$9 \times 3\f$ matrix that stores all 3 matrices sequentially).

The calculated fundamental matrix may be passed further to
computeCorrespondEpilines that finds the epipolar lines corresponding to the
specified points. It can also be passed to stereoRectifyUncalibrated to compute
the rectification transformation. :
@code
    // Example. Estimation of fundamental matrix using the RANSAC algorithm
    int point_count = 100;
    vector<Point2f> points1(point_count);
    vector<Point2f> points2(point_count);

    // initialize the points here ...
    for( int i = 0; i < point_count; i++ )
    {
        points1[i] = ...;
        points2[i] = ...;
    }

    Mat fundamental_matrix =
     findFundamentalMat(points1, points2, FM_RANSAC, 3, 0.99);
@endcode
 */
Mat findFundamentalMat_MaxIter(InputArray points1, InputArray points2, int method = FM_RANSAC,
                               double ransacReprojThreshold = 3., double confidence = 0.99,
                               OutputArray mask = noArray(), int max_iter = 100);

}  // end namespace cv
