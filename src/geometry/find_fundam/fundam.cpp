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
// Copyright (C) 2000, Intel Corporation, all rights reserved.
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

#include "precomp.hpp"
// #include "rho.h"
#include <iostream>

namespace cv {
/* Estimation of Fundamental Matrix from point correspondences.
   The original code has been written by Valery Mosyagin */

/* The algorithms (except for RANSAC) and the notation have been taken from
   Zhengyou Zhang's research report
   "Determining the Epipolar Geometry and its Uncertainty: A Review"
   that can be found at
   http://www-sop.inria.fr/robotvis/personnel/zzhang/zzhang-eng.html */

/************************************** 7-point algorithm
 * *******************************/
/**
 * Compute the fundamental matrix using the 7-point algorithm.
 *
 * \f[
 *  (\mathrm{m2}_i,1)^T \mathrm{fmatrix} (\mathrm{m1}_i,1) = 0
 * \f]
 *
 * @param _m1 Contain points in the reference view. Depth CV_32F with 2-channel
 *            1 column or 1-channel 2 columns. It has 7 rows.
 * @param _m2 Contain points in the other view. Depth CV_32F with 2-channel
 *            1 column or 1-channel 2 columns. It has 7 rows.
 * @param _fmatrix Output fundamental matrix (or matrices) of type CV_64FC1.
 *                 The user is responsible for allocating the memory before
 * calling this function.
 * @return Number of fundamental matrices. Valid values are 1, 2 or 3.
 *  - 1, row 0 to row 2 in _fmatrix is a valid fundamental matrix
 *  - 2, row 3 to row 5 in _fmatrix is a valid fundamental matrix
 *  - 3, row 6 to row 8 in _fmatrix is a valid fundamental matrix
 *
 * Note that the computed fundamental matrix is normalized, i.e.,
 * the last element \f$F_{33}\f$ is 1.
 */
static int run7Point(const Mat& _m1, const Mat& _m2, Mat& _fmatrix) {
  double a[7 * 9], w[7], u[9 * 9], v[9 * 9], c[4], r[3] = {0};
  double *f1, *f2;
  double t0, t1, t2;
  Mat A(7, 9, CV_64F, a);
  Mat U(7, 9, CV_64F, u);
  Mat Vt(9, 9, CV_64F, v);
  Mat W(7, 1, CV_64F, w);
  Mat coeffs(1, 4, CV_64F, c);
  Mat roots(1, 3, CV_64F, r);
  const Point2f* m1 = _m1.ptr<Point2f>();
  const Point2f* m2 = _m2.ptr<Point2f>();
  double* fmatrix = _fmatrix.ptr<double>();
  int i, k, n;

  // form a linear system: i-th row of A(=a) represents
  // the equation: (m2[i], 1)'*F*(m1[i], 1) = 0
  for (i = 0; i < 7; i++) {
    double x0 = m1[i].x, y0 = m1[i].y;
    double x1 = m2[i].x, y1 = m2[i].y;

    a[i * 9 + 0] = x1 * x0;
    a[i * 9 + 1] = x1 * y0;
    a[i * 9 + 2] = x1;
    a[i * 9 + 3] = y1 * x0;
    a[i * 9 + 4] = y1 * y0;
    a[i * 9 + 5] = y1;
    a[i * 9 + 6] = x0;
    a[i * 9 + 7] = y0;
    a[i * 9 + 8] = 1;
  }

  // A*(f11 f12 ... f33)' = 0 is singular (7 equations for 9 variables), so
  // the solution is linear subspace of dimensionality 2.
  // => use the last two singular vectors as a basis of the space
  // (according to SVD properties)
  SVDecomp(A, W, U, Vt, SVD::MODIFY_A + SVD::FULL_UV);
  f1 = v + 7 * 9;
  f2 = v + 8 * 9;

  // f1, f2 is a basis => lambda*f1 + mu*f2 is an arbitrary fundamental matrix,
  // as it is determined up to a scale, normalize lambda & mu (lambda + mu = 1),
  // so f ~ lambda*f1 + (1 - lambda)*f2.
  // use the additional constraint det(f) = det(lambda*f1 + (1-lambda)*f2) to
  // find lambda. it will be a cubic equation. find c - polynomial coefficients.
  for (i = 0; i < 9; i++) f1[i] -= f2[i];

  t0 = f2[4] * f2[8] - f2[5] * f2[7];
  t1 = f2[3] * f2[8] - f2[5] * f2[6];
  t2 = f2[3] * f2[7] - f2[4] * f2[6];

  c[3] = f2[0] * t0 - f2[1] * t1 + f2[2] * t2;

  c[2] = f1[0] * t0 - f1[1] * t1 + f1[2] * t2 - f1[3] * (f2[1] * f2[8] - f2[2] * f2[7]) +
         f1[4] * (f2[0] * f2[8] - f2[2] * f2[6]) - f1[5] * (f2[0] * f2[7] - f2[1] * f2[6]) +
         f1[6] * (f2[1] * f2[5] - f2[2] * f2[4]) - f1[7] * (f2[0] * f2[5] - f2[2] * f2[3]) +
         f1[8] * (f2[0] * f2[4] - f2[1] * f2[3]);

  t0 = f1[4] * f1[8] - f1[5] * f1[7];
  t1 = f1[3] * f1[8] - f1[5] * f1[6];
  t2 = f1[3] * f1[7] - f1[4] * f1[6];

  c[1] = f2[0] * t0 - f2[1] * t1 + f2[2] * t2 - f2[3] * (f1[1] * f1[8] - f1[2] * f1[7]) +
         f2[4] * (f1[0] * f1[8] - f1[2] * f1[6]) - f2[5] * (f1[0] * f1[7] - f1[1] * f1[6]) +
         f2[6] * (f1[1] * f1[5] - f1[2] * f1[4]) - f2[7] * (f1[0] * f1[5] - f1[2] * f1[3]) +
         f2[8] * (f1[0] * f1[4] - f1[1] * f1[3]);

  c[0] = f1[0] * t0 - f1[1] * t1 + f1[2] * t2;

  // solve the cubic equation; there can be 1 to 3 roots ...
  n = solveCubic(coeffs, roots);

  if (n < 1 || n > 3) return n;

  for (k = 0; k < n; k++, fmatrix += 9) {
    // for each root form the fundamental matrix
    double lambda = r[k], mu = 1.;
    double s = f1[8] * r[k] + f2[8];

    // normalize each matrix, so that F(3,3) (~fmatrix[8]) == 1
    if (fabs(s) > DBL_EPSILON) {
      mu = 1. / s;
      lambda *= mu;
      fmatrix[8] = 1.;
    } else
      fmatrix[8] = 0.;

    for (i = 0; i < 8; i++) fmatrix[i] = f1[i] * lambda + f2[i] * mu;
  }

  return n;
}

/**
 * Compute the fundamental matrix using the 8-point algorithm.
 *
 * \f[
 *  (\mathrm{m2}_i,1)^T \mathrm{fmatrix} (\mathrm{m1}_i,1) = 0
 * \f]
 *
 * @param _m1 Contain points in the reference view. Depth CV_32F with 2-channel
 *            1 column or 1-channel 2 columns. It has 8 rows.
 * @param _m2 Contain points in the other view. Depth CV_32F with 2-channel
 *            1 column or 1-channel 2 columns. It has 8 rows.
 * @param _fmatrix Output fundamental matrix (or matrices) of type CV_64FC1.
 *                 The user is responsible for allocating the memory before
 * calling this function.
 * @return 1 on success, 0 on failure.
 *
 * Note that the computed fundamental matrix is normalized, i.e.,
 * the last element \f$F_{33}\f$ is 1.
 */
static int run8Point(const Mat& _m1, const Mat& _m2, Mat& _fmatrix) {
  Point2d m1c(0, 0), m2c(0, 0);
  double t, scale1 = 0, scale2 = 0;

  const Point2f* m1 = _m1.ptr<Point2f>();
  const Point2f* m2 = _m2.ptr<Point2f>();
  assert((_m1.cols == 1 || _m1.rows == 1) && _m1.size() == _m2.size());
  int i, count = _m1.checkVector(2);

  // compute centers and average distances for each of the two point sets
  for (i = 0; i < count; i++) {
    m1c += Point2d(m1[i]);
    m2c += Point2d(m2[i]);
  }

  // calculate the normalizing transformations for each of the point sets:
  // after the transformation each set will have the mass center at the
  // coordinate origin and the average distance from the origin will be
  // ~sqrt(2).
  t = 1. / count;
  m1c *= t;
  m2c *= t;

  for (i = 0; i < count; i++) {
    scale1 += norm(Point2d(m1[i].x - m1c.x, m1[i].y - m1c.y));
    scale2 += norm(Point2d(m2[i].x - m2c.x, m2[i].y - m2c.y));
  }

  scale1 *= t;
  scale2 *= t;

  if (scale1 < FLT_EPSILON || scale2 < FLT_EPSILON) return 0;

  scale1 = std::sqrt(2.) / scale1;
  scale2 = std::sqrt(2.) / scale2;

  Matx<double, 9, 9> A;

  // form a linear system Ax=0: for each selected pair of points m1 & m2,
  // the row of A(=a) represents the coefficients of equation: (m2, 1)'*F*(m1,
  // 1) = 0 to save computation time, we compute (At*A) instead of A and then
  // solve (At*A)x=0.
  for (i = 0; i < count; i++) {
    double x1 = (m1[i].x - m1c.x) * scale1;
    double y1 = (m1[i].y - m1c.y) * scale1;
    double x2 = (m2[i].x - m2c.x) * scale2;
    double y2 = (m2[i].y - m2c.y) * scale2;
    Vec<double, 9> r(x2 * x1, x2 * y1, x2, y2 * x1, y2 * y1, y2, x1, y1, 1);
    A += r * r.t();
  }

  Vec<double, 9> W;
  Matx<double, 9, 9> V;

  eigen(A, W, V);

  for (i = 0; i < 9; i++) {
    if (fabs(W[i]) < DBL_EPSILON) break;
  }

  if (i < 8) return 0;

  Matx33d F0(V.val + 9 * 8);  // take the last column of v as a solution of Af = 0

  // make F0 singular (of rank 2) by decomposing it with SVD,
  // zeroing the last diagonal element of W and then composing the matrices
  // back.

  Vec3d w;
  Matx33d U;
  Matx33d Vt;

  SVD::compute(F0, w, U, Vt);
  w[2] = 0.;

  F0 = U * Matx33d::diag(w) * Vt;

  // apply the transformation that is inverse
  // to what we used to normalize the point coordinates
  Matx33d T1(scale1, 0, -scale1 * m1c.x, 0, scale1, -scale1 * m1c.y, 0, 0, 1);
  Matx33d T2(scale2, 0, -scale2 * m2c.x, 0, scale2, -scale2 * m2c.y, 0, 0, 1);

  F0 = T2.t() * F0 * T1;

  // make F(3,3) = 1
  if (fabs(F0(2, 2)) > FLT_EPSILON) F0 *= 1. / F0(2, 2);

  Mat(F0).copyTo(_fmatrix);

  return 1;
}

class FMEstimatorCallback : public PointSetRegistrator::Callback {
 public:
  bool checkSubset(InputArray _ms1, InputArray _ms2, int count) const override {
    Mat ms1 = _ms1.getMat(), ms2 = _ms2.getMat();
    return !haveCollinearPoints(ms1, count) && !haveCollinearPoints(ms2, count);
  }

  int runKernel(InputArray _m1, InputArray _m2, OutputArray _model) const override {
    double f[9 * 3];
    Mat m1 = _m1.getMat(), m2 = _m2.getMat();
    int count = m1.checkVector(2);
    Mat F(count == 7 ? 9 : 3, 3, CV_64F, f);
    int n = count == 7 ? run7Point(m1, m2, F) : run8Point(m1, m2, F);

    if (n == 0)
      _model.release();
    else
      F.rowRange(0, n * 3).copyTo(_model);

    return n;
  }

  void computeError(InputArray _m1, InputArray _m2, InputArray _model, OutputArray _err) const override {
    Mat __m1 = _m1.getMat(), __m2 = _m2.getMat(), __model = _model.getMat();
    int i, count = __m1.checkVector(2);
    const Point2f* m1 = __m1.ptr<Point2f>();
    const Point2f* m2 = __m2.ptr<Point2f>();
    const double* F = __model.ptr<double>();
    _err.create(count, 1, CV_32F);
    float* err = _err.getMat().ptr<float>();

    for (i = 0; i < count; i++) {
      double a, b, c, d1, d2, s1, s2;

      a = F[0] * m1[i].x + F[1] * m1[i].y + F[2];
      b = F[3] * m1[i].x + F[4] * m1[i].y + F[5];
      c = F[6] * m1[i].x + F[7] * m1[i].y + F[8];

      s2 = 1. / (a * a + b * b);
      d2 = m2[i].x * a + m2[i].y * b + c;

      a = F[0] * m2[i].x + F[3] * m2[i].y + F[6];
      b = F[1] * m2[i].x + F[4] * m2[i].y + F[7];
      c = F[2] * m2[i].x + F[5] * m2[i].y + F[8];

      s1 = 1. / (a * a + b * b);
      d1 = m1[i].x * a + m1[i].y * b + c;

      err[i] = (float)std::max(d1 * d1 * s1, d2 * d2 * s2);
    }
  }
};

cv::Mat findFundamentalMat_MaxIter(InputArray _points1, InputArray _points2, int method, double ransacReprojThreshold,
                                   double confidence, OutputArray _mask, int max_iter) {
  // CV_INSTRUMENT_REGION()

  Mat points1 = _points1.getMat(), points2 = _points2.getMat();
  Mat m1, m2, F;
  int npoints = -1;

  for (int i = 1; i <= 2; i++) {
    Mat& p = i == 1 ? points1 : points2;
    Mat& m = i == 1 ? m1 : m2;
    npoints = p.checkVector(2, -1, false);
    if (npoints < 0) {
      npoints = p.checkVector(3, -1, false);
      if (npoints < 0) CV_Error(Error::StsBadArg, "The input arrays should be 2D or 3D point sets");
      if (npoints == 0) return Mat();
      convertPointsFromHomogeneous(p, p);
    }
    p.reshape(2, npoints).convertTo(m, CV_32F);
  }

  assert(m1.checkVector(2) == m2.checkVector(2));

  if (npoints < 7) return Mat();

  Ptr<PointSetRegistrator::Callback> cb = makePtr<FMEstimatorCallback>();
  int result;

  if (npoints == 7 || method == FM_8POINT) {
    result = cb->runKernel(m1, m2, F);
    if (_mask.needed()) {
      _mask.create(npoints, 1, CV_8U, -1, true);
      Mat mask = _mask.getMat();
      assert((mask.cols == 1 || mask.rows == 1) && (int)mask.total() == npoints);
      mask.setTo(Scalar::all(1));
    }
  } else {
    if (ransacReprojThreshold <= 0) ransacReprojThreshold = 3;
    if (confidence < DBL_EPSILON || confidence > 1 - DBL_EPSILON) confidence = 0.99;

    // if( (method & ~3) == FM_RANSAC && npoints >= 15 )
    result = createRANSACPointSetRegistrator_SenseSLAM(cb, 7, ransacReprojThreshold, confidence, max_iter)
                 ->run(m1, m2, F, _mask);
    // else
    //     result = createLMeDSPointSetRegistrator(cb, 7, confidence)->run(m1,
    //     m2, F, _mask);
  }

  if (result <= 0) return Mat();

  return F;
}

inline void convertPointsFromHomogeneous(InputArray _src, OutputArray _dst) {
  // CV_INSTRUMENT_REGION()

  Mat src = _src.getMat();
  if (!src.isContinuous()) src = src.clone();
  int i, npoints = src.checkVector(3), depth = src.depth(), cn = 3;
  if (npoints < 0) {
    npoints = src.checkVector(4);
    assert(npoints >= 0);
    cn = 4;
  }
  assert(npoints >= 0 && (depth == CV_32S || depth == CV_32F || depth == CV_64F));

  int dtype = CV_MAKETYPE(depth <= CV_32F ? CV_32F : CV_64F, cn - 1);
  _dst.create(npoints, 1, dtype);
  Mat dst = _dst.getMat();
  if (!dst.isContinuous()) {
    _dst.release();
    _dst.create(npoints, 1, dtype);
    dst = _dst.getMat();
  }
  assert(dst.isContinuous());

  if (depth == CV_32S) {
    if (cn == 3) {
      const Point3i* sptr = src.ptr<Point3i>();
      Point2f* dptr = dst.ptr<Point2f>();
      for (i = 0; i < npoints; i++) {
        float scale = sptr[i].z != 0 ? 1.f / sptr[i].z : 1.f;
        dptr[i] = Point2f(sptr[i].x * scale, sptr[i].y * scale);
      }
    } else {
      const Vec4i* sptr = src.ptr<Vec4i>();
      Point3f* dptr = dst.ptr<Point3f>();
      for (i = 0; i < npoints; i++) {
        float scale = sptr[i][3] != 0 ? 1.f / sptr[i][3] : 1.f;
        dptr[i] = Point3f(sptr[i][0] * scale, sptr[i][1] * scale, sptr[i][2] * scale);
      }
    }
  } else if (depth == CV_32F) {
    if (cn == 3) {
      const Point3f* sptr = src.ptr<Point3f>();
      Point2f* dptr = dst.ptr<Point2f>();
      for (i = 0; i < npoints; i++) {
        float scale = sptr[i].z != 0.f ? 1.f / sptr[i].z : 1.f;
        dptr[i] = Point2f(sptr[i].x * scale, sptr[i].y * scale);
      }
    } else {
      const Vec4f* sptr = src.ptr<Vec4f>();
      Point3f* dptr = dst.ptr<Point3f>();
      for (i = 0; i < npoints; i++) {
        float scale = sptr[i][3] != 0.f ? 1.f / sptr[i][3] : 1.f;
        dptr[i] = Point3f(sptr[i][0] * scale, sptr[i][1] * scale, sptr[i][2] * scale);
      }
    }
  } else if (depth == CV_64F) {
    if (cn == 3) {
      const Point3d* sptr = src.ptr<Point3d>();
      Point2d* dptr = dst.ptr<Point2d>();
      for (i = 0; i < npoints; i++) {
        double scale = sptr[i].z != 0. ? 1. / sptr[i].z : 1.;
        dptr[i] = Point2d(sptr[i].x * scale, sptr[i].y * scale);
      }
    } else {
      const Vec4d* sptr = src.ptr<Vec4d>();
      Point3d* dptr = dst.ptr<Point3d>();
      for (i = 0; i < npoints; i++) {
        double scale = sptr[i][3] != 0.f ? 1. / sptr[i][3] : 1.;
        dptr[i] = Point3d(sptr[i][0] * scale, sptr[i][1] * scale, sptr[i][2] * scale);
      }
    }
  } else
    CV_Error(Error::StsUnsupportedFormat, "");
}

}  // namespace cv
/* End of file. */
