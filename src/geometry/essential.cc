#include "essential.h"

#include <math.h>

#include "utility/random.h"

namespace itslam {
namespace {
class Polynomial {
 public:
  // clang-format off
            enum GRevLexMonomials {
                XXX = 0, XXY = 1, XYY = 2, YYY = 3, XXZ = 4, XYZ = 5, YYZ = 6, XZZ = 7, YZZ = 8, ZZZ = 9,
                XX = 10, XY = 11, YY = 12, XZ = 13, YZ = 14, ZZ = 15, X = 16, Y = 17, Z = 18, I = 19
            };
  // clang-format on

  vector<20> v;

  Polynomial(const vector<20> &coeffcients) : v(coeffcients) {}

 public:
  Polynomial() : Polynomial(vector<20>::Zero()) {}

  Polynomial(double w) {
    v.setZero();
    v[I] = w;
  }

  void set_xyzw(double x, double y, double z, double w) {
    v.setZero();
    v[X] = x;
    v[Y] = y;
    v[Z] = z;
    v[I] = w;
  }

  Polynomial operator-() const { return Polynomial(-v); }

  Polynomial operator+(const Polynomial &b) const { return Polynomial(v + b.v); }

  Polynomial operator-(const Polynomial &b) const { return Polynomial(v - b.v); }

  Polynomial operator*(const Polynomial &b) const {
    Polynomial r;

    r.v[I] = v[I] * b.v[I];

    r.v[Z] = v[I] * b.v[Z] + v[Z] * b.v[I];
    r.v[Y] = v[I] * b.v[Y] + v[Y] * b.v[I];
    r.v[X] = v[I] * b.v[X] + v[X] * b.v[I];

    r.v[ZZ] = v[I] * b.v[ZZ] + v[Z] * b.v[Z] + v[ZZ] * b.v[I];
    r.v[YZ] = v[I] * b.v[YZ] + v[Z] * b.v[Y] + v[Y] * b.v[Z] + v[YZ] * b.v[I];
    r.v[XZ] = v[I] * b.v[XZ] + v[Z] * b.v[X] + v[X] * b.v[Z] + v[XZ] * b.v[I];
    r.v[YY] = v[I] * b.v[YY] + v[Y] * b.v[Y] + v[YY] * b.v[I];
    r.v[XY] = v[I] * b.v[XY] + v[Y] * b.v[X] + v[X] * b.v[Y] + v[XY] * b.v[I];
    r.v[XX] = v[I] * b.v[XX] + v[X] * b.v[X] + v[XX] * b.v[I];

    r.v[ZZZ] = v[I] * b.v[ZZZ] + v[Z] * b.v[ZZ] + v[ZZ] * b.v[Z] + v[ZZZ] * b.v[I];
    r.v[YZZ] = v[I] * b.v[YZZ] + v[Z] * b.v[YZ] + v[Y] * b.v[ZZ] + v[ZZ] * b.v[Y] + v[YZ] * b.v[Z] + v[YZZ] * b.v[I];
    r.v[XZZ] = v[I] * b.v[XZZ] + v[Z] * b.v[XZ] + v[X] * b.v[ZZ] + v[ZZ] * b.v[X] + v[XZ] * b.v[Z] + v[XZZ] * b.v[I];
    r.v[YYZ] = v[I] * b.v[YYZ] + v[Z] * b.v[YY] + v[Y] * b.v[YZ] + v[YZ] * b.v[Y] + v[YY] * b.v[Z] + v[YYZ] * b.v[I];
    r.v[XYZ] = v[I] * b.v[XYZ] + v[Z] * b.v[XY] + v[Y] * b.v[XZ] + v[X] * b.v[YZ] + v[YZ] * b.v[X] + v[XZ] * b.v[Y] +
               v[XY] * b.v[Z] + v[XYZ] * b.v[I];
    r.v[XXZ] = v[I] * b.v[XXZ] + v[Z] * b.v[XX] + v[X] * b.v[XZ] + v[XZ] * b.v[X] + v[XX] * b.v[Z] + v[XXZ] * b.v[I];
    r.v[YYY] = v[I] * b.v[YYY] + v[Y] * b.v[YY] + v[YY] * b.v[Y] + v[YYY] * b.v[I];
    r.v[XYY] = v[I] * b.v[XYY] + v[Y] * b.v[XY] + v[X] * b.v[YY] + v[YY] * b.v[X] + v[XY] * b.v[Y] + v[XYY] * b.v[I];
    r.v[XXY] = v[I] * b.v[XXY] + v[Y] * b.v[XX] + v[X] * b.v[XY] + v[XY] * b.v[X] + v[XX] * b.v[Y] + v[XXY] * b.v[I];
    r.v[XXX] = v[I] * b.v[XXX] + v[X] * b.v[XX] + v[XX] * b.v[X] + v[XXX] * b.v[I];

    return r;
  }

  const vector<20> &coeffcients() const { return v; }
};

Polynomial operator*(const double &scale, const Polynomial &poly) { return Polynomial(scale * poly.coeffcients()); }

inline matrix<3> to_matrix(const vector<9> &vec) {
  return (matrix<3>() << vec.segment<3>(0), vec.segment<3>(3), vec.segment<3>(6)).finished();
}

inline matrix<9, 4> generate_nullspace_basis(const std::array<vector<2>, 5> &points1,
                                             const std::array<vector<2>, 5> &points2) {
  matrix<5, 9> A;
  for (size_t i = 0; i < 5; ++i) {
    matrix<3> h = vector<3>(points1[i].homogeneous()) * points2[i].homogeneous().transpose();
    for (size_t j = 0; j < 3; ++j) {
      A.block<1, 3>(i, j * 3) = h.row(j);
    }
  }
  return A.jacobiSvd(Eigen::ComputeFullV).matrixV().block<9, 4>(0, 5);
}

inline matrix<10, 20> generate_polynomials(const matrix<9, 4> &basis) {
  typedef matrix<3, 3, false, Polynomial> matrix_poly;
  matrix<3> Ex = to_matrix(basis.col(0));
  matrix<3> Ey = to_matrix(basis.col(1));
  matrix<3> Ez = to_matrix(basis.col(2));
  matrix<3> Ew = to_matrix(basis.col(3));

  matrix_poly Epoly;
  for (size_t i = 0; i < 3; ++i) {
    for (size_t j = 0; j < 3; ++j) {
      Epoly(i, j).set_xyzw(Ex(i, j), Ey(i, j), Ez(i, j), Ew(i, j));
    }
  }

  matrix<10, 20> polynomials;

  matrix_poly EEt = Epoly * Epoly.transpose();
  matrix_poly singular_value_constraints = (EEt * Epoly) - (0.5 * EEt.trace()) * Epoly;
  for (size_t i = 0; i < 3; ++i) {
    for (size_t j = 0; j < 3; ++j) {
      polynomials.row(i * 3 + j) = singular_value_constraints(i, j).coeffcients();
    }
  }

  Polynomial detE = Epoly.determinant();
  polynomials.row(9) = detE.coeffcients();

  return polynomials;
}

inline matrix<10> generate_action_matrix(matrix<10, 20> &polynomials) {
  std::array<size_t, 10> perm;
  for (size_t i = 0; i < 10; ++i) {
    perm[i] = i;
  }
  for (size_t i = 0; i < 10; ++i) {
    for (size_t j = i + 1; j < 10; ++j) {
      if (abs(polynomials(perm[i], i)) < abs(polynomials(perm[j], i))) {
        std::swap(perm[i], perm[j]);
      }
    }
    if (polynomials(perm[i], i) == 0) continue;
    polynomials.row(perm[i]) /= polynomials(perm[i], i);
    for (size_t j = i + 1; j < 10; ++j) {
      polynomials.row(perm[j]) -= polynomials.row(perm[i]) * polynomials(perm[j], i);
    }
  }
  for (size_t i = 9; i > 0; --i) {
    for (size_t j = 0; j < i; ++j) {
      polynomials.row(perm[j]) -= polynomials.row(perm[i]) * polynomials(perm[j], i);
    }
  }

  matrix<10> action;
  action.row(0) = -polynomials.block<1, 10>(perm[Polynomial::XXX], Polynomial::XX);
  action.row(1) = -polynomials.block<1, 10>(perm[Polynomial::XXY], Polynomial::XX);
  action.row(2) = -polynomials.block<1, 10>(perm[Polynomial::XYY], Polynomial::XX);
  action.row(3) = -polynomials.block<1, 10>(perm[Polynomial::XXZ], Polynomial::XX);
  action.row(4) = -polynomials.block<1, 10>(perm[Polynomial::XYZ], Polynomial::XX);
  action.row(5) = -polynomials.block<1, 10>(perm[Polynomial::XZZ], Polynomial::XX);
  action.row(6) = vector<10>::Unit(Polynomial::XX - Polynomial::XX).transpose();
  action.row(7) = vector<10>::Unit(Polynomial::XY - Polynomial::XX).transpose();
  action.row(8) = vector<10>::Unit(Polynomial::XZ - Polynomial::XX).transpose();
  action.row(9) = vector<10>::Unit(Polynomial::X - Polynomial::XX).transpose();

  return action;
}

inline std::vector<vector<3>> solve_grobner_system(const matrix<10> &action) {
  Eigen::EigenSolver<matrix<10>> eigen(action, true);
  vector<10, std::complex<double>> xs = eigen.eigenvalues();

  std::vector<vector<3>> results;
  for (size_t i = 0; i < 10; ++i) {
    if (abs(xs[i].imag()) < 1.0e-10) {
      vector<10> h = eigen.eigenvectors().col(i).real();
      double xw = h(Polynomial::X - Polynomial::XX);
      double yw = h(Polynomial::Y - Polynomial::XX);
      double zw = h(Polynomial::Z - Polynomial::XX);
      double w = h(Polynomial::I - Polynomial::XX);
      results.emplace_back(xw / w, yw / w, zw / w);
    }
  }
  return results;
}
}  // namespace

void decompose_essential(const matrix<3> &E, matrix<3> &R1, matrix<3> &R2, vector<3> &T) {
#ifdef ESSENTIAL_DECOMPOSE_HORN
  matrix<3> EET = E * E.transpose();
  double halfTrace = 0.5 * EET.trace();
  vector<3> b;

  vector<3> e0e1 = E.col(0).cross(E.col(1));
  vector<3> e1e2 = E.col(1).cross(E.col(2));
  vector<3> e2e0 = E.col(2).cross(E.col(0));

#if ESSENTIAL_DECOMPOSE_HORN == 1
  if (e0e1.norm() > e1e2.norm() && e0e1.norm() > e2e0.norm()) {
    b = e0e1.normalized() * sqrt(halfTrace);
  } else if (e1e2.norm() > e0e1.norm() && e1e2.norm() > e2e0.norm()) {
    b = e1e2.normalized() * sqrt(halfTrace);
  } else {
    b = e2e0.normalized() * sqrt(halfTrace);
  }
#else
  matrix<3> bbT = halfTrace * matrix<3>::Identity() - EET;
  vector<3> bbT_diag = bbT.diagonal();
  if (bbT_diag(0) > bbt_diag(1) && bbT_diag(0) > bbT_diag(2)) {
    b = bbT.row(0) / sqrt(bbT_diag(0));
  } else if (bbT_diag(1) > bbT_diag(0) && bbT_diag(1) > bbT_diag(2)) {
    b = bbT.row(1) / sqrt(bbT_diag(1));
  } else {
    b = bbT.row(2) / sqrt(bbT_diag(2));
  }
#endif

  matrix<3> cofactorsT;
  cofactorsT.col(0) = e1e2;
  cofactorsT.col(1) = e2e0;
  cofactorsT.col(2) = e0e1;

  matrix<3> skew_b;
  skew_b << 0, -b.z(), b.y(), b.z(), 0, -b.x(), -b.y(), b.x(), 0;
  matrix<3> bxE = skew_b * E;

  double bTb = b.dot(b);
  R1 = (cofactorsT - bxE) / bTb;
  R2 = (cofactorsT + bxE) / bTb;
  T = b;
#else
  Eigen::JacobiSVD<matrix<3>> svd(E, Eigen::ComputeFullU | Eigen::ComputeFullV);
  matrix<3> U = svd.matrixU();
  matrix<3> VT = svd.matrixV().transpose();
  if (U.determinant() < 0) {
    U = -U;
  }
  if (VT.determinant() < 0) {
    VT = -VT;
  }
  matrix<3> W;
  W << 0, 1, 0, -1, 0, 0, 0, 0, 1;
  R1 = U * W * VT;
  R2 = U * W.transpose() * VT;
  T = U.col(2);
#endif
}

double sampson_error(const matrix<3> &E, const vector<2> &p1, const vector<2> &p2) {
  vector<3> Ep1 = E * p1.homogeneous();
  vector<3> Etp2 = E.transpose() * p2.homogeneous();
  double r = p2.homogeneous().transpose() * Ep1;
  return r * r * (1 / Ep1.segment<2>(0).squaredNorm() + 1 / Etp2.segment<2>(0).squaredNorm());
}

std::vector<matrix<3>> solve_essential_5pt(const std::array<vector<2>, 5> &points1,
                                           const std::array<vector<2>, 5> &points2) {
  matrix<9, 4> basis = generate_nullspace_basis(points1, points2);
  matrix<10, 20> polynomials = generate_polynomials(basis);
  matrix<10> action = generate_action_matrix(polynomials);
  std::vector<vector<3>> solutions = solve_grobner_system(action);
  std::vector<matrix<3>> results(solutions.size());
  for (size_t i = 0; i < solutions.size(); ++i) {
    results[i] = to_matrix(basis * solutions[i].homogeneous());
  }
  return results;
}

matrix<3> find_essential_matrix(const std::vector<vector<2>> &points1, const std::vector<vector<2>> &points2,
                                double threshold, double confidence, size_t max_iteration, int seed) {
  // Diagnosis::TimingItem timing = Diagnosis::time(DI_F_ESSENTIAL_RANSAC_TIME);
#if 0
        ITSLAM_UNUSED_EXPR(max_iteration);
    std::vector<cv::Point2f> cvPoints1, cvPoints2;
    for (size_t i = 0; i < points1.size(); ++i) {
        cvPoints1.emplace_back(float(points1[i].x()), float(points1[i].y()));
        cvPoints2.emplace_back(float(points2[i].x()), float(points2[i].y()));
    }
    cv::Mat cvE = cv::findEssentialMat(cvPoints1, cvPoints2, cv::Mat::eye(3, 3, CV_32FC1), cv::RANSAC, confidence, threshold, cv::noArray());
    // workaround: fuck opencv
    if (cvE.rows != 3 || cvE.cols != 3) {
        return matrix<3>::Identity() * std::numeric_limits<double>::quiet_NaN();
    }
    matrix<3> E;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            E(i, j) = cvE.at<double>(i, j);
        }
    }
    return E;
#else
  LotBox lotbox(points1.size());
  lotbox.seed(seed);
  double K = log(1 - confidence);
  double threshold_sq = 2 * threshold * threshold;

  matrix<3> best_E = matrix<3>::Constant(std::numeric_limits<double>::quiet_NaN());
  size_t best_inlier = 0;
  double best_score = std::numeric_limits<double>::max();

  size_t iter_max = max_iteration;
  for (size_t iter = 0; iter < iter_max; ++iter) {
    // generate hypothesis
    lotbox.refill_all();
    std::array<vector<2>, 5> pts1, pts2;
    for (size_t i = 0; i < 5; ++i) {
      size_t sample_id = lotbox.draw_without_replacement();
      pts1[i] = points1[sample_id];
      pts2[i] = points2[sample_id];
    }

    // solve essential matrix
    std::vector<matrix<3>> Es = solve_essential_5pt(pts1, pts2);

    // find best hypothesis
    for (size_t i = 0; i < Es.size(); ++i) {
      const matrix<3> &E = Es[i];
      size_t inlier = 0;
      double score = 0.0;
      for (size_t j = 0; j < points1.size(); ++j) {
        double se = sampson_error(E, points1[j], points2[j]);
        score += log(1.0 + se * se / threshold_sq);
        if (se < threshold_sq) {
          inlier++;
        }
      }
      // update current best model and iteration limit
      if (inlier > best_inlier || (inlier == best_inlier && best_inlier > 0 && score < best_score)) {
        best_E = E;
        best_inlier = inlier;
        best_score = score;

        double inlier_ratio = best_inlier / (double)points1.size();
        double N = K / log(1 - pow(inlier_ratio, 5));
        if (N < (double)iter_max) {  // guard for NaN
          iter_max = (size_t)ceil(N);
        }
      }
    }
  }
  // printf("iter_num: %d inlier_num: %d\n", iter_max, best_inlier);
  return best_E;
#endif
}

void solve_essential(const std::vector<vector<2>> &points1, const std::vector<vector<2>> &points2,
                     const double threshold, matrix<3> &E, int &inlier_num, std::vector<char> &inlier_mask) {
  E = find_essential_matrix(points1, points2, threshold, 0.9, 50, 0);  // slow
  inlier_num = 0;
  inlier_mask.assign(points1.size(), 0);
  double threshold_sq = 2 * threshold * threshold;
  for (size_t i = 0; i < points1.size(); ++i) {
    double se = sampson_error(E, points1[i], points2[i]);
    if (se < threshold_sq) {
      inlier_num++;
      inlier_mask[i] = 1;
    }
  }
}

vector<4> triangulate_point(const matrix<3, 4> P1, const matrix<3, 4> P2, const vector<2> point1,
                            const vector<2> point2) {
  matrix<4> A;
  A.row(0) = point1(0) * P1.row(2) - P1.row(0);
  A.row(1) = point1(1) * P1.row(2) - P1.row(1);
  A.row(2) = point2(0) * P2.row(2) - P2.row(0);
  A.row(3) = point2(1) * P2.row(2) - P2.row(1);
  vector<4> q = A.jacobiSvd(Eigen::ComputeFullV).matrixV().col(3);
  return q;
}

bool check_point(const matrix<3, 4> P1, const matrix<3, 4> P2, const vector<2> point1, const vector<2> point2,
                 vector<3> &p) {
  vector<4> q = triangulate_point(P1, P2, point1, point2);

  vector<3> q1 = P1 * q;
  vector<3> q2 = P2 * q;

  if (q1[2] * q[3] > 0 && q2[2] * q[3] > 0 && q1[2] / q[3] < 100 && q2[2] / q[3] < 100) {
    p = q.hnormalized();
    return true;
  }
  return false;
}

int check_essential_rt(const std::vector<vector<2>> &points1, const std::vector<vector<2>> &points2, const matrix<3> R,
                       const vector<3> T, std::vector<vector<3>> &result_points, std::vector<char> &result_status) {
  // set P1 P2
  matrix<3, 4> P1, P2;
  P1.setIdentity();
  P2 << R, T;
  // triangulate
  result_points.resize(points1.size());
  result_status.resize(points1.size());

  int count = 0;
  for (size_t i = 0; i < points1.size(); ++i) {
    vector<3> p;
    if (check_point(P1, P2, points1[i], points2[i], result_points[i])) {
      result_status[i] = 1;
      count++;
    } else {
      result_status[i] = 0;
    }
  }

  // printf("%d %d\n", count, points1.size());

  return count;
}

void decompose_rt(const Eigen::Matrix3d &E, const std::vector<vector<2>> &points1,
                  const std::vector<vector<2>> &points2, matrix<3> &R, vector<3> &T,
                  std::vector<vector<3>> &result_points, std::vector<char> &result_status) {
  // decompose RT
  matrix<3> R1, R2;
  vector<3> T1;
  decompose_essential(E, R1, R2, T1);
  matrix<3> Rs[] = {R1, R1, R2, R2};
  vector<3> Ts[] = {T1, -T1, T1, -T1};
  // get good R T
  std::array<std::vector<vector<3>>, 4> points_array;
  std::array<std::vector<char>, 4> status_array;
  int best_num = 0;
  int best_k = 0;
  for (int k = 0; k < 4; ++k) {
    int goods = check_essential_rt(points1, points2, Rs[k], Ts[k], points_array[k], status_array[k]);
    if (goods > best_num) {
      best_num = goods;
      best_k = k;
    }
  }
  R = Rs[best_k];
  T = Ts[best_k];
  result_points = points_array[best_k];
  result_status = status_array[best_k];
}
}  // namespace itslam

double CheckEssential(Map &map, FramePair &fp) {
  Eigen::Matrix3d K = Eigen::Matrix3d::Zero();
  K(0, 0) = map.cameras_[0].fx();
  K(1, 1) = map.cameras_[0].fy();
  K(0, 2) = map.cameras_[0].cx();
  K(1, 2) = map.cameras_[0].cy();
  K(2, 2) = 1;

  int num_matches = 0;
  int num_inliers = 0;
  const double th = 8;
  //    auto &image = images[fp.id2];
  auto &frame1 = map.frames_[fp.id1];
  auto &frame2 = map.frames_[fp.id2];
  for (int i = 0; i < fp.matches.size(); ++i) {
    if (!fp.inlier_mask[i]) continue;
    num_matches++;

    Eigen::Vector3d ray1 = frame1.qwc() * frame1.points_normalized[fp.matches[i].id1].homogeneous();
    Eigen::Vector3d ray2 = frame2.qwc() * frame2.points_normalized[fp.matches[i].id2].homogeneous();
    Eigen::Vector3d t = frame1.center() - frame2.center();
    Eigen::Vector3d el = K.inverse().transpose() * (frame2.qwc().inverse() * ray1.cross(t));
    Eigen::Vector2d pt = frame2.points[fp.matches[i].id2];
    Eigen::Vector2d ptn = frame2.points_normalized[fp.matches[i].id2];
    // std::cout << pt.homogeneous().transpose() << " " << (K *
    // ptn.homogeneous()).transpose() << std::endl; std::cout << (K.inverse() *
    // pt.homogeneous()).transpose() << " " << ptn.homogeneous().transpose() <<
    // std::endl; std::cout<<ray1.cross(t).dot(ray2)<<std::endl;
    // std::cout<<el.dot(frame2.points_normalized[fp.matches[i].id2].homogeneous())<<std::endl;

    double distance_epipolar = abs(el.dot(pt.homogeneous())) / (el.head<2>().norm());
    if (distance_epipolar < th) {
      num_inliers++;
    }
  }
  const double ratio = 1.0 * num_inliers / num_matches;
  //    printf("CheckEssential:%d %d %d %d %lf\n", fp.id1, fp.id2, num_inliers,
  //           num_matches, ratio);
  if (ratio < 0.5) {
    //        DrawFeatureMatches(images[fp.id1], images[fp.id2], frame1.points,
    //        frame2.points,
    //                           fp.matches, fp.inlier_mask);
    //        DrawFeatureMatches1(images[fp.id1], frame1.points, frame2.points,
    //                            fp.matches, fp.inlier_mask);
    //        cv::waitKey();
    //        for (int i = 0; i < fp.matches.size(); ++i) {
    //            if (!fp.inlier_mask[i])continue;
    //            num_matches++;
    //            Eigen::Vector3d ray1 = frame1.qwc() *
    //            frame1.points_normalized[fp.matches[i].id1].homogeneous();
    //            Eigen::Vector3d ray2 = frame2.qwc() *
    //            frame2.points_normalized[fp.matches[i].id2].homogeneous();
    //            Eigen::Vector3d t = frame1.center() - frame2.center();
    //            Eigen::Vector3d el = K.inverse().transpose() *
    //            (frame2.qwc().inverse() * ray1.cross(t)); auto pt =
    //            frame2.points[fp.matches[i].id2]; double distance_epipolar =
    //            abs(el.dot(pt.homogeneous())) / (el.head<2>().norm());
    //            std::cout<<distance_epipolar<<std::endl;
    //            cv::circle(image, cv::Point(pt.x(), pt.y()), 5, cv::Scalar(0,
    //            0, 255)); line(image, cv::Point(0, -el.z() / el.y()),
    //            cv::Point(image.cols, -(el.z() + el.x() * image.cols) /
    //            el.y()),
    //                 cv::Scalar(0, 255, 0));
    //            cv::imshow("-", image);
    //            cv::waitKey();
    //        }
  }
  return ratio;
}

double CheckColmapEssential(Map &map, FramePair &fp) {
  int num_matches = 0;
  int num_inliers = 0;
  constexpr double th = 1.0;
  const auto &frame1 = map.frames_[fp.id1];
  const auto &frame2 = map.frames_[fp.id2];
  const Eigen::Vector3d t = frame1.center() - frame2.center();
  for (int i = 0; i < fp.matches.size(); ++i) {
    if (!fp.inlier_mask[i]) continue;
    num_matches++;
    // compute deg between ray2 and plane_ray1-t
    Eigen::Vector2d p2d1 = frame1.points_normalized[fp.matches[i].id1];
    Eigen::Vector2d p2d2 = frame2.points_normalized[fp.matches[i].id2];
    Eigen::Vector3d ray1 = frame1.qwc() * p2d1.homogeneous().normalized();
    Eigen::Vector3d ray2 = frame2.qwc() * p2d2.homogeneous().normalized();
    Eigen::Vector3d n = (ray1.cross(t)).normalized();  // norm of plane_ray1-t
    const double sin_theta = std::abs(n.dot(ray2));
    if (sin_theta < sin(th * M_PI / 180)) {
      num_inliers++;
    }
  }
  const double ratio = 1.0 * num_inliers / num_matches;
  // if (ratio < 0.5) {
  //   Eigen::Matrix3d K = Eigen::Matrix3d::Zero();
  //   K(0, 0) = map.cameras_[0].fx();
  //   K(1, 1) = map.cameras_[0].fy();
  //   K(0, 2) = map.cameras_[0].cx();
  //   K(1, 2) = map.cameras_[0].cy();
  //   K(2, 2) = 1;

  //   DrawFeatureMatches(images[fp.id1], images[fp.id2], frame1.points, frame2.points, fp.matches,
  //                      fp.inlier_mask);
  //   DrawFeatureMatches1(images[fp.id1], frame1.points, frame2.points, fp.matches,
  //   fp.inlier_mask); cv::waitKey(); for (int i = 0; i < fp.matches.size(); ++i) {
  //     if (!fp.inlier_mask[i]) continue;
  //     num_matches++;
  //     Eigen::Vector3d ray1 =
  //         frame1.qwc() * frame1.points_normalized[fp.matches[i].id1].homogeneous();
  //     Eigen::Vector3d ray2 =
  //         frame2.qwc() * frame2.points_normalized[fp.matches[i].id2].homogeneous();
  //     Eigen::Vector3d t = frame1.center() - frame2.center();
  //     Eigen::Vector3d el = K.inverse().transpose() * (frame2.qwc().inverse() * ray1.cross(t));
  //     auto pt = frame2.points[fp.matches[i].id2];
  //     double distance_epipolar = abs(el.dot(pt.homogeneous())) / (el.head<2>().norm());
  //     std::cout << distance_epipolar << std::endl;
  //     cv::circle(image, cv::Point(pt.x(), pt.y()), 5, cv::Scalar(0, 0, 255));
  //     line(image, cv::Point(0, -el.z() / el.y()),
  //          cv::Point(image.cols, -(el.z() + el.x() * image.cols) / el.y()), cv::Scalar(0, 255,
  //          0));
  //     cv::imshow("-", image);
  //     cv::waitKey();
  //   }
  // }
  return ratio;
}

bool CheckAllEssential(Map &map, int frame_id) {
  int num_inlier = 0;
  int num_all = 0;
  for (auto &fp : map.frame_pairs_) {
    if (fp.id1 == frame_id || fp.id2 == frame_id) {
      if (map.frames_[fp.id1].registered && map.frames_[fp.id2].registered) {
        double ratio = CheckEssential(map, fp);
        if (ratio >= 0.6) ++num_inlier;
        ++num_all;
      }
    }
  }
  if (num_inlier < 0.6 * num_all) {
    std::cerr << "Bad Registeration\n";
    for (auto &fp : map.frame_pairs_) {
      if (fp.id1 == frame_id || fp.id2 == frame_id) {
        if (map.frames_[fp.id1].registered && map.frames_[fp.id2].registered) {
          double ratio = CheckEssential(map, fp);
          printf("%lf\n", ratio);
        }
      }
    }
    return false;
  }
  return true;
}
