#ifndef ITSLAM_FIVE_POINT_H
#define ITSLAM_FIVE_POINT_H

#include <array>

#include "base/map.h"
#include "utility/global.h"

namespace xrsfm {

double sampson_error(const matrix<3> &E, const vector<2> &p1, const vector<2> &p2);

void decompose_essential(const matrix<3> &E, matrix<3> &R1, matrix<3> &R2, vector<3> &T);

// p2^T * E * p1 = 0
std::vector<matrix<3>> solve_essential_5pt(const std::array<vector<2>, 5> &points1,
                                           const std::array<vector<2>, 5> &points2);

// d(p2, E * p1)
inline double essential_geometric_error(const matrix<3> &E, const vector<2> &p1, const vector<2> &p2) {
  vector<3> Ep1 = E * p1.homogeneous();
  double r = p2.homogeneous().transpose() * Ep1;
  return r * r / Ep1.segment<2>(0).squaredNorm();
}

matrix<3> find_essential_matrix(const std::vector<vector<2>> &points1, const std::vector<vector<2>> &points2,
                                double threshold, double confidence, size_t max_iteration, int seed);

void solve_essential(const std::vector<vector<2>> &points1, const std::vector<vector<2>> &points2,
                     const double threshold, matrix<3> &E, int &inlier_num, std::vector<char> &inlier_mask);

vector<4> triangulate_point(const matrix<3, 4> P1, const matrix<3, 4> P2, const vector<2> point1,
                            const vector<2> point2);

bool check_point(const matrix<3, 4> P1, const matrix<3, 4> P2, const vector<2> point1, const vector<2> point2,
                 vector<3> &p);

int check_essential_rt(const std::vector<vector<2>> &points1, const std::vector<vector<2>> &points2, const matrix<3> R,
                       const vector<3> T, std::vector<vector<3>> &result_points, std::vector<char> &result_status);

void decompose_rt(const Eigen::Matrix3d &E, const std::vector<vector<2>> &points1,
                  const std::vector<vector<2>> &points2, matrix<3> &R, vector<3> &T,
                  std::vector<vector<3>> &result_points, std::vector<char> &result_status);

}  // namespace xrsfm

#endif  // ITSLAM_FIVE_POINT_H
