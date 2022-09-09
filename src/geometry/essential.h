#ifndef XRSFM_SRC_GEOMETRY_ESSENTIAL_H
#define XRSFM_SRC_GEOMETRY_ESSENTIAL_H

#include <array>

#include "base/map.h"
#include "utility/global.h"

namespace xrsfm {

double sampson_error(const matrix3 &E, const vector2 &p1, const vector2 &p2);

void decompose_essential(const matrix3 &E, matrix3 &R1, matrix3 &R2,
                         vector3 &T);

// p2^T * E * p1 = 0
std::vector<matrix3> solve_essential_5pt(const std::array<vector2, 5> &points1,
                                         const std::array<vector2, 5> &points2);

// d(p2, E * p1)
inline double essential_geometric_error(const matrix3 &E, const vector2 &p1,
                                        const vector2 &p2) {
    vector3 Ep1 = E * p1.homogeneous();
    double r = p2.homogeneous().transpose() * Ep1;
    return r * r / Ep1.segment<2>(0).squaredNorm();
}

matrix3 find_essential_matrix(const std::vector<vector2> &points1,
                              const std::vector<vector2> &points2,
                              double threshold, double confidence,
                              size_t max_iteration, int seed);

void solve_essential(const std::vector<vector2> &points1,
                     const std::vector<vector2> &points2,
                     const double threshold, matrix3 &E, int &inlier_num,
                     std::vector<char> &inlier_mask);

vector<4> triangulate_point(const matrix<3, 4> P1, const matrix<3, 4> P2,
                            const vector2 point1, const vector2 point2);

bool check_point(const matrix<3, 4> P1, const matrix<3, 4> P2,
                 const vector2 point1, const vector2 point2, vector3 &p);

int check_essential_rt(const std::vector<vector2> &points1,
                       const std::vector<vector2> &points2, const matrix3 R,
                       const vector3 T, std::vector<vector3> &result_points,
                       std::vector<char> &result_status);

void decompose_rt(const matrix3 &E, const std::vector<vector2> &points1,
                  const std::vector<vector2> &points2, matrix3 &R, vector3 &T,
                  std::vector<vector3> &result_points,
                  std::vector<char> &result_status);

} // namespace xrsfm

#endif // ITSLAM_FIVE_POINT_H
