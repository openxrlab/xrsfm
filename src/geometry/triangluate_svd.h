//
// Created by SENSETIME\yezhichao1 on 2020/12/30.
//

#ifndef XRSFM_SRC_GEOMETRY_TRIANGLUATE_SVD_H
#define XRSFM_SRC_GEOMETRY_TRIANGLUATE_SVD_H

#include "utility/global.h"

namespace xrsfm {
inline double DegToRad(const double deg) {
    return deg * 0.0174532925199432954743716805978692718781530857086181640625;
}

inline size_t NChooseK(const size_t n, const size_t k) {
    if (k == 0) {
        return 1;
    }
    return (n * NChooseK(n - 1, k - 1)) / k;
}

bool triangulate_point_checked(const std::vector<xrsfm::matrix<3, 4>> &Ps,
                               const std::vector<xrsfm::vector<2>> &points, xrsfm::vector<3> &p);

double CalculateTriangulationAngle(const Eigen::Vector3d &center1, const Eigen::Vector3d &center2,
                                   const Eigen::Vector3d &point3D);
} // namespace xrsfm

#endif // XRSFM_SRC_GEOMETRY_TRIANGLUATE_SVD_H
