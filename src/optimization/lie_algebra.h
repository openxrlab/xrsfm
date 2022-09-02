//
// Created by SENSETIME\yezhichao1 on 2020/9/27.
//

#ifndef XRSFM_SRC_OPTIMIZATION_LIE_ALGEBRA_H
#define XRSFM_SRC_OPTIMIZATION_LIE_ALGEBRA_H

#include "utility/global.h"

namespace xrsfm {

inline vector3 logmap(const Eigen::Quaterniond &q) {
    Eigen::AngleAxisd aa(q);
    return aa.angle() * aa.axis();
}

inline Eigen::Quaterniond expmap(const vector3 &w) {
    Eigen::AngleAxisd aa(w.norm(), w.stableNormalized());
    return Eigen::Quaterniond(aa);
}

inline matrix3 skewSymmetric(const vector3 &q) {
    matrix3 ans;
    ans << 0, -q(2), q(1), q(2), 0, -q(0), -q(1), q(0), 0;
    return ans;
}

inline matrix3 jr(vector3 theta) {
    double norm = theta.norm();
    matrix3 jr;
    if (norm < 1.745329252e-7) {
        jr = matrix3::Identity() - 1.0 / 2.0 * skewSymmetric(theta) +
             1.0 / 6.0 * skewSymmetric(theta) * skewSymmetric(theta);
    } else {
        jr = matrix3::Identity() -
             (1.0 - cos(norm)) / (norm * norm) * skewSymmetric(theta) +
             (norm - sin(norm)) / (norm * norm * norm) * skewSymmetric(theta) *
                 skewSymmetric(theta);
    }
    return jr;
}

inline matrix3 jri(vector3 theta) {
    double norm = theta.norm();
    matrix3 jri;
    if (norm < 1.745329252e-7) {
        jri = matrix3::Identity() + 1.0 / 2.0 * skewSymmetric(theta) +
              1.0 / 12.0 * skewSymmetric(theta) * skewSymmetric(theta);
    } else {
        jri = matrix3::Identity() + 1.0 / 2.0 * skewSymmetric(theta) +
              ((1.0) / (norm * norm) -
               (1.0 + cos(norm)) / (2 * norm * sin(norm))) *
                  skewSymmetric(theta) * skewSymmetric(theta);
    }
    return jri;
}
} // namespace xrsfm

#endif // XRSFM_SRC_OPTIMIZATION_LIE_ALGEBRA_H
