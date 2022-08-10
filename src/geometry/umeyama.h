//
// Created by yzc on 2019/11/12.
//

#ifndef XRSFM_SRC_GEOMETRY_UMEYAMA_H
#define XRSFM_SRC_GEOMETRY_UMEYAMA_H

#include "utility/global.h"

namespace xrsfm {

struct SRT {
    double scale;
    quaternion q;
    vector3 t;
};

inline matrix3 kabsch(const std::vector<vector3> &src, const std::vector<vector3> &dst) {
    matrix3 cov = matrix3::Zero();
    for (size_t i = 0; i < src.size(); ++i) {
        cov += src[i] * dst[i].transpose();
    }
    cov = cov * (1.0 / src.size());
    Eigen::JacobiSVD<matrix3> svd(cov, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const matrix3 &U = svd.matrixU();
    const matrix3 &V = svd.matrixV();
    matrix3 E = matrix3::Identity();
    if ((V * U.transpose()).determinant() >= 0.0) {
        E(2, 2) = 1.0;
    } else {
        E(2, 2) = -1.0;
    }

    return V * E * U.transpose();
}

inline SRT umeyama(std::vector<vector3> gt, std::vector<vector3> in, bool fix_scale = false) {
    vector3 gt_avg = vector3::Zero();
    vector3 in_avg = vector3::Zero();
    for (size_t i = 0; i < gt.size(); ++i) {
        gt_avg += gt[i];
        in_avg += in[i];
    }
    gt_avg /= (double)gt.size();
    in_avg /= (double)in.size();

    double gt_d2 = 0;
    double in_d2 = 0;
    for (size_t i = 0; i < gt.size(); ++i) {
        gt[i] -= gt_avg;
        in[i] -= in_avg;
        gt_d2 += gt[i].squaredNorm();
        in_d2 += in[i].squaredNorm();
    }

    double S = sqrt(in_d2 / gt_d2);
    if (fix_scale) {
        S = 1;
    }
    matrix3 R = kabsch(in, gt);
    vector3 T = S * gt_avg - R * in_avg;

    quaternion q;
    q = R;

    SRT srt;
    srt.scale = S;
    srt.q = q;
    srt.t = T;

    return srt;
}

} // namespace xrsfm

#endif // XRSFM_SRC_GEOMETRY_UMEYAMA_H