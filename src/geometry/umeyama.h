//
// Created by yzc on 2019/11/12.
//

#ifndef WARG_UMYAMA_H
#define WARG_UMYAMA_H

namespace xrsfm {

using quaternion = Eigen::Quaternion<double>;

template <int Rows = Eigen::Dynamic, int Cols = Rows, bool UseRowMajor = false, typename T = double>
using matrix = typename std::conditional<Rows != 1 && Cols != 1,
                                         Eigen::Matrix<T, Rows, Cols, UseRowMajor ? Eigen::RowMajor : Eigen::ColMajor>,
                                         Eigen::Matrix<T, Rows, Cols>>::type;

template <int Dimension = Eigen::Dynamic, bool RowVector = false, typename T = double>
using vector =
    typename std::conditional<RowVector, matrix<1, Dimension, false, T>, matrix<Dimension, 1, false, T>>::type;

struct SRT {
    double scale;
    quaternion q;
    vector<3> t;
};

inline matrix<3> kabsch(const std::vector<vector<3>> &src, const std::vector<vector<3>> &dst) {
    matrix<3> cov = matrix<3>::Zero();
    for (size_t i = 0; i < src.size(); ++i) {
        cov += src[i] * dst[i].transpose();
    }
    cov = cov * (1.0 / src.size());
    Eigen::JacobiSVD<matrix<3>> svd(cov, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const matrix<3> &U = svd.matrixU();
    const matrix<3> &V = svd.matrixV();
    matrix<3> E = matrix<3>::Identity();
    if ((V * U.transpose()).determinant() >= 0.0) {
        E(2, 2) = 1.0;
    } else {
        E(2, 2) = -1.0;
    }

    return V * E * U.transpose();
}

inline SRT umeyama(std::vector<vector<3>> gt, std::vector<vector<3>> in, bool fix_scale = false) {
    vector<3> gt_avg = vector<3>::Zero();
    vector<3> in_avg = vector<3>::Zero();
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
    matrix<3> R = kabsch(in, gt);
    vector<3> T = S * gt_avg - R * in_avg;

    quaternion q;
    q = R;

    SRT srt;
    srt.scale = S;
    srt.q = q;
    srt.t = T;

    return srt;
}
} // namespace xrsfm

#endif // WARG_UMYAMA_H