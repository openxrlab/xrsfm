//
// Created by SENSETIME\yezhichao1 on 2020/9/27.
//

#ifndef RGBD_TEST_LIE_ALGEBRA_H
#define RGBD_TEST_LIE_ALGEBRA_H

inline Eigen::Vector3d logmap(const Eigen::Quaterniond &q) {
  Eigen::AngleAxisd aa(q);
  return aa.angle() * aa.axis();
}

inline Eigen::Quaterniond expmap(const Eigen::Vector3d &w) {
  Eigen::AngleAxisd aa(w.norm(), w.stableNormalized());
  return Eigen::Quaterniond(aa);
}

inline Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d &q) {
  Eigen::Matrix3d ans;
  ans << 0, -q(2), q(1), q(2), 0, -q(0), -q(1), q(0), 0;
  return ans;
}
inline Eigen::Matrix3d jr(Eigen::Vector3d theta) {
  double norm = theta.norm();
  Eigen::Matrix3d jr;
  if (norm < 1.745329252e-7) {
    jr = Eigen::Matrix3d::Identity() - 1.0 / 2.0 * skewSymmetric(theta) +
         1.0 / 6.0 * skewSymmetric(theta) * skewSymmetric(theta);
  } else {
    jr = Eigen::Matrix3d::Identity() - (1.0 - cos(norm)) / (norm * norm) * skewSymmetric(theta) +
         (norm - sin(norm)) / (norm * norm * norm) * skewSymmetric(theta) * skewSymmetric(theta);
  }
  return jr;
}

inline Eigen::Matrix3d jri(Eigen::Vector3d theta) {
  double norm = theta.norm();
  Eigen::Matrix3d jri;
  if (norm < 1.745329252e-7) {
    jri = Eigen::Matrix3d::Identity() + 1.0 / 2.0 * skewSymmetric(theta) +
          1.0 / 12.0 * skewSymmetric(theta) * skewSymmetric(theta);
  } else {
    jri = Eigen::Matrix3d::Identity() + 1.0 / 2.0 * skewSymmetric(theta) +
          ((1.0) / (norm * norm) - (1.0 + cos(norm)) / (2 * norm * sin(norm))) * skewSymmetric(theta) *
              skewSymmetric(theta);
  }
  return jri;
}

#endif  // RGBD_TEST_LIE_ALGEBRA_H
