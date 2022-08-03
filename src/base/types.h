#ifndef TYPES_H
#define TYPES_H

#include <Eigen/Eigen>

typedef Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> UINT8Descriptors;
typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> FeatureDescriptors;

struct Match {
  Match(int _id1 = 0, int _id2 = 0, double _dist = 0) : id1(_id1), id2(_id2), distance(_dist) {}

  int id1;
  int id2;
  double distance;
};

struct ImageSize {
  int width, height;
  ImageSize() {}
  ImageSize(int w, int h) {
    width = w;
    height = h;
  }
};

struct Pose {
  explicit Pose(Eigen::Quaterniond _q = Eigen::Quaterniond::Identity(), Eigen::Vector3d _t = Eigen::Vector3d::Zero())
      : q(_q), t(_t){};
  Pose(Eigen::Matrix4d T){
    Eigen::Matrix3d R = T.block<3,3>(0,0);
    q = Eigen::Quaterniond(R);
    q = q.normalized();
    t = T.col(3).head<3>();
  }
  Eigen::Quaterniond q;
  Eigen::Vector3d t;

  inline Eigen::Vector3d center() { return -(q.inverse() * t); }

  inline Pose inverse() {
    Pose p;
    p.t = -(q.inverse() * t);
    p.q = q.inverse();
    return p;
  }

  inline Pose mul(Pose p) {
    p.t = (q * p.t) + t;
    p.q = q * p.q;
    return p;
  }
   
  inline Pose scale(double s) { return Pose(q, s * t); }
};

#endif  // TYPES_H