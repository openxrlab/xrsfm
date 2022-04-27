#ifndef TYPES_H
#define TYPES_H

#include <Eigen/Eigen>

typedef Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> UINT8Descriptors;
typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> FeatureDescriptors;

constexpr int MIN_OBS_NUM_LEVEL = 20;

struct Match {
  Match(int _id1 = 0, int _id2 = 0, double _dist = 0) : id1(_id1), id2(_id2), distance(_dist) {}

  int id1;
  int id2;
  double distance;
};

#endif  // TYPES_H