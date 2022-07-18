//
// Created by SENSETIME\yezhichao1 on 2020/12/24.
//

#ifndef SIMPLEBA_TYPE_MG_H
#define SIMPLEBA_TYPE_MG_H

// #define EIGEN_NO_DEBUG

#include <Eigen/Eigen> 

namespace mgba {

using default_float = double;

template <typename T>
using map = Eigen::Map<T>;

template <typename T>
using const_map = Eigen::Map<const T>;

// If the storage order is not specified, then Eigen defaults to storing the entry in column-major.
template <int Rows = Eigen::Dynamic, int Cols = Rows, bool UseRowMajor = false, typename T = default_float>
using matrix = typename std::conditional<Rows != 1 && Cols != 1,
                                         Eigen::Matrix<T, Rows, Cols, UseRowMajor ? Eigen::RowMajor : Eigen::ColMajor>,
                                         Eigen::Matrix<T, Rows, Cols>>::type;

template <int Dimension = Eigen::Dynamic, bool RowVector = false, typename T = default_float>
using vector =
    typename std::conditional<RowVector, matrix<1, Dimension, false, T>, matrix<Dimension, 1, false, T>>::type;

using quaternion = Eigen::Quaternion<default_float>;

using matrix6 = matrix<6, 6>;
using matrix3 = matrix<3, 3>;
using matrix6x3 = matrix<6, 3>;
using matrixX = matrix<-1, -1>;

using vector6 = vector<6>;
using vector5 = vector<5>;
using vector4 = vector<4>;
using vector3 = vector<3>;
using vector2 = vector<2>;
using vectorX = vector<-1>;

}  // namespace mgba
#endif  // SIMPLEBA_TYPE_MG_H
