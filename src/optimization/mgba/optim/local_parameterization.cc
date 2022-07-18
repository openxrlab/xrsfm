#include "local_parameterization.h"

#include <algorithm>
#include "Eigen/Geometry"

namespace mgba {
LocalParameterization::~LocalParameterization() {}

bool LocalParameterization::MultiplyByJacobian(const double *x, const int num_rows, const double *global_matrix,
                                               double *local_matrix) const {
  if (LocalSize() == 0) {
    return true;
  }

  Eigen::MatrixXd jacobian(GlobalSize(), LocalSize());
  if (!ComputeJacobian(x, jacobian.data())) {
    return false;
  }

  Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(local_matrix, num_rows,
                                                                                     LocalSize()) =
      Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(global_matrix, num_rows,
                                                                                               GlobalSize()) *
      jacobian;
  return true;
}
}  // namespace mgba