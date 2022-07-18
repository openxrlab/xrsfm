#ifndef LOCAL_PARAMETERIZATION_H_
#define LOCAL_PARAMETERIZATION_H_

#include "../type_mg.h"
#include "../matrix_math.h"

namespace mgba {

class LocalParameterization {
 public:
  virtual ~LocalParameterization();

  virtual bool Plus(const default_float *x, const default_float *delta, default_float *x_plus_delta) const = 0;

  virtual bool ComputeJacobian(const double *x, double *jacobian) const = 0;

  virtual bool MultiplyByJacobian(const double *x, const int num_rows, const double *global_matrix,
                                  double *local_matrix) const;

  // Size of x.
  virtual int GlobalSize() const = 0;

  // Size of delta.
  virtual int LocalSize() const = 0;
};

class EQuatParam : public LocalParameterization {
 public:
  bool Plus(const default_float *x, const default_float *delta, default_float *x_plus_delta) const override {
    const_map<quaternion> _q(x);
    const_map<vector3> theta(delta);
    map<quaternion> q(x_plus_delta);
    quaternion dq = expmap(theta);
    q = (_q * dq).normalized();
    return true;
  }

  bool ComputeJacobian(const double *x, double *jacobian) const override {
    Eigen::Map<Eigen::Matrix<double, 4, 3, Eigen::RowMajor>> j(jacobian);
    j.setIdentity();
    return true;
  }

  int GlobalSize() const override { return 4; };

  int LocalSize() const override { return 3; };
};

}  // namespace mgba
#endif