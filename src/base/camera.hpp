#ifndef CAMERA_H
#define CAMERA_H

#include <Eigen/Eigen>
// #include "xrprimer/data_structure/camera/camera.h"

namespace xrsfm{
// class Camera : public BaseCameraParameter {
class Camera {
 public:
  explicit Camera(int _id = 0, double fx = 0, double fy = 0, double cx = 0, double cy = 0, double d = 0) {
    id = _id;
    camera_model = CameraModel::SIMPLE_RADIAL;
    camera_params = {fx, fy, cx, cy};
    distort_params = {d, 0, 0, 0, 0};
  }

  enum CameraModel { OpenCV, SIMPLE_RADIAL } camera_model;

  uint32_t id = -1;
  // double fx, fy, cx, cy;
  std::array<double, 4> camera_params;
  // k1 k2 p1 p2 k3
  std::array<double, 5> distort_params;

  inline const double fx() const { return camera_params[0]; }

  inline const double fy() const { return camera_params[1]; }

  inline const double cx() const { return camera_params[2]; }

  inline const double cy() const { return camera_params[3]; }

  inline void log() { printf("%d %lf %lf %lf %lf %lf\n", id, fx(), fy(), cx(), cy(), distort_params[0]); }
};

template <typename T>
inline void Distortion(const T *extra_params, const T u, const T v, T *du, T *dv) {
  const T k = extra_params[0];

  const T u2 = u * u;
  const T v2 = v * v;
  const T r2 = u2 + v2;
  const T radial = k * r2;
  *du = u * radial;
  *dv = v * radial;
}

inline void IterativeUndistortion(const double *params, double *u, double *v) {
  // Parameters for Newton iteration using numerical differentiation with
  // central differences, 100 iterations should be enough even for complex
  // camera models with higher order terms.
  const size_t kNumIterations = 100;
  const double kMaxStepNorm = 1e-10;
  const double kRelStepSize = 1e-6;

  Eigen::Matrix2d J;
  const Eigen::Vector2d x0(*u, *v);
  Eigen::Vector2d x(*u, *v);
  Eigen::Vector2d dx;
  Eigen::Vector2d dx_0b;
  Eigen::Vector2d dx_0f;
  Eigen::Vector2d dx_1b;
  Eigen::Vector2d dx_1f;

  for (size_t i = 0; i < kNumIterations; ++i) {
    const double step0 = std::max(std::numeric_limits<double>::epsilon(), std::abs(kRelStepSize * x(0)));
    const double step1 = std::max(std::numeric_limits<double>::epsilon(), std::abs(kRelStepSize * x(1)));
    Distortion(params, x(0), x(1), &dx(0), &dx(1));
    Distortion(params, x(0) - step0, x(1), &dx_0b(0), &dx_0b(1));
    Distortion(params, x(0) + step0, x(1), &dx_0f(0), &dx_0f(1));
    Distortion(params, x(0), x(1) - step1, &dx_1b(0), &dx_1b(1));
    Distortion(params, x(0), x(1) + step1, &dx_1f(0), &dx_1f(1));
    J(0, 0) = 1 + (dx_0f(0) - dx_0b(0)) / (2 * step0);
    J(0, 1) = (dx_1f(0) - dx_1b(0)) / (2 * step1);
    J(1, 0) = (dx_0f(1) - dx_0b(1)) / (2 * step0);
    J(1, 1) = 1 + (dx_1f(1) - dx_1b(1)) / (2 * step1);
    const Eigen::Vector2d step_x = J.inverse() * (x + dx - x0);
    x -= step_x;
    if (step_x.squaredNorm() < kMaxStepNorm) {
      break;
    }
  }

  *u = x(0);
  *v = x(1);
}

inline void ImageToNormalized(const Camera &c, const Eigen::Vector2d &p2d, Eigen::Vector2d &p2d_n) {
  double u = (p2d.x() - c.cx()) / c.fx();
  double v = (p2d.y() - c.cy()) / c.fy();
  // std::cout << u << " " << v << std::endl;
  IterativeUndistortion(c.distort_params.data(), &u, &v);  // bug
  // std::cout << u << " " << v << std::endl;
  p2d_n << u, v;
}

inline void NormalizedToImage(const Camera &c, const Eigen::Vector2d &p2d_n, Eigen::Vector2d &p2d) {
  Eigen::Vector2d p2d_n_d;
  Distortion(c.distort_params.data(), p2d_n.x(), p2d_n.y(), &p2d_n_d.x(), &p2d_n_d.y());
  p2d_n_d = p2d_n + p2d_n_d;
  double u = c.fx() * p2d_n_d.x() + c.cx();
  double v = c.fy() * p2d_n_d.y() + c.cy();
  p2d << u, v;
}
}
#endif