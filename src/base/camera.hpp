#ifndef XRSFM_SRC_BASE_CAMERA_HPP
#define XRSFM_SRC_BASE_CAMERA_HPP

#include <Eigen/Eigen>

#include "camera_model.hpp"

namespace xrsfm {

class Camera {
  public:
    Camera() {}
    Camera(int _id, int _model_id) {
        id_ = _id;
        model_id_ = _model_id;
        is_valid = true;
    }
    Camera(int _id, double fx = 0, double fy = 0, double cx = 0, double cy = 0,
           double d = 0) {
        id_ = _id;
        model_id_ = 2;
        params_ = {fx, cx, cy, d};
        std::cout << params_.size() << std::endl;
        is_valid = true;
    }

    uint32_t id_ = -1;
    uint32_t model_id_ = -1;
    std::vector<double> params_;

    bool is_valid;

    inline const double fx() const { return params_[index_fx(model_id_)]; }

    inline const double fy() const { return params_[index_fy(model_id_)]; }

    inline const double cx() const { return params_[index_cx(model_id_)]; }

    inline const double cy() const { return params_[index_cy(model_id_)]; }

    inline double *distort_params() {
        return params_.data() + index_distort(model_id_);
    }

    inline bool valid() { return is_valid; }

    inline void set_invalid() { is_valid = false; }

    inline void log() {
        printf("%d %lf %lf %lf %lf %lf\n", id_, fx(), fy(), cx(), cy(),
               distort_params()[0]);
    }
};

template <typename CameraModel>
inline void IterativeUndistortion(const double *params, Eigen::Vector2d &x) {
    // Parameters for Newton iteration using numerical differentiation with
    // central differences, 100 iterations should be enough even for complex
    // camera models with higher order terms.
    const size_t kNumIterations = 100;
    const double kMaxStepNorm = 1e-10;
    const double kRelStepSize = 1e-6;

    Eigen::Matrix2d J;
    const Eigen::Vector2d x0 = x;
    Eigen::Vector2d dx;
    Eigen::Vector2d dx_0b;
    Eigen::Vector2d dx_0f;
    Eigen::Vector2d dx_1b;
    Eigen::Vector2d dx_1f;
    Eigen::Vector2d xy_plus;

    for (size_t i = 0; i < kNumIterations; ++i) {
        // std::cout << i << " " << kNumIterations << std::endl;
        const double step0 = std::max(std::numeric_limits<double>::epsilon(),
                                      std::abs(kRelStepSize * x(0)));
        const double step1 = std::max(std::numeric_limits<double>::epsilon(),
                                      std::abs(kRelStepSize * x(1)));

        CameraModel::Distortion(params, x.data(), dx.data());
        xy_plus << x(0) - step0, x(1);
        CameraModel::Distortion(params, xy_plus.data(), dx_0b.data());
        xy_plus << x(0) + step0, x(1);
        CameraModel::Distortion(params, xy_plus.data(), dx_0f.data());
        xy_plus << x(0), x(1) - step1;
        CameraModel::Distortion(params, xy_plus.data(), dx_1b.data());
        xy_plus << x(0), x(1) + step1;
        CameraModel::Distortion(params, xy_plus.data(), dx_1f.data());

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

    // *u = x(0);
    // *v = x(1);
}

inline void ImageToNormalized(const Camera &c, const Eigen::Vector2d &p2d,
                              Eigen::Vector2d &p2d_n) {
    double u = (p2d.x() - c.cx()) / c.fx();
    double v = (p2d.y() - c.cy()) / c.fy();
    Eigen::Vector2d xy(u, v);

    if (c.model_id_ == 2) {
        IterativeUndistortion<SimpleRadialCameraModel>(c.distort_params(), xy);
    } else if (c.model_id_ == 4) {
        IterativeUndistortion<OpenCVCameraModel>(c.distort_params(), xy);
    } else {
        std::cout << "ERROR: (ImageToNormalized) unsupport camera_id: "
                  << c.model_id_ << "\n";
        exit(-1);
    }

    p2d_n << xy(0), xy(1);
}

inline void NormalizedToImage(const Camera &c, const Eigen::Vector2d &p2d_n,
                              Eigen::Vector2d &p2d) {
    if (c.model_id_ == 2) {
        SimpleRadialCameraModel::WorldToImage(c.params_.data(), p2d_n.data(),
                                              p2d.data());
    } else if (c.model_id_ == 4) {
        OpenCVCameraModel::WorldToImage(c.params_.data(), p2d_n.data(),
                                        p2d.data());
    } else {
        std::cout << "ERROR: (NormalizedToImage) unsupport camera_id: "
                  << c.model_id_ << "\n";
        exit(-1);
    }
}

inline Eigen::Vector2d GetPointNormalized(const Camera &c,
                                          const Eigen::Vector2d &p2d) {
    Eigen::Vector2d p2d_n;
    ImageToNormalized(c, p2d, p2d_n);
    return p2d_n;
}

} // namespace xrsfm
#endif
