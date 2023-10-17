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
        params_.resize(camera_model_param_size(model_id_));
        is_valid = true;
    }
    Camera(int _id, double fx, double fy, double cx, double cy, double d) {
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
        const int id_distort = index_distort(model_id_);
        if (id_distort == -1)
            return nullptr;
        return params_.data() + id_distort;
    }

    inline bool valid() { return is_valid; }

    inline void set_invalid() { is_valid = false; }

    inline void log() {
        const int id_distort = index_distort(model_id_);
        const double distrot_param = id_distort == -1 ? 0 : params_[id_distort];
        printf("%d %lf %lf %lf %lf %lf\n", id_, fx(), fy(), cx(), cy(),
               distrot_param);
    }
};

inline void ImageToNormalized(const Camera &c, const Eigen::Vector2d &p2d,
                              Eigen::Vector2d &p2d_n) {
#define CAMERA_MODEL_CASE(CameraModel)                                         \
    if (c.model_id_ == CameraModel::kModelId) {                                \
        CameraModel::ImageToWorld(c.params_.data(), p2d.data(), p2d_n.data()); \
        return;                                                                \
    }

    CAMERA_MODEL_CASES
#undef CAMERA_MODEL_CASE

    std::cout << "ERROR: (ImageToNormalized) unsupported model type: "
              << c.model_id_ << "\n";
    exit(-1);
}

inline void NormalizedToImage(const Camera &c, const Eigen::Vector2d &p2d_n,
                              Eigen::Vector2d &p2d) {
#define CAMERA_MODEL_CASE(CameraModel)                                         \
    if (c.model_id_ == CameraModel::kModelId) {                                \
        CameraModel::WorldToImage(c.params_.data(), p2d_n.data(), p2d.data()); \
        return;                                                                \
    }

    CAMERA_MODEL_CASES
#undef CAMERA_MODEL_CASE

    std::cout << "ERROR: (NormalizedToImage) unsupported model type: "
              << c.model_id_ << "\n";
    exit(-1);
}

inline Eigen::Vector2d GetPointNormalized(const Camera &c,
                                          const Eigen::Vector2d &p2d) {
    Eigen::Vector2d p2d_n;
    ImageToNormalized(c, p2d, p2d_n);
    return p2d_n;
}

} // namespace xrsfm
#endif
