#ifndef XRSFM_SRC_BASE_CAMERA_MODEL_HPP
#define XRSFM_SRC_BASE_CAMERA_MODEL_HPP

#include <Eigen/Eigen>

namespace xrsfm {

template <typename CameraModel>
inline void IterativeUndistortion(const double *params, double *xy) {
    // Parameters for Newton iteration using numerical differentiation
    // with central differences, 100 iterations should be enough even for
    // complex camera models with higher order terms.
    const size_t kNumIterations = 100;
    const double kMaxStepNorm = 1e-10;
    const double kRelStepSize = 1e-6;

    Eigen::Map<Eigen::Vector2d> x(xy);

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
}

#define DefineFunctionWorldToImage                                             \
    template <typename T>                                                      \
    static void WorldToImage(const T *params, const T *xy, T *uv) {            \
        const T fx = params[kIndexFx];                                         \
        const T fy = params[kIndexFy];                                         \
        const T cx = params[kIndexCx];                                         \
        const T cy = params[kIndexCy];                                         \
        T duv[2];                                                              \
        Distortion(params + kIndexDistort, xy, duv);                           \
        uv[0] = fx * (xy[0] + duv[0]) + cx;                                    \
        uv[1] = fy * (xy[1] + duv[1]) + cy;                                    \
    };

#define DefineFunctionImageToWorld(camera_model)                               \
    template <typename T>                                                      \
    static void ImageToWorld(const T *params, const T *uv, T *xy) {            \
        const T fx = params[kIndexFx];                                         \
        const T fy = params[kIndexFy];                                         \
        const T cx = params[kIndexCx];                                         \
        const T cy = params[kIndexCy];                                         \
        xy[0] = (uv[0] - cx) / fx;                                             \
        xy[1] = (uv[1] - cy) / fy;                                             \
        IterativeUndistortion<camera_model>(params + kIndexDistort, xy);       \
    };

#define DefineFunctionImageToWorld1                                            \
    template <typename T>                                                      \
    static void ImageToWorld(const T *params, const T *uv, T *xy) {            \
        const T fx = params[kIndexFx];                                         \
        const T fy = params[kIndexFy];                                         \
        const T cx = params[kIndexCx];                                         \
        const T cy = params[kIndexCy];                                         \
        xy[0] = (uv[0] - cx) / fx;                                             \
        xy[1] = (uv[1] - cy) / fy;                                             \
    };

struct SimplePinholeCameraModel {
    static constexpr int kModelId = 0;
    static constexpr int kNumParams = 3;
    static constexpr int kIndexFx = 0;
    static constexpr int kIndexFy = 0;
    static constexpr int kIndexCx = 1;
    static constexpr int kIndexCy = 2;
    static constexpr int kIndexDistort = -1;
    template <typename T>
    static void Distortion(const T *extra_params, const T *xy, T *duv) {
        duv[0] = xy[0];
        duv[1] = xy[1];
    };

    DefineFunctionWorldToImage;

    DefineFunctionImageToWorld1;
};

struct PinholeCameraModel {
    static constexpr int kModelId = 1;
    static constexpr int kNumParams = 4;
    static constexpr int kIndexFx = 0;
    static constexpr int kIndexFy = 1;
    static constexpr int kIndexCx = 2;
    static constexpr int kIndexCy = 3;
    static constexpr int kIndexDistort = -1;
    template <typename T>
    static void Distortion(const T *extra_params, const T *xy, T *duv) {
        duv[0] = xy[0];
        duv[1] = xy[1];
    }

    DefineFunctionWorldToImage;

    DefineFunctionImageToWorld1;
};

struct SimpleRadialCameraModel {
    static constexpr int kModelId = 2;
    static constexpr int kNumParams = 4;
    static constexpr int kIndexFx = 0;
    static constexpr int kIndexFy = 0;
    static constexpr int kIndexCx = 1;
    static constexpr int kIndexCy = 2;
    static constexpr int kIndexDistort = 3;
    template <typename T>
    static void Distortion(const T *extra_params, const T *xy, T *duv) {
        const T k = extra_params[0];
        const T u2 = xy[0] * xy[0];
        const T v2 = xy[1] * xy[1];
        const T r2 = u2 + v2;
        const T radial = k * r2;
        duv[0] = xy[0] * radial;
        duv[1] = xy[1] * radial;
    }

    DefineFunctionWorldToImage;

    DefineFunctionImageToWorld(SimpleRadialCameraModel);
};

struct RadialCameraModel {
    static constexpr int kModelId = 3;
    static constexpr int kNumParams = 5;
    static constexpr int kIndexFx = 0;
    static constexpr int kIndexFy = 1;
    static constexpr int kIndexCx = 2;
    static constexpr int kIndexCy = 3;
    static constexpr int kIndexDistort = 4;
    template <typename T>
    static void Distortion(const T *extra_params, const T *xy, T *duv) {
        const T k = extra_params[0];
        const T u2 = xy[0] * xy[0];
        const T v2 = xy[1] * xy[1];
        const T r2 = u2 + v2;
        const T radial = k * r2;
        duv[0] = xy[0] * radial;
        duv[1] = xy[1] * radial;
    }

    DefineFunctionWorldToImage;

    DefineFunctionImageToWorld(RadialCameraModel);
};

struct OpenCVCameraModel {
    static constexpr int kModelId = 4;
    static constexpr int kNumParams = 8;
    static constexpr int kIndexFx = 0;
    static constexpr int kIndexFy = 1;
    static constexpr int kIndexCx = 2;
    static constexpr int kIndexCy = 3;
    static constexpr int kIndexDistort = 4;
    template <typename T>
    static void Distortion(const T *extra_params, const T *_xy, T *duv) {
        const T k1 = extra_params[0];
        const T k2 = extra_params[1];
        const T p1 = extra_params[2];
        const T p2 = extra_params[3];

        const T x = _xy[0];
        const T y = _xy[1];

        const T x2 = x * x;
        const T xy = x * y;
        const T y2 = y * y;
        const T r2 = x2 + y2;
        const T radial = k1 * r2 + k2 * r2 * r2;
        duv[0] = x * radial + T(2) * p1 * xy + p2 * (r2 + T(2) * x2);
        duv[1] = y * radial + T(2) * p2 * xy + p1 * (r2 + T(2) * y2);
    }

    DefineFunctionWorldToImage;

    DefineFunctionImageToWorld(OpenCVCameraModel);
};

#ifndef CAMERA_MODEL_CASES

#define CAMERA_MODEL_CASES                                                     \
    CAMERA_MODEL_CASE(SimplePinholeCameraModel)                                \
    CAMERA_MODEL_CASE(PinholeCameraModel)                                      \
    CAMERA_MODEL_CASE(SimpleRadialCameraModel)                                 \
    CAMERA_MODEL_CASE(RadialCameraModel)                                       \
    CAMERA_MODEL_CASE(OpenCVCameraModel)

#endif

inline int index_fx(int model_id) {
#define CAMERA_MODEL_CASE(CameraModel)                                         \
    if (model_id == CameraModel::kModelId)                                     \
        return CameraModel::kIndexFx;

    CAMERA_MODEL_CASES
#undef CAMERA_MODEL_CASE
    assert(false);
    return -1;
};

inline int index_fy(int model_id) {
#define CAMERA_MODEL_CASE(CameraModel)                                         \
    if (model_id == CameraModel::kModelId)                                     \
        return CameraModel::kIndexFy;

    CAMERA_MODEL_CASES
#undef CAMERA_MODEL_CASE
    assert(false);
    return -1;
};

inline int index_cx(int model_id) {
#define CAMERA_MODEL_CASE(CameraModel)                                         \
    if (model_id == CameraModel::kModelId)                                     \
        return CameraModel::kIndexCx;

    CAMERA_MODEL_CASES
#undef CAMERA_MODEL_CASE
    assert(false);
    return -1;
};

inline int index_cy(int model_id) {
#define CAMERA_MODEL_CASE(CameraModel)                                         \
    if (model_id == CameraModel::kModelId)                                     \
        return CameraModel::kIndexCy;

    CAMERA_MODEL_CASES
#undef CAMERA_MODEL_CASE
    assert(false);
    return -1;
};

inline int index_distort(int model_id) {
#define CAMERA_MODEL_CASE(CameraModel)                                         \
    if (model_id == CameraModel::kModelId)                                     \
        return CameraModel::kIndexDistort;

    CAMERA_MODEL_CASES
#undef CAMERA_MODEL_CASE
    assert(false);
    return -1;
};

inline int camera_model_param_size(int model_id) {
#define CAMERA_MODEL_CASE(CameraModel)                                         \
    if (model_id == CameraModel::kModelId)                                     \
        return CameraModel::kNumParams;

    CAMERA_MODEL_CASES
#undef CAMERA_MODEL_CASE
    assert(false);
    return -1;
}

} // namespace xrsfm
#endif
