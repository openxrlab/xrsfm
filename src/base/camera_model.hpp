#ifndef XRSFM_SRC_BASE_CAMERA_MODEL_HPP
#define XRSFM_SRC_BASE_CAMERA_MODEL_HPP

#include <Eigen/Eigen>

namespace xrsfm {

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
    }

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

    DefineFunctionWorldToImage
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

    DefineFunctionWorldToImage
};

#ifndef CAMERA_MODEL_CASES
#define CAMERA_MODEL_CASES                                                     \
    CAMERA_MODEL_CASE(SimpleRadialCameraModel)       \ 
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

} // namespace xrsfm
#endif