#ifndef XRSFM_SRC_GEOMETRY_PROJECTION_HPP
#define XRSFM_SRC_GEOMETRY_PROJECTION_HPP

#include "base/camera.hpp"

namespace xrsfm {
inline double ReprojectionError(const Pose &pose, const Camera &camera,
                                const vector2 &point2d,
                                const vector3 &point3d) {
    vector3 p_c = pose.q * point3d + pose.t;
    if (p_c.z() < 0)
        return 100;
    vector2 estimate;
    NormalizedToImage(camera, p_c.hnormalized(), estimate);
    vector2 residual = estimate - point2d;
    return residual.norm();
}
} // namespace xrsfm

#endif // XRSFM_SRC_GEOMETRY_PROJECTION_HPP