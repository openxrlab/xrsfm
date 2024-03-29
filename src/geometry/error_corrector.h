#ifndef XRSFM_SRC_GEOMETRY_ERROR_CORRECTOR_H
#define XRSFM_SRC_GEOMETRY_ERROR_CORRECTOR_H

#include "base/map.h"
#include "geometry/track_processor.h"
#include "optimization/ba_solver.h"
// #include "utility/view.h"
// #include "utility/viewer.h"

namespace xrsfm {
inline double ToDeg(double theta) { return theta * 180 / M_PI; }

class ErrorDetector {
  public:
    bool CheckAllRelativePose(Map &map, int frame_id,
                              std::set<int> &bad_matched_frame_ids);
    bool IsGoodRelativePose(const Map &map, const FramePair &fp,
                            std::vector<char> &inlier_mask);

    std::string image_dir_;
};

class ErrorCorrector {
  public:
    ErrorCorrector(){};
    ErrorCorrector(BASolver *ba_ptr, Point3dProcessor *p3d_ptr)
        : ba_solver_(ba_ptr), p3d_processor_(p3d_ptr){};
    bool CheckAndCorrectPose(Map &map, int next_frame_id, int iter);

    bool only_correct_with_sim3_ = false;
    BASolver *ba_solver_;
    Point3dProcessor *p3d_processor_;
    ErrorDetector error_detector;
};
} // namespace xrsfm
#endif
