#ifndef XRSFM_SRC_GEOMETRY_ERROR_CORRECTOR_H
#define XRSFM_SRC_GEOMETRY_ERROR_CORRECTOR_H

#include "geometry/track_processor.h"
#include "base/map.h"
#include "optimization/ba_solver.h"
#include "utility/view.h"
#include "utility/viewer.h"

namespace xrsfm {
inline double ToDeg(double theta) { return theta * 180 / M_PI; }

class ErrorDetector {
  public:
    Init(std::string image_dir, bool debug) {
        debug_ = debug;
        image_dir_ = image_dir;
    };

    bool CheckAllRelativePose(Map &map, int frame_id,
                              std::set<int> &bad_matched_frame_ids);
    bool IsGoodRelativePose(const Map &map, const FramePair &fp,
                            std::vector<char> &inlier_mask);
    void IsGoodRelativePose_Debug(Map &map, FramePair &fp,
                                  const std::vector<char> &inlier_mask);
    void StoreRelativePose(Map &map, int frame_id, std::ofstream &file);

    bool debug_;
    ViewerThread *viewerTh_;
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
