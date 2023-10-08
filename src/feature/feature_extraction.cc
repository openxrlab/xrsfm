//
// Created by yzc on 19-4-8.
//

#include <x86intrin.h>

#include <algorithm>
#include <numeric>

#include "base/map.h"
#include "feature_processing.h"
#include "geometry/colmap/estimators/fundamental_matrix.h"
#include "geometry/essential.h"
#include "optim/loransac.h"
#include "optim/ransac.h"
#include "sift_extractor.h"
#include "utility/io_ecim.hpp"
#include "utility/timer.h"

namespace xrsfm {

void FeatureExtract(const std::string &image_dir_path,
                    std::vector<Frame> &frames) {
#ifndef USE_ORB
    constexpr int _feature_num = 8192;
    // std::cout << "FeatureExtract:  feature number = " << _feature_num <<
    // "\n";
    SiftExtractor sift(_feature_num);
#else
    const int _feature_num = 2048;
    const float _lever_ratio = 1.2;
    const int _lever_num = 8;
    const int _initTh = 20;
    const int _minTh = 7;
    ORB_SLAM2::OrbExtractor orb(_feature_num, _lever_ratio, _lever_num, _initTh,
                                _minTh);
#endif
    for (int i = 0; i < frames.size(); i++) {
        Frame &frame = frames[i];
        frame.keypoints_.clear();
        const cv::Mat image = cv::imread(image_dir_path + frame.name);
        if (image.rows == 0) {
            std::cout << "Can't read " << image_dir_path + frame.name
                      << std::endl;
            exit(0);
        }
#ifndef USE_ORB
        // std::cout << image_dir_path + frame.name << std::endl;
        sift.ExtractUINT8(image, frame.keypoints_, frame.uint_descs_);
        // std::cout << i << " " << frame.uint_descs_.rows() << std::endl;
#else
        orb(image, cv::Mat(), frame.keypoints_, frame.orb_descs_);
#endif
    }
    SetUpFramePoints(frames);
}

} // namespace xrsfm
