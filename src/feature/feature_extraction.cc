//
// Created by yzc on 19-4-8.
//

#include <x86intrin.h>

#include <algorithm>
#include <numeric>

#include "base/map.h"
#include "feature_processing.h"
#include "estimators/fundamental_matrix.h"
#include "geometry/essential.h"
#include "ransac/loransac.h"
#include "ransac/ransac.h"
#include "sift_extractor.h"
#include "utility/timer.h"

namespace xrsfm {

void FeatureExtract(const std::string &image_dir_path,
                    std::vector<Frame> &frames,
                    std::vector<ImageSize> &image_size) {
#ifndef USE_ORB
    constexpr int _feature_num = 8192;
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
        const cv::Mat image = cv::imread(image_dir_path + frame.name,
                                         cv::IMREAD_IGNORE_ORIENTATION);
        if (image.rows == 0) {
            std::cout << "ERROR: fail to read image: "
                      << image_dir_path + frame.name << std::endl;
            exit(0);
        }

        image_size.push_back(ImageSize(image.cols, image.rows));

#ifndef USE_ORB
        sift.ExtractUINT8(image, frame.keypoints_, frame.uint_descs_);
#else
        orb(image, cv::Mat(), frame.keypoints_, frame.orb_descs_);
#endif
    }
    SetUpFramePoints(frames);
}

} // namespace xrsfm
