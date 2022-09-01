//
// Created by yzc on 19-4-8.
//

#ifndef XRSFM_SRC_FEATURE_FEATURE_PROCESSING_H
#define XRSFM_SRC_FEATURE_FEATURE_PROCESSING_H

#include <vector>

#include <SiftGPU.h>

#include "base/types.h"
#include "base/map.h"

// #define USE_ORB

namespace xrsfm {

void SetUpFramePoints(std::vector<Frame> &frames);

bool CreateSiftGPUMatcher(SiftMatchGPU *sift_match_gpu);

void SiftMatch(const FeatureDescriptors &descs1, const FeatureDescriptors &descs2,
               std::vector<Match> &matches, SiftMatchGPU *sift_match_gpu, float max_ratio = 0.9);

void SiftMatch(const UINT8Descriptors &descs1, const UINT8Descriptors &descs2,
               std::vector<Match> &matches, SiftMatchGPU *sift_match_gpu, float max_ratio = 0.9);

void OrbMatch(const cv::Mat &descs1, const cv::Mat &descs2, std::vector<Match> &matches);

void FeatureExtract(const std::string &image_dir_path, std::vector<Frame> &frames);

void FeatureMatching(const std::vector<Frame> &frames,
                     const std::vector<std::pair<int, int>> &candidate_pairs,
                     std::vector<FramePair> &frame_pairs,
                     bool b_use_fundamental = false); // DONE(BCHO)

std::vector<Match> ExtractInlierMatches(const std::vector<Match> &matches, const size_t num_inliers,
                                        const std::vector<char> &inlier_mask);

void ExpansionAndMatching(Map &map, const std::map<int, std::vector<int>> &retrieval_rank_of_frames,
                          const int num_iteration, const std::vector<ImageSize> &images,
                          const size_t &ini_frame1, const size_t &ini_frame2,
                          bool b_use_fundamental,
                          std::vector<std::pair<int, int>> &init_candidates);

} // namespace xrsfm
#endif // XRSFM_BASE_FEATURE_PROCESSING_H
