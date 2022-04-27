//
// Created by yzc on 19-4-8.
//

#ifndef WSFM_FEATURE_PROCESSING_H
#define WSFM_FEATURE_PROCESSING_H

#include <SiftGPU.h>

#include <vector>

#include "base/map.h"
#include "base/types.h"

bool CreateSiftGPUMatcher(SiftMatchGPU *sift_match_gpu);

void SiftMatch(const FeatureDescriptors &descs1, const FeatureDescriptors &descs2, std::vector<Match> &matches,
               SiftMatchGPU *sift_match_gpu, float max_ratio = 0.9);

void SiftMatch(const UINT8Descriptors &descs1, const UINT8Descriptors &descs2, std::vector<Match> &matches,
               SiftMatchGPU *sift_match_gpu, float max_ratio = 0.9);

void OrbMatch(const cv::Mat &descs1, const cv::Mat &descs2, std::vector<Match> &matches);

void FeatureExtract(const std::string &dir_path, const Camera &camera, const std::vector<cv::Mat> &images,
                    std::vector<Frame> &frames);

void FeatureMatching(const std::vector<Frame> &frames, std::vector<FramePair> &frame_pairs);

void FeatureMatchingUnorder(const std::vector<Frame> &frames, std::vector<FramePair> &frame_pairs);

#endif  // WSFM_FEATURE_PROCESSING_H
