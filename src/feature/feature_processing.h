//
// Created by yzc on 19-4-8.
//

#ifndef WSFM_FEATURE_PROCESSING_H
#define WSFM_FEATURE_PROCESSING_H

#include <SiftGPU.h>

#include <vector>

#include "base/types.h"
#include "base/map.h"
#include "match_expansion.h"

// #define USE_ORB

namespace xrsfm{
bool CreateSiftGPUMatcher(SiftMatchGPU *sift_match_gpu);

void SiftMatch(const FeatureDescriptors &descs1, const FeatureDescriptors &descs2,
               std::vector<Match> &matches, SiftMatchGPU *sift_match_gpu, float max_ratio = 0.9);

void SiftMatch(const UINT8Descriptors &descs1, const UINT8Descriptors &descs2,
               std::vector<Match> &matches, SiftMatchGPU *sift_match_gpu, float max_ratio = 0.9);

void OrbMatch(const cv::Mat &descs1, const cv::Mat &descs2, std::vector<Match> &matches);

void FeatureExtract(const std::vector<cv::Mat> &images, std::vector<Frame> &frames);

void FeatureExtract(const std::string &image_dir_path, std::vector<Frame> &frames);

void FeatureExtract(const std::string &dir_path, const Camera &camera,
                    const std::vector<cv::Mat> &images, std::vector<Frame> &frames);

void FeatureExtractMixData(const std::vector<Camera> &cameras,
                           std::map<int, int> &seqindex2cameraindex,
                           const std::vector<cv::Mat> &images, std::vector<Frame> &frames);

void FeatureMatchingSeq(const std::vector<Frame> &frames, std::vector<FramePair> &frame_pairs);

void FeatureMatching(const std::vector<Frame> &frames,
                     const std::vector<std::pair<int, int>> &candidate_pairs,
                     std::vector<FramePair> &frame_pairs,
                     bool b_use_fundamental = false);  // DONE(BCHO)

void FeatureMatchingUnorder(const std::vector<Frame> &frames, std::vector<FramePair> &frame_pairs);

void GeometryVerification(const std::vector<Frame> &frames, std::vector<FramePair> &frame_pairs);

std::vector<Match> ExtractInlierMatches(const std::vector<Match> &matches, const size_t num_inliers,
                                        const std::vector<char> &inlier_mask);

void ExpansionMatching(Map &map, const int num_iteration, const std::string &nvall_path,
                       std::map<std::string, int> &name2frameindex_map,
                       const std::vector<ImageSize> &images,
                       const std::vector<std::pair<int, int>> &image_pairs,
                       const size_t &initial_pair1, const size_t &initial_pair2,
                       bool b_use_initial_frame_prior, bool b_use_fundamental = false,
                       bool verbose = false,
                       const std::vector<std::string> *sequential_image_names = nullptr,
                       const std::string &output_dir = "");  // DONE(BCHO)

void ExpansionAndMatching(Map &map, const std::map<int, std::vector<int>> &retrieval_rank_of_frames,
                          const int num_iteration, const std::vector<ImageSize> &images,
                          const size_t &ini_frame1, const size_t &ini_frame2,
                          bool b_use_fundamental,
                          std::vector<std::pair<int, int>> &init_candidates);

void MapClustering(const Map &map, const std::map<int, std::vector<int>> &retrieval_rank_of_frames,
                   const std::vector<ImageSize> &images_size, std::vector<int> &root_of_frames);

void CreateClusterMap(const Map &map, const std::vector<int> &root_of_frames,
                      const std::map<int, std::vector<int>> &retrieval_rank_of_frames,
                      const size_t &initial_pair1, const size_t &initial_pair2,
                      const std::vector<ImageSize> &images_size, Map &cluster_map,
                      std::unordered_map<int, std::vector<int>> &frames_of_cluster,
                      std::map<int, std::vector<int>> &retrieval_rank_of_clusters,
                      size_t &initial_clusterid1, size_t &initial_clusterid2,
                      std::map<std::string, int> &name2clusterindex_map,
                      std::vector<ImageSize> &clusterimage_size);

}
#endif  // WSFM_FEATURE_PROCESSING_H
