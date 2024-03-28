
#ifndef XRSFM_SRC_FEATURE_FEATURE_PROCESSOR_H
#define XRSFM_SRC_FEATURE_FEATURE_PROCESSOR_H

#include "base/map.h"
#include "feature/feature_processing.h"
#include "utility/io_ecim.hpp"
#include "utility/timer.h"

namespace xrsfm {
class FeatureProcessor {
  public:
    FeatureProcessor(std::string images_path, std::string output_path,
                     std::string matching_type = "sequential",
                     std::string retrieval_path = "");

    void Run();

  private:
    void GetFeatures(const std::string &images_path,
                     const std::vector<std::string> &image_names,
                     const std::string &ftr_path, const std::string &size_path,
                     std::vector<Frame> &frames,
                     std::vector<ImageSize> &image_size);
    void GetInitFramePairs(const std::string &path,
                           const std::vector<Frame> &frames,
                           const std::vector<std::pair<int, int>> id_pairs,
                           std::vector<FramePair> &frame_pairs);
    void
    ExtractNearestImagePairs(const std::map<int, std::vector<int>> &id2rank,
                             const int num_candidates,
                             std::vector<std::pair<int, int>> &image_id_pairs);
    std::tuple<int, int> GetInitId(const int num_image,
                                   std::vector<FramePair> &frame_pairs);
    void MatchingSeq(std::vector<Frame> &frames, const std::string &fp_path,
                     const std::map<int, std::vector<int>> &id2rank,
                     std::vector<std::pair<int, int>> &id_pairs);

    std::string images_path_;
    std::string output_path_;
    std::string matching_type_;
    std::string retrieval_path_;
};
} // namespace xrsfm

#endif // XRSFM_SRC_FEATURE_FEATURE_PROCESSOR_H
