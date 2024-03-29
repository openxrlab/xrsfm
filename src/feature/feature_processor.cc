
#include "feature_processor.h"

#include <cctype>
#include <experimental/filesystem>
#include <regex>
#include <unordered_set>

#include "base/map.h"
#include "feature/feature_processing.h"
#include "utility/io_ecim.hpp"
#include "utility/timer.h"

namespace xrsfm {

FeatureProcessor::FeatureProcessor(std::string images_path,
                                   std::string output_path,
                                   std::string matching_type,
                                   std::string retrieval_path)
    : images_path_(images_path), output_path_(output_path),
      matching_type_(matching_type), retrieval_path_(retrieval_path) {
    if (!std::experimental::filesystem::exists(images_path)) {
        std::cout << "image path not exists :" << images_path << "\n";
        exit(-1);
    }
    if (!std::experimental::filesystem::exists(output_path)) {
        std::cout << "output path not exists :" << output_path << "\n";
        exit(-1);
    }
}

void FeatureProcessor::GetFeatures(const std::string &images_path,
                                   const std::vector<std::string> &image_names,
                                   const std::string &ftr_path,
                                   const std::string &size_path,
                                   std::vector<Frame> &frames,
                                   std::vector<ImageSize> &image_size) {

    std::ifstream ftr_bin(ftr_path);
    std::ifstream size_bin(size_path);
    if (ftr_bin.good() && size_bin.good()) {
        ReadFeatures(ftr_path, frames);
        SetUpFramePoints(frames);
        LoadImageSize(size_path, image_size);
    } else {
        FeatureExtract(images_path, frames, image_size);
        SaveFeatures(ftr_path, frames, true);
        SaveImageSize(size_path, image_size);
    }
}

void FeatureProcessor::GetInitFramePairs(
    const std::string &path, const std::vector<Frame> &frames,
    const std::vector<std::pair<int, int>> id_pairs,
    std::vector<FramePair> &frame_pairs) {
    std::ifstream bin(path);
    if (bin.good()) {
        ReadFramePairs(path, frame_pairs);
    } else {
        FeatureMatching(frames, id_pairs, frame_pairs, true);
        SaveFramePairs(path, frame_pairs);
    }
}

void FeatureProcessor::ExtractNearestImagePairs(
    const std::map<int, std::vector<int>> &id2rank, const int num_candidates,
    std::vector<std::pair<int, int>> &image_id_pairs) {
    std::set<std::pair<int, int>> image_pair_set;
    // search nv5 in candidates except for nearest images
    for (const auto &[id, retrieval_results] : id2rank) {
        int count = 0;
        for (const auto &id2 : retrieval_results) {
            auto ret = image_pair_set.insert(
                std::make_pair(std::min(id, id2), std::max(id, id2)));
            count++;
            if (count >= num_candidates)
                break;
        }
    }
    image_id_pairs.insert(image_id_pairs.end(), image_pair_set.begin(),
                          image_pair_set.end());
    std::sort(image_id_pairs.begin(), image_id_pairs.end(),
              [](auto &a, auto &b) {
                  if (a.first < b.first ||
                      (a.first == b.first && a.second < b.second))
                      return true;
                  return false;
              });
}

std::tuple<int, int>
FeatureProcessor::GetInitId(const int num_image,
                            std::vector<FramePair> &frame_pairs) {
    std::vector<std::map<int, int>> id2cor_num_vec(num_image);
    std::vector<std::pair<int, int>> connect_number_vec(num_image);
    std::map<int, std::set<int>> connect_id_vec;
    for (int i = 0; i < num_image; ++i) {
        connect_number_vec[i] = {i, 0};
    }

    for (auto &fp : frame_pairs) {
        if (fp.inlier_num < 100)
            continue; // for Trafalgar
        connect_number_vec[fp.id1].second++;
        connect_number_vec[fp.id2].second++;
        id2cor_num_vec[fp.id1][fp.id2] = fp.matches.size();
        id2cor_num_vec[fp.id2][fp.id1] = fp.matches.size();
    }
    std::sort(connect_number_vec.begin(), connect_number_vec.end(),
              [](const std::pair<int, int> &a, const std::pair<int, int> &b) {
                  return a.second > b.second;
              });
    const int init_id1 = connect_number_vec[0].first;
    int init_id2 = -1;
    for (auto &[id, number] : connect_number_vec) {
        if (id2cor_num_vec[init_id1].count(id) != 0 &&
            id2cor_num_vec[init_id1][id] >= 100) {
            init_id2 = id;
            break;
        }
    }
    return std::tuple<int, int>(init_id1, init_id2);
}

void FeatureProcessor::MatchingSeq(
    std::vector<Frame> &frames, const std::string &fp_path,
    const std::map<int, std::vector<int>> &id2rank,
    std::vector<std::pair<int, int>> &id_pairs) {
    const int num_frame = frames.size();
    std::set<std::pair<int, int>> set_pairs;
    for (int i = 0; i < num_frame; ++i) {
        for (int k = 1; k < 20 && i + k < num_frame; ++k)
            set_pairs.insert(std::pair<int, int>(i, i + k));
    }

    for (const auto &[id1, vec] : id2rank) {
        if (id1 % 5 != 0)
            continue;
        for (auto &id2 : vec) {
            if (id1 < id2)
                set_pairs.insert(std::pair<int, int>(id1, id2));
            else if (id2 < id1)
                set_pairs.insert(std::pair<int, int>(id2, id1));
        }
    }

    id_pairs.assign(set_pairs.begin(), set_pairs.end());
}

void FeatureProcessor::Run() {
    const std::string ftr_path = output_path_ + "ftr.bin";
    const std::string size_path = output_path_ + "size.bin";
    const std::string fp_init_path = output_path_ + "fp_init.bin";
    const std::string fp_path = output_path_ + "fp.bin";

    // 2.read images
    std::vector<std::string> image_names;
    LoadImageNames(images_path_, image_names);
    const int num_image = image_names.size();
    std::cout << "Load Image Info Done.\n";

    std::vector<Frame> frames;
    frames.resize(num_image);
    for (int i = 0; i < num_image; ++i) {
        frames[i].id = i;
        frames[i].name = image_names[i];
    }

    // 2.feature extraction
    std::vector<ImageSize> image_size_vec;
    GetFeatures(images_path_, image_names, ftr_path, size_path, frames,
                image_size_vec);
    std::cout << "Extract Features Done.\n";

    // 3.image matching
    std::map<int, std::vector<int>> id2rank;
    std::map<std::string, int> name2id;
    for (int i = 0; i < num_image; ++i) {
        name2id[image_names[i]] = i;
    }
    bool have_retrieval_info =
        LoadRetrievalRank(retrieval_path_, name2id, id2rank);
    std::cout << "Load Retrieval Info Done.\n";

    Timer timer("%lf s\n");
    timer.start();

    std::vector<FramePair> frame_pairs;
    if (matching_type_ == "covisibility") {
        if (!have_retrieval_info) {
            std::cout << "The retrieval file is required when using "
                         "covisibility-based matching!!!\n";
            return 0;
        }
        std::vector<std::pair<int, int>> id_pairs;
        ExtractNearestImagePairs(id2rank, 5, id_pairs);
        GetInitFramePairs(fp_init_path, frames, id_pairs, frame_pairs);
        std::cout << "Init Matching Done.\n";

        constexpr int num_iteration = 5;
        constexpr bool use_fundamental = true;
        const auto [init_id1, init_id2] = GetInitId(num_image, frame_pairs);

        Map map;
        map.frames_ = std::move(frames);
        map.frame_pairs_ = std::move(frame_pairs);
        ExpansionAndMatching(map, id2rank, num_iteration, image_size_vec,
                             init_id1, init_id2, use_fundamental, id_pairs);
    } else {
        std::vector<std::pair<int, int>> id_pairs;
        if (matching_type_ == "sequential") {
            if (!have_retrieval_info) {
                std::cout << "Without the retrieval file, sequential matching "
                             "method only matches adjacent frames.\n";
            }
            MatchingSeq(frames, fp_path, id2rank, id_pairs);
        } else if (matching_type_ == "retrieval") {
            if (!have_retrieval_info) {
                std::cout << "ERROR: The retrieval file is required when using "
                             "retrieval-based matching!!!\n";
                return 0;
            }
            ExtractNearestImagePairs(id2rank, 25, id_pairs);
        }
        FeatureMatching(frames, id_pairs, frame_pairs, true);
    }
    SaveFramePairs(fp_path, frame_pairs);

    timer.stop();
    timer.print();
}

} // namespace xrsfm
