

#include <cctype>
#include <regex>
#include <unordered_set>

#include "feature/feature_processing.h"
#include "base/map.h"
#include "utility/io_ecim.hpp"
#include "utility/timer.h"

using namespace xrsfm;

void SetUpFramePoints(std::vector<Frame>& frames) {
    for (auto& frame : frames) {
        const int num_points = frame.keypoints_.size();
        frame.points.clear();
        frame.points_normalized.clear();
        for (const auto& kpt : frame.keypoints_) {
            const auto& pt = kpt.pt;
            frame.points.emplace_back(pt.x, pt.y);
        }
        frame.track_ids_.assign(num_points, -1);
    }
}

void GetFeatures(const std::string& image_dir_path, const std::string& ftr_path,
                 std::vector<Frame>& frames) {
    std::ifstream ftr_bin(ftr_path);
    if (ftr_bin.good()) {
        ReadFeatures(ftr_path, frames);
    } else {
        FeatureExtract(image_dir_path, frames);
        SaveFeatures(ftr_path, frames, true);
    }
    SetUpFramePoints(frames);
}

void GetImageSizeVec(const std::string& image_dir_path, const std::vector<std::string>& image_names,
                     const std::string& path, std::vector<ImageSize>& image_size) {
    std::ifstream bin(path);
    if (bin.good()) {
        LoadImageSize(path, image_size);
    } else {
        for (const auto& image_name : image_names) {
            const cv::Mat image = cv::imread(image_dir_path + image_name);
            image_size.push_back(ImageSize(image.cols, image.rows));
        }
        SaveImageSize(path, image_size);
    }
}

void GetInitFramePairs(const std::string& path, const std::vector<Frame>& frames,
                       const std::vector<std::pair<int, int>> id_pairs,
                       std::vector<FramePair>& frame_pairs) {
    std::ifstream bin(path);
    if (bin.good()) {
        ReadFramePairs(path, frame_pairs);
    } else {
        FeatureMatching(frames, id_pairs, frame_pairs, true);
        SaveFramePairs(path, frame_pairs);
    }
}

inline void ExtractNearestImagePairs(const std::map<int, std::vector<int>>& id2rank,
                                     const int num_candidates,
                                     std::vector<std::pair<int, int>>& image_id_pairs) {
    std::set<std::pair<int, int>> image_pair_set;
    // search nv5 in candidates except for nearest images
    for (const auto& [id, retrieval_results] : id2rank) {
        int count = 0;
        for (const auto& id2 : retrieval_results) {
            auto ret = image_pair_set.insert(std::make_pair(std::min(id, id2), std::max(id, id2)));
            // if (!ret.second)continue;
            count++;
            if (count >= num_candidates) break;
        }
    }
    image_id_pairs.insert(image_id_pairs.end(), image_pair_set.begin(), image_pair_set.end());
    std::sort(image_id_pairs.begin(), image_id_pairs.end(), [](auto& a, auto& b) {
        if (a.first < b.first || (a.first == b.first && a.second < b.second)) return true;
        return false;
    });
}

std::tuple<int, int> GetInitId(const int num_image, std::vector<FramePair>& frame_pairs) {
    std::vector<std::map<int, int>> id2cor_num_vec(num_image);
    std::vector<std::pair<int, int>> connect_number_vec(num_image);
    std::map<int, std::set<int>> connect_id_vec;
    for (int i = 0; i < num_image; ++i) {
        connect_number_vec[i] = {i, 0};
    }

    for (auto& fp : frame_pairs) {
        if (fp.inlier_num < 100) continue; // for Trafalgar
        connect_number_vec[fp.id1].second++;
        connect_number_vec[fp.id2].second++;
        id2cor_num_vec[fp.id1][fp.id2] = fp.matches.size();
        id2cor_num_vec[fp.id2][fp.id1] = fp.matches.size();
    }
    std::sort(connect_number_vec.begin(), connect_number_vec.end(),
              [](const std::pair<int, int>& a, const std::pair<int, int>& b) {
                  return a.second > b.second;
              });
    const int init_id1 = connect_number_vec[0].first;
    // for (auto& fp : frame_pairs) {
    //     if (fp.id1 == init_id1 || fp.id2 == init_id1) {
    //         std::cout << "infp: " << fp.id1 << " " << fp.id2 << " " << fp.matches.size() << " "
    //                   << fp.inlier_num << std::endl;
    //     }
    // // }
    // std::cout << "init_id1: " << connect_number_vec[0].first << " number: " << connect_number_vec[0].second
    //           << std::endl;
    int init_id2 = -1;
    for (auto& [id, number] : connect_number_vec) {
        if (id2cor_num_vec[init_id1].count(id) != 0 && id2cor_num_vec[init_id1][id] >= 100) {
            init_id2 = id;
            // std::cout << id << " " << number << std::endl;
            break;
        }
    }
    // std::cout << id2cor_num_vec[init_id1][init_id2] << std::endl;
    return std::tuple<int, int>(init_id1, init_id2);
}

void MatchingSeq(std::vector<Frame>& frames, const std::string& fp_path, std::map<int, std::vector<int>>& id2rank) {
    std::set<std::pair<int, int>> set_pairs;
    for (int i = 0, num = frames.size(); i < num; ++i) {
        for (int k = 0; k < 20 && i + k < num; ++k) set_pairs.insert(std::pair<int, int>(i, i + k));
    }
    for (auto& [id1, vec] : id2rank) {
        if (id1 % 5 != 0) continue;
        for (auto& id2 : vec) {
            set_pairs.insert(std::pair<int, int>(id1, id2));
        }
    }
    std::vector<std::pair<int, int>> id_pairs;
    id_pairs.assign(set_pairs.begin(), set_pairs.end());

    std::vector<FramePair> frame_pairs;
    FeatureMatching(frames, id_pairs, frame_pairs, true);
    SaveFramePairs(fp_path, frame_pairs);
}

int main(int argc, const char* argv[]) {
    std::string config_path = "./config_open.json";
    auto config_json = LoadJSON(config_path);
    const std::string image_dir_path = config_json["image_dir_path"];
    const std::string retrival_path = config_json["retrival_path"];
    const std::string matching_type = config_json["matching_type"];
    const std::string output_path = config_json["output_path"];
    std::string ftr_path = output_path + "ftr.bin";
    std::string size_path = output_path + "size.bin";
    std::string fp_init_path = output_path + "fp_init.bin";
    std::string fp_path = output_path + "fp.bin";

    // 1.read images
    std::vector<std::string> image_names;
    LoadImageNames(image_dir_path, image_names);
    std::vector<ImageSize> image_size_vec;
    GetImageSizeVec(image_dir_path, image_names, size_path, image_size_vec);
    std::map<std::string, int> name2id;
    const int num_image = image_names.size();
    std::vector<Frame> frames(num_image);
    for (int i = 0; i < num_image; ++i) {
        frames[i].id = i;
        frames[i].name = image_names[i];
        name2id[image_names[i]] = i;
    }
    std::cout << "Load Image Info Done.\n";

    // 2.feature extraction
    GetFeatures(image_dir_path, ftr_path, frames);
    std::cout << "Extract Features Done.\n";

    // 5.image matching
    std::map<int, std::vector<int>> id2rank;
    LoadRetrievalRank(retrival_path, name2id, id2rank);
    std::cout << "Load Retrieval Info Done.\n";

    Timer timer("%lf s\n");
    timer.start();
    if (matching_type == "sequential") {
        MatchingSeq(frames, fp_path, id2rank);
    } else if (matching_type == "unordered") {
        std::vector<std::pair<int, int>> id_pairs;
        ExtractNearestImagePairs(id2rank, 25, id_pairs);
        std::vector<FramePair> frame_pairs;
        FeatureMatching(frames, id_pairs, frame_pairs, true);
        SaveFramePairs(fp_path, frame_pairs);
    } else {
        std::vector<std::pair<int, int>> id_pairs;
        ExtractNearestImagePairs(id2rank, 5, id_pairs);
        std::vector<FramePair> frame_pairs;
        GetInitFramePairs(fp_init_path, frames, id_pairs, frame_pairs);
        std::cout << "Init Matching Done.\n";

        constexpr int num_iteration = 5;
        constexpr bool use_fundamental = true;
        const auto [init_id1, init_id2] = GetInitId(num_image, frame_pairs);

        Map map;
        map.frames_ = std::move(frames);
        map.frame_pairs_ = std::move(frame_pairs);
        ExpansionAndMatching(map, id2rank, num_iteration, image_size_vec, init_id1, init_id2,
                             use_fundamental, id_pairs);
        SaveFramePairs(fp_path, map.frame_pairs_);
    }
    timer.stop();
    timer.print();

    return 0;
}