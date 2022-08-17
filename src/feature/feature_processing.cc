//
// Created by yzc on 19-4-8.
//

#include "feature_processing.h"

#include <x86intrin.h>

#include <algorithm>
#include <numeric>

#include "geometry/colmap/estimators/fundamental_matrix.h"
#include "geometry/epipolar_geometry.hpp"
#include "geometry/essential.h"
#include "base/map.h"
#include "optim/loransac.h"
#include "optim/ransac.h"
#include "sift_extractor.h"
#include "utility/io_ecim.hpp"
#include "utility/timer.h"
#include "match_expansion.h"

namespace xrsfm {
constexpr int max_match = 16384;
constexpr bool b_verbose_matches_option = true;
constexpr bool b_verbose_matches_size = false;

struct FeatureMatch {
    FeatureMatch() :
        point2D_idx1(0), point2D_idx2(0) {
    }

    FeatureMatch(const uint32_t point2D_idx1, const uint32_t point2D_idx2) :
        point2D_idx1(point2D_idx1), point2D_idx2(point2D_idx2) {
    }

    // Feature index in first image.
    uint32_t point2D_idx1 = -1;

    // Feature index in second image.
    uint32_t point2D_idx2 = -1;
};

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


bool CreateSiftGPUMatcher(SiftMatchGPU *sift_match_gpu) {
    // SiftGPU uses many global static state variables and the initialization
    // must be thread-safe in order to work correctly. This is enforced here.
    std::vector<int> gpu_indices(1, 0);
    int num_cuda_devices = 1;
    //    cudaGetDeviceCount(&num_cuda_devices);
    gpu_indices.resize(num_cuda_devices);
    std::iota(gpu_indices.begin(), gpu_indices.end(), 0);

    SiftGPU sift_gpu;
    sift_gpu.SetVerbose(0);

    *sift_match_gpu = SiftMatchGPU(4096);

    if (gpu_indices[0] >= 0) {
        sift_match_gpu->SetLanguage(SiftMatchGPU::SIFTMATCH_CUDA_DEVICE0 + gpu_indices[0]);
    } else {
        sift_match_gpu->SetLanguage(SiftMatchGPU::SIFTMATCH_CUDA);
    }

    if (sift_match_gpu->VerifyContextGL() == 0) {
        return false;
    }

    if (!sift_match_gpu->Allocate(max_match, true)) {
        std::cout << "ERROR: Not enough GPU memory to match %d features. "
                     "Reduce the maximum number of matches."
                  << std::endl;
        return false;
    }

    sift_match_gpu->gpu_index = gpu_indices[0];

    return true;
}

void SiftMatch(const FeatureDescriptors &descs1, const FeatureDescriptors &descs2,
               std::vector<Match> &matches, SiftMatchGPU *sift_match_gpu, float max_ratio) {
    // Loose threshold will be slow while strict lead to little matches
    const float distance_th = 1.0;
    max_ratio = 0.9;

    std::vector<FeatureMatch> t_matches(max_match);
    const int num1 = (int)descs1.rows();
    const int num2 = (int)descs2.rows();
    sift_match_gpu->SetDescriptors(0, num1, descs1.data());
    sift_match_gpu->SetDescriptors(1, num2, descs2.data());

    const int max_num_matches = num1;
    matches.resize(static_cast<size_t>(max_num_matches));
    int num_matches = sift_match_gpu->GetSiftMatch(max_num_matches,
                                                   reinterpret_cast<uint32_t(*)[2]>(t_matches.data()),
                                                   distance_th, max_ratio, true);

    // printf("%d\n",num_matches);
    matches.resize(num_matches);
    for (int i = 0; i < num_matches; ++i) {
        int id1 = t_matches[i].point2D_idx1;
        int id2 = t_matches[i].point2D_idx2;
        matches[i] = Match(id1, id2);
    }
}

void SiftMatch(const UINT8Descriptors &descs1, const UINT8Descriptors &descs2,
               std::vector<Match> &matches, SiftMatchGPU *sift_match_gpu, float max_ratio) {
    // Loose threshold will be slow while strict lead to little matches
    const float distance_th = 0.7; // 100
    max_ratio = 0.8;               // 0.9 highly affect

    std::vector<FeatureMatch> t_matches(max_match);
    int num1 = (int)descs1.rows();
    int num2 = (int)descs2.rows();
    if (b_verbose_matches_size) {
        std::cout << "n1: " << num1 << "->\n";
        std::cout << "n2: " << num2 << "->\n";
        std::cout << "c: " << (int)descs1.cols() << "->\n";
        std::cout << "distance_th: " << distance_th << "->\n";
        std::cout << "max_ratio: " << max_ratio << "->\n";
        std::cout << "max_match: " << max_match << "->\n";
    }
    sift_match_gpu->SetDescriptors(0, num1, descs1.data());
    sift_match_gpu->SetDescriptors(1, num2, descs2.data());

    const int max_num_matches = max_match;
    if (b_verbose_matches_size) {
        std::cout << "max_num_matches: " << max_num_matches << "->\n";
    }
    t_matches.resize(static_cast<size_t>(max_num_matches));
    int num_matches = sift_match_gpu->GetSiftMatch(max_num_matches,
                                                   reinterpret_cast<uint32_t(*)[2]>(t_matches.data()),
                                                   distance_th, max_ratio, true);

    matches.resize(num_matches);
    for (int i = 0; i < num_matches; ++i) {
        int id1 = t_matches[i].point2D_idx1;
        int id2 = t_matches[i].point2D_idx2;
        matches[i] = Match(id1, id2);
    }
}

int OrbDistance(const unsigned char *A, const unsigned char *B) {
    int dist = 0;
    //    _mm_popcnt_u64()
    const unsigned int *pa = (unsigned int *)A;
    const unsigned int *pb = (unsigned int *)B;
    for (int i = 0; i < 8; i++, pa++, pb++) {
        int v = *pa ^ *pb;
        v = v - ((v >> 1) & 0x55555555);
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        int tdis = (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
        dist += tdis;
    }
    return dist;
}

void OrbMatch(const cv::Mat &descs1, const cv::Mat &descs2, std::vector<Match> &matches) {
    const int max_dist = 256;
    const int distance_th = 80;
    const float max_ratio = 0.9;

    Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> dists(descs1.rows,
                                                                              descs2.rows);
    for (int i1 = 0; i1 < descs1.rows; ++i1) {
        for (int i2 = 0; i2 < descs2.rows; ++i2) {
            dists(i1, i2) = OrbDistance(descs1.data + i1 * 32, descs2.data + i2 * 32);
        }
    }
    // find matches
    matches.resize(0);
    std::vector<int> map12(descs1.rows, -1), map21(descs2.rows, -1);
    std::vector<int> dist12(descs1.rows, max_dist), dist21(descs2.rows, max_dist);
    std::vector<int> sdist12(descs1.rows, max_dist), sdist21(descs2.rows, max_dist);
    for (int i1 = 0; i1 < descs1.rows; ++i1) {
        for (int i2 = 0; i2 < descs2.rows; ++i2) {
            const int dist = dists(i1, i2);
            // update map12
            if (dist < dist12[i1]) {
                map12[i1] = i2;
                sdist12[i1] = dist12[i1];
                dist12[i1] = dist;
            } else if (dist < sdist12[i1]) {
                sdist12[i1] = dist;
            }
            // update map21
            if (dist < dist21[i2]) {
                map21[i2] = i1;
                sdist21[i2] = dist21[i2];
                dist21[i2] = dist;
            } else if (dist < sdist12[i2]) {
                sdist21[i2] = dist;
            }
        }
    }
    for (int i1 = 0; i1 < descs1.rows; ++i1) {
        if (map12[i1] == -1 || dist12[i1] > distance_th || map21[map12[i1]] != i1 || dist12[i1] > sdist12[i1] * max_ratio)
            continue;
        matches.emplace_back(Match(i1, map12[i1], dist12[i1]));
    }
}

static bool init = false;
void FeatureMatching(const std::vector<Frame> &frames,
                     const std::vector<std::pair<int, int>> &candidate_pairs,
                     std::vector<FramePair> &frame_pairs, bool b_use_fundamental) {
    constexpr int min_num_matches = 15;
    constexpr int min_num_inlier = 15;
    constexpr double min_ratio_inlier = 0.25;

#ifndef USE_ORB
    static SiftMatchGPU sift_match_gpu;
    if (!init) {
        CreateSiftGPUMatcher(&sift_match_gpu);
        init = true;
    }
#endif

    int count_inlier_pairs = 0;
    const int num_frames = static_cast<const int>(frames.size());
    for (const auto &[id1, id2] : candidate_pairs) {
        const auto &frame1 = frames[id1];
        const auto &frame2 = frames[id2];
        FramePair frame_pair;
        frame_pair.id1 = id1;
        frame_pair.id2 = id2;
        frame_pair.inlier_num = 0;
#ifndef USE_ORB
        // use uint_descs will have more matches frame.sift_descs_
        SiftMatch(frame1.uint_descs_, frame2.uint_descs_, frame_pair.matches, &sift_match_gpu);
#else
        OrbMatch(frame1.orb_descs_, frame2.orb_descs_, frame_pair.matches);
#endif
        frame_pairs.emplace_back(frame_pair);
    }
// #pragma omp parallel for schedule(dynamic, 8)
    for (auto &frame_pair : frame_pairs) {
        if (frame_pair.matches.size() < min_num_matches) {
            continue;
        }
        const auto &frame1 = frames[frame_pair.id1];
        const auto &frame2 = frames[frame_pair.id2];
        if (b_use_fundamental) {
            std::vector<Eigen::Vector2d> points1, points2;
            for (const auto &match : frame_pair.matches) {
                points1.push_back(frame1.points[match.id1]);
                points2.push_back(frame2.points[match.id2]);
            }
            SolveFundamnetalCOLMAP(points1, points2, frame_pair);
        } else {
            std::vector<Eigen::Vector2d> points1_normalized, points2_normalized;
            for (const auto &match : frame_pair.matches) {
                points1_normalized.push_back(frame1.points_normalized[match.id1]);
                points2_normalized.push_back(frame2.points_normalized[match.id2]);
            }
            xrsfm::solve_essential(points1_normalized, points2_normalized, 10.0 / 525, frame_pair.E,
                                   frame_pair.inlier_num, frame_pair.inlier_mask);
        }
        const int inlier_threshold =
            std::max(min_num_inlier, (int)(min_ratio_inlier * frame_pair.matches.size()));
        if (frame_pair.inlier_num < inlier_threshold) {
            frame_pair.inlier_num = 0;
            continue;
        }
        // filter the outlier matches
        std::vector<Match> inliner_matches =
            ExtractInlierMatches(frame_pair.matches, frame_pair.inlier_num, frame_pair.inlier_mask);
        frame_pair.inlier_mask.assign(inliner_matches.size(), true);
        frame_pair.matches.swap(inliner_matches);
    }
    
    std::vector<FramePair> frame_pairs_filter;
    for (auto &frame_pair : frame_pairs) {
        if (frame_pair.inlier_num == 0) continue;
        frame_pairs_filter.emplace_back(frame_pair);
        count_inlier_pairs++;
    }
    frame_pairs = std::move(frame_pairs_filter);
    std::cout << "matched image pairs: " << count_inlier_pairs << "/" << candidate_pairs.size() << std::endl;
}

std::vector<Match> ExtractInlierMatches(const std::vector<Match> &matches, const size_t num_inliers,
                                        const std::vector<char> &inlier_mask) {
    std::vector<Match> inlier_matches(num_inliers);
    size_t j = 0;
    for (size_t i = 0; i < matches.size(); ++i) {
        if (inlier_mask[i]) {
            inlier_matches[j] = matches[i];
            j += 1;
        }
    }
    return inlier_matches;
}

void ExpansionAndMatching(Map &map, const std::map<int, std::vector<int>> &id2rank,
                          const int num_iteration, const std::vector<ImageSize> &images,
                          const size_t &ini_frame1, const size_t &ini_frame2,
                          bool b_use_fundamental,
                          std::vector<std::pair<int, int>> &init_candidates) {
    Timer timer_searching("Time searching %.6lfs\n");
    Timer timer_matching("Time matching %.6lfs\n");

    std::cout << "init: " << map.frame_pairs_.size() << " " << init_candidates.size() << std::endl;
    std::vector<std::set<int>> id2matched_ids(map.frames_.size());
    for (auto &[id1, id2] : init_candidates) {
        id2matched_ids[id1].insert(id2);
        id2matched_ids[id2].insert(id1);
    }

    auto &all_candidates = init_candidates;
    for (int i = 0; i < num_iteration; i++) {
        std::cout << "////////Iter (" << i << ")/////////\n";
        // set up match expansion solver
        std::vector<std::pair<int, int>> cur_candidats;
        MatchExpansionSolver match_expansion_solver;
        match_expansion_solver.id2matched_ids = id2matched_ids;
        match_expansion_solver.SetUp(map, id2rank, images, ini_frame1, ini_frame2);
        TIMING(timer_searching, match_expansion_solver.Run(map, cur_candidats));

        std::vector<std::pair<int, int>> filtered_candidates;
        for (auto &[id1, id2] : cur_candidats) {
            if (id2matched_ids[id1].count(id2) == 0) {
                filtered_candidates.push_back({id1, id2});
                all_candidates.push_back({id1, id2});
                id2matched_ids[id1].insert(id2);
                id2matched_ids[id2].insert(id1);
            }
        }
        cur_candidats = filtered_candidates;

        TIMING(timer_matching,
               FeatureMatching(map.frames_, cur_candidats, map.frame_pairs_, b_use_fundamental));

        std::cout << "expansion: " << cur_candidats.size() << " " << map.frame_pairs_.size() << " "
                  << all_candidates.size() << std::endl;
        std::cout << "precision: " << 100.0 * map.frame_pairs_.size() / all_candidates.size()
                  << std::endl;
        timer_searching.print();
        timer_matching.print();
    }
}
} // namespace xrsfm
