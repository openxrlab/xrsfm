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
#include "orb_extractor.h"
#include "sift_extractor.h"
#include "utility/io_ecim.hpp"
#include "utility/timer.h"

const int max_match = 16384;
bool b_verbose_matches_option = true;
bool b_verbose_matches_size = false;

struct FeatureMatch {
  FeatureMatch() : point2D_idx1(0), point2D_idx2(0) {}

  FeatureMatch(const uint32_t point2D_idx1, const uint32_t point2D_idx2)
      : point2D_idx1(point2D_idx1), point2D_idx2(point2D_idx2) {}

  // Feature index in first image.
  uint32_t point2D_idx1 = -1;

  // Feature index in second image.
  uint32_t point2D_idx2 = -1;
};

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
  const float distance_th = 0.7;  // 100
  max_ratio = 0.8;                // 0.9 highly affect

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
    if (map12[i1] == -1 || dist12[i1] > distance_th || map21[map12[i1]] != i1 ||
        dist12[i1] > sdist12[i1] * max_ratio)
      continue;
    matches.emplace_back(Match(i1, map12[i1], dist12[i1]));
  }
}

void FeatureMatchingUnorder(const std::vector<Frame> &frames, std::vector<FramePair> &frame_pairs) {
  const int min_num_matches = 15;
  const int min_num_inlier = 15;
  const double min_ratio_inlier = 0.25;

#ifndef USE_ORB
  SiftMatchGPU sift_match_gpu;
  CreateSiftGPUMatcher(&sift_match_gpu);
#endif
  const int num_frames = static_cast<const int>(frames.size());
  for (int id = 0; id < num_frames; ++id) {
    const auto &frame = frames[id];
    for (int id_candidat = id - 1; id_candidat >= 0; --id_candidat) {
      const auto &frame_candidat = frames[id_candidat];
      FramePair frame_pair;
      frame_pair.id1 = id_candidat;
      frame_pair.id2 = id;
#ifndef USE_ORB
      // use uint_descs will have more matches
      // SiftMatch(frame_candidat.sift_descs_, frame.sift_descs_,
      // frame_pair.matches,&sift_match_gpu);
      SiftMatch(frame_candidat.uint_descs_, frame.uint_descs_, frame_pair.matches, &sift_match_gpu);
#else
      OrbMatch(frame_candidat.orb_descs_, frame.orb_descs_, frame_pair.matches);
#endif
      if (frame_pair.matches.size() < min_num_matches) continue;
      std::vector<Eigen::Vector2d> points1_normalized, points2_normalized;
      for (const auto &match : frame_pair.matches) {
        points1_normalized.push_back(frame_candidat.points_normalized[match.id1]);
        points2_normalized.push_back(frame.points_normalized[match.id2]);
      }
      itslam::solve_essential(points1_normalized, points2_normalized, 10.0 / 525, frame_pair.E,
                              frame_pair.inlier_num, frame_pair.inlier_mask);
      const int inlier_threshold =
          std::max(min_num_inlier, (int)(min_ratio_inlier * frame_pair.matches.size()));

      printf("id1:%d id2:%d %d %zu\n", id_candidat, id, frame_pair.inlier_num,
             frame_pair.matches.size());

      if (id_candidat < id - 1 && frame_pair.inlier_num < inlier_threshold) break;
      frame_pairs.emplace_back(frame_pair);
      //            DrawFeatureMatches(images[id_candidat], images[id], frame_candidat.keypoints_,
      //            frame.keypoints_, frame_pair.matches, frame_pair.inlier_mask);
    }
  }
}

void FeatureMatchingSeq(const std::vector<Frame> &frames, std::vector<FramePair> &frame_pairs) {
  const int min_num_matches = 30;
  const int min_num_inlier = 20;
  const double min_ratio_inlier = 0.25;
#ifndef USE_ORB
  SiftMatchGPU sift_match_gpu;
  CreateSiftGPUMatcher(&sift_match_gpu);
#endif
  const int num_frames = static_cast<const int>(frames.size());
  for (int id = 0; id < num_frames; ++id) {
    const auto &frame = frames[id];
    for (int id_candidat = id - 1; id_candidat >= 0 && id_candidat > id - 40; --id_candidat) {
      const auto &frame_candidat = frames[id_candidat];
      FramePair frame_pair;
      frame_pair.id1 = id_candidat;
      frame_pair.id2 = id;
#ifndef USE_ORB
      // use uint_descs will have more matches
      //            SiftMatch(frame_candidat.sift_descs_, frame.sift_descs_, frame_pair.matches,
      //            &sift_match_gpu);
      SiftMatch(frame_candidat.uint_descs_, frame.uint_descs_, frame_pair.matches, &sift_match_gpu);
#else
      OrbMatch(frame_candidat.orb_descs_, frame.orb_descs_, frame_pair.matches);
#endif
      if (frame_pair.matches.size() < min_num_matches && id_candidat < id - 1) break;
      std::vector<Eigen::Vector2d> points1_normalized, points2_normalized;
      for (const auto &match : frame_pair.matches) {
        points1_normalized.push_back(frame_candidat.points_normalized[match.id1]);
        points2_normalized.push_back(frame.points_normalized[match.id2]);
      }
      // TODO why 10.0 / 525
      itslam::solve_essential(points1_normalized, points2_normalized, 10.0 / 525, frame_pair.E,
                              frame_pair.inlier_num, frame_pair.inlier_mask);
      const int inlier_threshold =
          std::max(min_num_inlier, (int)(min_ratio_inlier * frame_pair.matches.size()));

      printf("id1:%d id2:%d %d %zu\n", id_candidat, id, frame_pair.inlier_num,
             frame_pair.matches.size());

      if (id_candidat < id - 1 && frame_pair.inlier_num < inlier_threshold) break;
      frame_pairs.emplace_back(frame_pair);
    }
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
    FramePair frame_pair;  // Note(BCHO): assure that the pair is ordered.
    frame_pair.id1 = id1;
    frame_pair.id2 = id2;
#ifndef USE_ORB
    // use uint_descs will have more matches frame.sift_descs_
    SiftMatch(frame1.uint_descs_, frame2.uint_descs_, frame_pair.matches, &sift_match_gpu);
#else
    OrbMatch(frame1.orb_descs_, frame2.orb_descs_, frame_pair.matches);
#endif
    if (frame_pair.matches.size() < min_num_matches) {
      continue;
    }
    if (!b_use_fundamental) {
      std::vector<Eigen::Vector2d> points1_normalized, points2_normalized;
      for (const auto &match : frame_pair.matches) {
        points1_normalized.push_back(frame1.points_normalized[match.id1]);
        points2_normalized.push_back(frame2.points_normalized[match.id2]);
      }
      itslam::solve_essential(points1_normalized, points2_normalized, 10.0 / 525, frame_pair.E,
                              frame_pair.inlier_num, frame_pair.inlier_mask);
    } else {
      std::vector<Eigen::Vector2d> points1, points2;
      for (const auto &match : frame_pair.matches) {
        points1.push_back(frame1.points[match.id1]);
        points2.push_back(frame2.points[match.id2]);
      }
      SolveFundamnetalCOLMAP(points1, points2, frame_pair);
    }
    const int inlier_threshold =
        std::max(min_num_inlier, (int)(min_ratio_inlier * frame_pair.matches.size()));
    if (frame_pair.inlier_num < inlier_threshold) continue;
    // filter the outlier matches
    std::vector<Match> inliner_matches =
        ExtractInlierMatches(frame_pair.matches, frame_pair.inlier_num, frame_pair.inlier_mask);
    frame_pair.inlier_mask.assign(inliner_matches.size(), true);
    frame_pair.matches.swap(inliner_matches);
    frame_pairs.emplace_back(frame_pair);
    count_inlier_pairs++;
  }
  std::cout << count_inlier_pairs << " " << candidate_pairs.size() << std::endl;
}

void GeometryVerification(const std::vector<Frame> &frames, std::vector<FramePair> &frame_pairs) {
  const int min_num_matches = 50;
  const int min_num_inlier = 30;
  const double min_ratio_inlier = 0.25;
  size_t num_inlier = 0;
  size_t num_tot = 0;
  for (auto &frame_pair : frame_pairs) {
    const int id = frame_pair.id1;
    const int id_candidat = frame_pair.id2;
    const auto &frame = frames[id];

    const auto &frame_candidat = frames[id_candidat];
    // FramePair frame_pair;  // Note(BCHO): assure that the pair is ordered.
    // frame_pair.id1 = id_candidat;
    // frame_pair.id2 = id;
    if (frame_pair.matches.size() < min_num_matches) continue;

    // //fundamental matrix of geometry verification
    {
      std::vector<Eigen::Vector2d> points1, points2;
      for (const auto &match : frame_pair.matches) {
        points1.push_back(frame.points[match.id1]);
        points2.push_back(frame_candidat.points[match.id2]);
      }
      colmap::Options option;
      option.ransac_options.max_error = 4.0;
      option.ransac_options.max_num_trials = 10000;
      option.ransac_options.min_num_trials = 100;
      option.ransac_options.confidence = 0.999;
      option.ransac_options.min_inlier_ratio = 0.25;
      colmap::LORANSAC<colmap::FundamentalMatrixSevenPointEstimator,
                       colmap::FundamentalMatrixEightPointEstimator>
          F_ransac(option.ransac_options);
      const auto F_report = F_ransac.Estimate(points1, points2);
      Eigen::Matrix3d F = F_report.model;
      frame_pair.inlier_num = F_report.support.num_inliers;
      frame_pair.inlier_mask = F_report.inlier_mask;
      frame_pair.F = F;
      std::cout << "fundamental:";
      // std::cout<<"fundamental geometry validation for
      // image("<<id<<","<<id_candidat<<"):"<<frame_pair.inlier_num<<".\n";
    }
    const int inlier_threshold =
        std::max(min_num_inlier, (int)(min_ratio_inlier * frame_pair.matches.size()));
    num_inlier += frame_pair.inlier_num;
    num_tot += frame_pair.matches.size();

    printf("(%d %d)(%d %d): %d  in %zu, %.2f%%\n", id_candidat, id, frame_candidat.points.size(),
           frame.points.size(), frame_pair.inlier_num, frame_pair.matches.size(),
           100.0 * frame_pair.inlier_num / frame_pair.matches.size());

    if (id_candidat < id - 1 && frame_pair.inlier_num < inlier_threshold) break;
  }
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

bool PatchVerification(const int row_num, const int col_num,
                       const std::vector<Eigen::Vector2d> &points1,
                       const std::vector<Eigen::Vector2d> &points2, const FramePair &frame_pair,
                       const ImageSize &image_size1, const ImageSize &image_size2,
                       bool verbose = false) {
  std::vector<std::vector<int>> patch_matches(row_num, std::vector<int>(col_num, 0));
  int step_x_1 = image_size1.width / col_num;
  int step_y_1 = image_size1.height / row_num;
  int step_x_2 = image_size2.width / col_num;
  int step_y_2 = image_size2.height / row_num;
  const int min_matches = 1;
  for (const Match &match : frame_pair.matches) {
    const Eigen::Vector2d &kpt1 = points1[match.id1];
    int x1 = (int)(kpt1.x() / step_x_1);
    int y1 = (int)(kpt1.y() / step_y_1);
    if (x1 >= col_num) x1 = col_num - 1;
    if (y1 >= row_num) y1 = row_num - 1;

    const Eigen::Vector2d &kpt2 = points2[match.id2];
    int x2 = (int)(kpt2.x() / step_x_2);
    int y2 = (int)(kpt2.y() / step_y_2);

    if (x2 >= col_num) x2 = col_num - 1;
    if (y2 >= row_num) y2 = row_num - 1;

    if (x1 == x2 && y1 == y2) {
      patch_matches[y1][x1]++;
    }
  }
  if (verbose) {
    std::cout << "FramePair " << frame_pair.id1 << " - " << frame_pair.id2 << "\n";
    for (int i = 0; i < row_num; i++) {
      for (int j = 0; j < col_num; j++) {
        std::cout << patch_matches[i][j] << "  ";
      }
      std::cout << "\n";
    }
  }

  for (int i = 0; i < row_num; i++) {
    for (int j = 0; j < col_num; j++) {
      if (patch_matches[i][j] < min_matches) {
        return false;
      }
    }
  }

  return true;
}

void MapClustering(const Map &map, const std::map<int, std::vector<int>> &retrieval_rank_of_frames,
                   const std::vector<ImageSize> &images_size, std::vector<int> &root_of_frames) {
  // clustering
  root_of_frames.assign(map.frames_.size(), -1);
  int num_clusters = 0;
  const int row_num = 3;
  const int col_num = 3;
  for (int i = 0; i < map.frames_.size(); i++) {
    if (root_of_frames[i] != -1) continue;
    root_of_frames[i] = i;
    num_clusters++;
    for (const int candidate_id : retrieval_rank_of_frames.at(i)) {
      if (root_of_frames[candidate_id] != -1) continue;
      std::vector<FramePair> frame_pairs;
      std::vector<std::pair<int, int>> tmp = {{i, candidate_id}};
      FeatureMatching(map.frames_, tmp, frame_pairs, true);
      if (frame_pairs.empty()) break;
      const FramePair &frame_pair = frame_pairs.at(0);
      if (PatchVerification(row_num, col_num, map.frames_.at(frame_pair.id1).points,
                            map.frames_.at(frame_pair.id2).points, frame_pair,
                            images_size.at(frame_pair.id1), images_size.at(frame_pair.id2))) {
        root_of_frames[candidate_id] = i;
      } else {
        break;
      }
    }
  }
}

void CreateClusterMap(const Map &map, const std::vector<int> &root_of_frames,
                      const std::map<int, std::vector<int>> &retrieval_rank_of_frames,
                      const size_t &initial_pair1, const size_t &initial_pair2,
                      const std::vector<ImageSize> &images_size, Map &cluster_map,
                      std::unordered_map<int, std::vector<int>> &frames_of_cluster,
                      std::map<int, std::vector<int>> &retrieval_rank_of_clusters,
                      size_t &initial_clusterid1, size_t &initial_clusterid2,
                      std::map<std::string, int> &name2clusterindex_map,
                      std::vector<ImageSize> &clusterimage_size) {
  // count number of clusters
  int num_clusters = 0;
  for (int i = 0; i < root_of_frames.size(); i++) {
    if (root_of_frames[i] == i) {
      num_clusters++;
    }
  }

  // set up frames of clusters
  std::vector<Frame> &clusters = cluster_map.frames_;
  // clusterid_of_root[old_root_id] = new_root_id
  clusters.reserve(map.frames_.size());
  std::unordered_map<int, int> clusterid_of_root;
  clusterid_of_root.reserve(num_clusters);
  clusterimage_size.reserve(num_clusters);
  for (int i = 0; i < map.frames_.size(); i++) {
    if (root_of_frames[i] == i) {
      Frame frame;
      const Frame &ref_frame = map.frames_[i];
      frame.id = clusters.size();
      frame.name = ref_frame.name;
      frame.uint_descs_ = ref_frame.uint_descs_;
      frame.points = ref_frame.points;
      frame.keypoints_ = ref_frame.keypoints_;
      frame.track_ids_ = ref_frame.track_ids_;
      clusters.emplace_back(frame);
      clusterid_of_root[i] = frame.id;
      clusterimage_size.emplace_back(images_size.at(i));
    }
  }

  // set up frames_of_cluster map
  for (int i = 0; i < map.frames_.size(); i++) {
    // if (root_of_frames[i] == i) continue; TODO
    int cluster_id = clusterid_of_root[root_of_frames[i]];
    frames_of_cluster[cluster_id].emplace_back(i);
  }

  // set up retrieval rank of frames
  // retrieval_rank_of_clusters.reserve(num_clusters);
  for (const auto retrieval_rank : retrieval_rank_of_frames) {
    if (!clusterid_of_root.count(retrieval_rank.first)) continue;
    std::vector<int> clustered_retrieval_rank;
    for (const int candidate_id : retrieval_rank.second) {
      if (!clusterid_of_root.count(candidate_id)) continue;
      clustered_retrieval_rank.emplace_back(clusterid_of_root.at(candidate_id));
    }
    if (!clustered_retrieval_rank.empty()) {
      retrieval_rank_of_clusters.emplace(clusterid_of_root.at(retrieval_rank.first),
                                         clustered_retrieval_rank);
    }
  }

  // set up frame pair
  for (const auto frame_pair : map.frame_pairs_) {
    if (clusterid_of_root.count(frame_pair.id1) && clusterid_of_root.count(frame_pair.id2)) {
      FramePair frame_pair;
      frame_pair.id1 = clusterid_of_root.at(frame_pair.id1);
      frame_pair.id2 = clusterid_of_root.at(frame_pair.id2);
      cluster_map.frame_pairs_.emplace_back(frame_pair);
    }
  }

  // set up name2clusterindex
  for (int i = 0; i < cluster_map.frames_.size(); i++) {
    name2clusterindex_map[cluster_map.frames_[i].name] = i;
  }

  initial_clusterid1 = clusterid_of_root[root_of_frames[initial_pair1]];
  initial_clusterid2 = clusterid_of_root[root_of_frames[initial_pair2]];
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