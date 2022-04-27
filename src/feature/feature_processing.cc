//
// Created by yzc on 19-4-8.
//

#include "feature_processing.h"

#include <x86intrin.h>

#include <algorithm>
#include <numeric>

#include "geometry/essential.h"
#include "base/map.h"
#include "orb_extractor.h"
#include "sift_extractor.h"

//#define USE_ORB

const int max_match = 4096;

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

  *sift_match_gpu = SiftMatchGPU(max_match);

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

void SiftMatch(const FeatureDescriptors &descs1, const FeatureDescriptors &descs2, std::vector<Match> &matches,
               SiftMatchGPU *sift_match_gpu, float max_ratio) {
  // Loose threshold will be slow while strict lead to little matches
  const float distance_th = 1.0;
  max_ratio = 0.9;

  std::vector<FeatureMatch> t_matches(max_match);
  const int num1 = (int)descs1.rows();
  const int num2 = (int)descs2.rows();
  sift_match_gpu->SetDescriptors(0, num1, descs1.data());
  sift_match_gpu->SetDescriptors(1, num2, descs2.data());
  // printf("%d %d\n",num1,num2);
  const int max_num_matches = num1;
  matches.resize(static_cast<size_t>(max_num_matches));
  int num_matches = sift_match_gpu->GetSiftMatch(max_num_matches, reinterpret_cast<uint32_t(*)[2]>(t_matches.data()),
                                                 distance_th, max_ratio, true);

  // printf("%d\n",num_matches);
  matches.resize(num_matches);
  for (int i = 0; i < num_matches; ++i) {
    int id1 = t_matches[i].point2D_idx1;
    int id2 = t_matches[i].point2D_idx2;
    matches[i] = Match(id1, id2);
  }
}

void SiftMatch(const UINT8Descriptors &descs1, const UINT8Descriptors &descs2, std::vector<Match> &matches,
               SiftMatchGPU *sift_match_gpu, float max_ratio) {
  // Loose threshold will be slow while strict lead to little matches
  const int max_dist = 256;
  const float distance_th = 0.8;  // 100
  max_ratio = 0.7;                // 0.9 highly affect

  std::vector<FeatureMatch> t_matches(max_match);
  int num1 = (int)descs1.rows();
  int num2 = (int)descs2.rows();
  sift_match_gpu->SetDescriptors(0, num1, descs1.data());
  sift_match_gpu->SetDescriptors(1, num2, descs2.data());
  // printf("%d %d\n",num1,num2);
  const int max_num_matches = num1;
  matches.resize(static_cast<size_t>(max_num_matches));
  int num_matches = sift_match_gpu->GetSiftMatch(max_num_matches, reinterpret_cast<uint32_t(*)[2]>(t_matches.data()),
                                                 distance_th, max_ratio, true);

  // printf("%d\n",num_matches);
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

  Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> dists(descs1.rows, descs2.rows);
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

void FeatureExtract(const std::string &dir_path, const Camera &camera, const std::vector<cv::Mat> &images,
                    std::vector<Frame> &frames) {
#ifndef USE_ORB
  SiftExtractor sift(2048);
#else
  const int _feature_num = 2048;
  const float _lever_ratio = 1.2;
  const int _lever_num = 8;
  const int _initTh = 20;
  const int _minTh = 7;
  ORB_SLAM2::OrbExtractor orb(_feature_num, _lever_ratio, _lever_num, _initTh, _minTh);
#endif

  int id = 0;
  for (const auto &image : images) {
    Frame frame;
    frame.id = id++;
#ifndef USE_ORB
    sift(image, frame.keypoints_, frame.sift_descs_);
    frame.uint_descs_ = FeatureDescriptorsToUnsignedByte(frame.sift_descs_);
#else
    orb(image, cv::Mat(), frame.keypoints_, frame.orb_descs_);
#endif
    frame.points.clear();
    frame.points_normalized.clear();
    size_t num_points = frame.keypoints_.size();
    //        cv::Point2f pt_pre;
    for (size_t i = 0; i < num_points; ++i) {
      const auto &pt = frame.keypoints_[i].pt;

      //            if(pt==pt_pre){
      //                std::cout<<i-1<<" "<<frame.keypoints_[i-1].size<<"
      //                "<<frame.keypoints_[i-1].angle<<std::endl;
      //                std::cout<<i<<" "<<pt.x<<" "<<pt.y<<"
      //                "<<frame.keypoints_[i].size<<"
      //                "<<frame.keypoints_[i].angle<<std::endl;
      //            }
      //            pt_pre = pt;

      frame.points.emplace_back(pt.x, pt.y);
      frame.points_normalized.emplace_back((pt.x - camera.cx()) / camera.fx(), (pt.y - camera.cy()) / camera.fy());
    }
    frame.track_ids_.resize(num_points, -1);
    frames.emplace_back(frame);
    // DrawFeature(image,frame.keypoints_);
  }
}

void FeatureMatchingUnorder(const std::vector<Frame> &frames, std::vector<FramePair> &frame_pairs) {
  const int min_num_matches = 50;
  const int min_num_inlier = 30;
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
      //            SiftMatch(frame_candidat.sift_descs_, frame.sift_descs_,
      //            frame_pair.matches, &sift_match_gpu);
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
      // TODO 10.0 / 525
      itslam::solve_essential(points1_normalized, points2_normalized, 10.0 / 525, frame_pair.E, frame_pair.inlier_num,
                              frame_pair.inlier_mask);
      const int inlier_threshold = std::max(min_num_inlier, (int)(min_ratio_inlier * frame_pair.matches.size()));

      printf("id1:%d id2:%d %d %zu\n", id_candidat, id, frame_pair.inlier_num, frame_pair.matches.size());

      if (id_candidat < id - 1 && frame_pair.inlier_num < inlier_threshold) break;
      frame_pairs.emplace_back(frame_pair);
      //            DrawFeatureMatches(images[id_candidat], images[id],
      //            frame_candidat.keypoints_, frame.keypoints_,
      //                               frame_pair.matches,
      //                               frame_pair.inlier_mask);
    }
  }
}

void FeatureMatching(const std::vector<Frame> &frames, std::vector<FramePair> &frame_pairs) {
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
      //            SiftMatch(frame_candidat.sift_descs_, frame.sift_descs_,
      //            frame_pair.matches, &sift_match_gpu);
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
      itslam::solve_essential(points1_normalized, points2_normalized, 10.0 / 525, frame_pair.E, frame_pair.inlier_num,
                              frame_pair.inlier_mask);
      const int inlier_threshold = std::max(min_num_inlier, (int)(min_ratio_inlier * frame_pair.matches.size()));

      printf("id1:%d id2:%d %d %zu\n", id_candidat, id, frame_pair.inlier_num, frame_pair.matches.size());

      if (id_candidat < id - 1 && frame_pair.inlier_num < inlier_threshold) break;
      frame_pairs.emplace_back(frame_pair);

      //            if(id_candidat==id-1)
      //                DrawFeatureMatches(images[id_candidat], images[id],
      //                frame_candidat.keypoints_, frame.keypoints_,
      //                                   frame_pair.matches,
      //                                   frame_pair.inlier_mask);
    }
  }
}

// void SolveEssentialFromMatch(const Frame &frame1, const Frame &frame2,
// FramePair &frame_pair) {
//    const auto &matches = frame_pair.matches;
//    std::vector<Eigen::Vector2d> matched_points1_normalized(matches.size());
//    std::vector<Eigen::Vector2d> matched_points2_normalized(matches.size());
//    for (size_t i = 0; i < matches.size(); ++i) {
//        matched_points1_normalized[i] =
//        frame1.normalized_points(matches[i].id1);
//        matched_points2_normalized[i] =
//        frame2.normalized_points(matches[i].id2);
//    }
//    //    auto start = clock();
//    itslam::solve_essential(matched_points1_normalized,
//    matched_points2_normalized,
//                            3.0 / 500, frame_pair.E, frame_pair.inlier_num,
//                            frame_pair.inlier_mask);
//    //    auto end = clock();
//    //    std::cout << "Time: " << (double) (end - start) / CLOCKS_PER_SEC <<
//    "s" << std::endl;
//}
