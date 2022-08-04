#ifndef MAP_H
#define MAP_H

#include <opencv2/opencv.hpp>

#include "types.h"
#include "camera.hpp"


namespace xrsfm{
constexpr int MIN_OBS_NUM_LEVEL = 20;

class Track {
 public:
  // Keyframes observing the point and associated index in keyframe
  std::map<int, int> observations_;
  Eigen::Vector3d point3d_;
  
  double angle_;
  double error;

  int ref_id;
  double depth;
  int hierarchical_level = -1;

  bool outlier = false;
  bool is_keypoint = false;
};

class Frame {
 public:

  double timestamp = 0;
  std::string name;
  uint32_t id = -1;
  uint32_t camera_id = 0;

  bool registered = false;
  bool registered_fail = false;
  bool is_keyframe = false;

  // feature extract
  std::vector<cv::KeyPoint> keypoints_;
  FeatureDescriptors sift_descs_;
  UINT8Descriptors uint_descs_;
  cv::Mat orb_descs_;

  // reconstruction
  std::vector<Eigen::Vector2d> points;
  std::vector<Eigen::Vector2d> points_normalized;
  std::vector<int> track_ids_;
  Pose Tcw, tcw_old;

  int num_visible_points3D_ = 0;
  std::vector<int> num_correspondences_have_point3D_;

  int ref_id = -1;
  int id_colmap = -1;
  bool flag_for_view = false;
  int num_neighbors_registered = 0;

  int hierarchical_level = -1;

  inline void AddNumCorHavePoint3D(int p2d_id) {
    if (num_correspondences_have_point3D_[p2d_id] == 0) num_visible_points3D_++;
    num_correspondences_have_point3D_[p2d_id]++;
  }

  inline void DeleteNumCorHavePoint3D(int p2d_id) {
    num_correspondences_have_point3D_[p2d_id]--;
    if (num_correspondences_have_point3D_[p2d_id] == 0) num_visible_points3D_--;
  }

  inline Eigen::Vector3d center() const { return -(Tcw.q.inverse() * Tcw.t); }

  inline Eigen::Quaterniond qwc() const { return Tcw.q.inverse(); }

  inline Eigen::Vector3d twc() const { return -(Tcw.q.inverse() * Tcw.t); }
};

class FramePair {
 public:
  explicit FramePair(int _id1 = 0, int _id2 = 0) : id1(_id1), id2(_id2) {
    inlier_num = inlier_num_f = 0;
    matches.resize(0);
  }

  int id1;
  int id2;
  std::vector<Match> matches;
  // inlier for essential
  int inlier_num, inlier_num_f;
  std::vector<char> inlier_mask, inlier_mask_f;
  Eigen::Matrix3d E, F;

  Eigen::Matrix3d R;
  Eigen::Vector3d t;
  double median_tri_angle;
};

class CorrespondenceGraph {
 public:
  struct CorrNode {
    int num_observations = 0;
    int num_correspondences = 0;
    int num_visible_point3d = 0;
    std::vector<std::vector<std::pair<int, int>>> corrs_vector;  //<frame,feature>
  };

  std::vector<CorrNode> frame_node_vec_;

  int GetMatch(int frame_id1, int frame_id2, std::vector<Match> &matches);
};

class Map {
 public:
  std::vector<cv::Mat> images_;  // for debug

  std::vector<Camera> cameras_;
  std::vector<Track> tracks_;
  std::vector<Frame> frames_;
  std::vector<FramePair> frame_pairs_;

  std::map<int, Frame> frame_map_;
  std::map<int, Track> track_map_;

  // for build
  std::unordered_map<int, int> frameid2pairid_;
  std::unordered_map<int, std::vector<int>> frameid2framepairids_;
  std::unordered_map<int, std::vector<int>> frameid2matched_frameids_;
  std::unordered_map<int, std::vector<int>> frameid2covisible_frameids_;
  CorrespondenceGraph corr_graph_;
  Frame tmp_frame;
  int init_id1 = -1;
  int init_id2 = -1;
  int current_frame_id_ = -1;
  double sre_key_ = 0;
  double avg_track_length_ = 0; 

  void Init();

  void RemoveRedundancyPoints();

  void RemoveRedundancyPoints(Map &tmp);

  void DeregistrationFrame(int frame_id);

  void SearchCorrespondences(const Frame &frame, std::vector<Eigen::Vector2d> &points2d,
                             std::vector<Eigen::Vector3d> &points3d, std::vector<std::pair<int, int>> &cor_2d_3d_ids,
                             const bool use_p2d_normalized);

  void SearchCorrespondences1(const Frame &frame, const std::set<int> cor_frame_id,
                              std::vector<Eigen::Vector2d> &points2d, std::vector<Eigen::Vector3d> &points3d,
                              std::vector<std::pair<int, int>> &cor_2d_3d_ids, const bool use_p2d_normalized);

  void SearchCorrespondencesOrder(const Frame &frame, std::vector<Eigen::Vector2d> &points2d,
                                  std::vector<Eigen::Vector3d> &points3d,
                                  std::vector<std::pair<int, int>> &cor_2d_3d_ids);

  int MaxPoint3dFrameId();
  int MaxPoint3dFrameId1();
  int get_num_p3d(const int frame_id);
  std::pair<int, int> MaxPoint3dFrameIdSeq();

  void LogReprojectError();
  void LogFrameReprojectError();
  void LogFrameReprojectError1(int frame_id);

  inline void AddNumCorHavePoint3D(int frame_id, int p2d_id) {
    for (const auto &[t_frame_id, t_p2d_id] : corr_graph_.frame_node_vec_[frame_id].corrs_vector[p2d_id]) {
      frames_[t_frame_id].AddNumCorHavePoint3D(t_p2d_id);
    }
  }
  inline void DeleteNumCorHavePoint3D(int frame_id, int p2d_id) {
    for (const auto &[t_frame_id, t_p2d_id] : corr_graph_.frame_node_vec_[frame_id].corrs_vector[p2d_id]) {
      frames_[t_frame_id].DeleteNumCorHavePoint3D(t_p2d_id);
    }
  }
};

struct LoopInfo {
  int frame_id;
  std::vector<std::set<int>> cor_frame_ids_vec;
  std::vector<Pose> twc_vec;
  std::vector<int> num_inlier_vec;
  double scale_obs = -1;
};

FramePair FindPair(const std::vector<FramePair> &frame_pairs, const int id1, const int id2);

bool FindPair(const std::vector<FramePair> &frame_pairs, const int id1, const int id2, FramePair &frame_pair);

bool UpdateCovisiblity(Map &map, int frame_id);

void KeyFrameSelection(Map &map, std::vector<int> loop_matched_frame_id, const bool is_sequential_data = false);

void UpdateByRefFrame(Map &map);

bool CheckMeaNumber(Map &map, int frame_id);
}
#endif  // MAP_H