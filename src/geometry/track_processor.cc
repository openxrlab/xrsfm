//
// Created by SENSETIME\yezhichao1 on 2020/10/26.
//

#include "track_processor.h"

#include "geometry/colmap/estimators/triangulation.h"
#include "geometry/triangluate_svd.h"
#include "base/camera.h"

double ReprojectionError(const Pose &pose, const Eigen::Vector2d &point2d, const Eigen::Vector3d &point3d) {
  Eigen::Vector3d p_c = pose.q * point3d + pose.t;
  Eigen::Vector2d residual = p_c.hnormalized() - point2d;
  return residual.norm();
}

double Reprojection_Error(const Pose &pose, const Camera &camera, const Eigen::Vector2d &point2d,
                          const Eigen::Vector3d &point3d) {
  Eigen::Vector3d p_c = pose.q * point3d + pose.t;
  Eigen::Vector2d estimate;
  NormalizedToImage(camera, p_c.hnormalized(), estimate);
  Eigen::Vector2d residual = estimate - point2d;
  return residual.norm();
}

void AddTrack(const std::vector<std::pair<int, int>> &observations, const Eigen::Vector3d &p, Map &map) {
  Track track;
  track.point3d_ = p;
  track.outlier = false;
  const int track_id = static_cast<int>(map.tracks_.size());

  int min_level1 = -1, min_level2 = -1;
  for (const auto &[frame_id, p2d_id] : observations) {
    track.observations_[frame_id] = p2d_id;
    map.frames_[frame_id].track_ids_[p2d_id] = track_id;

    int level = map.frames_[frame_id].hierarchical_level;
    if (level == -1) continue;

    if (min_level1 == -1) {
      min_level1 = level;
    } else if (min_level2 == -1 || level < min_level2) {
      if (level < min_level1) {
        min_level2 = min_level1;
        min_level1 = level;
      } else {
        min_level2 = level;
      }
    }
  }
  // LOG_IF(WARNING, min_level2 == -1) << "Track level not init";
  track.hierarchical_level = std::max(min_level2, 0);

  map.tracks_.emplace_back(track);

  for (const auto &[t_frame_id, t_p2d_id] : observations) {
    map.AddNumCorHavePoint3D(t_frame_id, t_p2d_id);
  }
}

void ContinueTrackUpdate(const int track_id, const int frame_id, const int p2d_id, Map &map) {
  auto &track = map.tracks_[track_id];
  track.observations_[frame_id] = p2d_id;

  auto &frame = map.frames_[frame_id];
  frame.track_ids_[p2d_id] = track_id;

  map.AddNumCorHavePoint3D(frame_id, p2d_id);
}

void SetTrackOutlier(Map &map, Track &track) {
  track.outlier = true;
  for (const auto &[t_frame_id, t_p2d_id] : track.observations_) {
    map.frames_[t_frame_id].track_ids_[t_p2d_id] = -1;
    map.DeleteNumCorHavePoint3D(t_frame_id, t_p2d_id);
  }
}

bool CreatePoint3d(const std::vector<std::pair<int, int>> observations, Map &map) {
  std::vector<itslam::matrix<3, 4>> Ps;
  std::vector<itslam::vector<2>> ps;
  // Prepare the data
  for (auto &[t_frame_id, t_p2d_id] : observations) {
    Frame &t_frame = map.frames_[t_frame_id];
    itslam::matrix<3, 3> R = t_frame.Tcw.q.toRotationMatrix();
    itslam::vector<3> T = t_frame.Tcw.t;
    itslam::matrix<3, 4> P;
    P << R, T;
    Ps.push_back(P);
    ps.push_back(t_frame.points_normalized[t_p2d_id]);
  }
  //  make new track;
  itslam::vector<3> p;
  if (triangulate_point_checked(Ps, ps, p)) {
    AddTrack(observations, p, map);
    return true;
  }
  return false;
}

bool CreatePoint3d1(const std::vector<std::pair<int, int>> observations, Map &map) {
  size_t data_size = observations.size();
  std::vector<colmap::TriangulationEstimator::PointData> point_data(data_size);
  std::vector<colmap::TriangulationEstimator::PoseData> pose_data(data_size);
  // Prepare the data
  for (size_t i = 0; i < data_size; ++i) {
    const auto &[t_frame_id, t_p2d_id] = observations[i];
    Frame &t_frame = map.frames_[t_frame_id];
    assert(t_frame.registered);
    itslam::matrix<3, 3> R = t_frame.Tcw.q.toRotationMatrix();
    itslam::vector<3> T = t_frame.Tcw.t;
    itslam::matrix<3, 4> P;
    P << R, T;

    point_data[i].point_normalized = t_frame.points_normalized[t_p2d_id];
    pose_data[i].proj_matrix = P;
    pose_data[i].proj_center = t_frame.center();
  }

  colmap::EstimateTriangulationOptions tri_options;
  tri_options.min_tri_angle = DegToRad(1.5);
  tri_options.residual_type = colmap::TriangulationEstimator::ResidualType::ANGULAR_ERROR;
  tri_options.ransac_options.max_error = DegToRad(2);
  tri_options.ransac_options.confidence = 0.9999;
  tri_options.ransac_options.min_inlier_ratio = 0.02;
  tri_options.ransac_options.max_num_trials = 10000;
  // Enforce exhaustive sampling for small track lengths.
  const size_t kExhaustiveSamplingThreshold = 15;
  if (point_data.size() <= kExhaustiveSamplingThreshold) {
    tri_options.ransac_options.min_num_trials = NChooseK(point_data.size(), 2);
  }

  //  make new track;
  itslam::vector<3> p;
  std::vector<char> inlier_mask;
  if (EstimateTriangulation(tri_options, point_data, pose_data, &inlier_mask, &p)) {
    std::vector<std::pair<int, int>> inlier_observations;
    for (int k = 0; k < observations.size(); ++k) {
      if (inlier_mask[k]) {
        inlier_observations.push_back(observations[k]);
      }
    }
    AddTrack(inlier_observations, p, map);
    return true;
  }
  return false;
}

inline std::tuple<int,double> GetMaxAngle(const Map &map,const std::set<int> &observed_track_ids,const int frame_id,const int p2d_id){
    int best_track_id = -1;
    double best_angle_error = 100; 
    const auto&frame = map.frames_.at(frame_id);
    for (const auto &track_id : observed_track_ids) { 
        const auto &track = map.tracks_[track_id];
        if (track.observations_.count(frame_id) != 0) continue;
        Eigen::Vector3d p3d = track.point3d_;
        Eigen::Vector3d ray1 = (frame.Tcw.q * p3d + frame.Tcw.t).normalized();
        Eigen::Vector3d ray2 = (frame.points_normalized[p2d_id].homogeneous()).normalized();
        const double angle_error = std::acos(ray1.dot(ray2));
        if (angle_error < best_angle_error) { 
          best_track_id = track_id;
          best_angle_error = angle_error;
        } 
    }
    return std::tuple<int,double>(best_track_id,best_angle_error);
}

int Point3dProcessor::TriangulateFramePoint(Map &map, const int frame_id, double deg_th) {
  const double const_angle_th = DegToRad(deg_th);

  int num_create = 0, num_extend = 0;
  int num_zero_track = 0, num_one_track = 0, num_multi_track = 0;

  Frame &frame = map.frames_[frame_id];
  const auto &corrs_vector = map.corr_graph_.frame_node_vec_[frame_id].corrs_vector;
  assert(frame.track_ids_.size()==corrs_vector.size()); 

  for (int p2d_id = 0,num_p2d =  frame.track_ids_.size(); p2d_id < num_p2d; ++p2d_id) {
    if (frame.track_ids_.at(p2d_id) != -1) continue;  // only triangulate un-trigulated points   

    std::set<int> observed_track_ids;
    std::vector<std::pair<int, int>> observations;
    for (const auto &[t_frame_id, t_p2d_id] : corrs_vector.at(p2d_id)) { 
      const auto& cor_frame = map.frames_.at(t_frame_id);
      if (!cor_frame.registered) continue;
      if(cor_frame.track_ids_.size()<=t_p2d_id){
        std::cout<<"bad"<<t_frame_id<<" "<<t_p2d_id<<std::endl;
        exit(0);
      }
      observations.emplace_back(t_frame_id, t_p2d_id);
      const int trakc_id = cor_frame.track_ids_.at(t_p2d_id);
      if (trakc_id != -1) {
        observed_track_ids.insert(trakc_id);
      }
    }
    observations.emplace_back(std::pair<int, int>(frame_id, p2d_id));
    if (observations.size() < 2) continue;


    const int num_track = observed_track_ids.size();
    
    // std::cout<<frame.id<<" 2 "<<p2d_id<<" "<<num_track<<std::endl;

    if (num_track == 0) {
      num_zero_track++;
      if (CreatePoint3d1(observations, map)) num_create++;
    } else if (num_track >= 1) {  
      auto [best_track_id,best_angle_error] = GetMaxAngle(map,observed_track_ids,frame_id,p2d_id); 
      if (best_angle_error < const_angle_th) {
        ContinueTrackUpdate(best_track_id, frame_id, p2d_id, map);
        num_extend++;
      }

      if (num_track == 1) {
        num_one_track++;
      } else {
        num_multi_track++;
      }
    }
  }
  printf("Tri %d %d/%d %d/%d+%d\n", frame_id, num_create, num_zero_track, num_extend, num_one_track, num_multi_track);
  return num_create + num_extend;
}

inline void UpdateTrackAngle(const std::vector<Frame> &frames, Track &track, const double min_tri_angle_rad) {
  std::vector<Eigen::Vector3d> centers(0);
  for (auto &obs_info : track.observations_) {
    const auto &frame = frames[obs_info.first];
    assert(frame.registered);
    centers.push_back(frame.Tcw.center());
  }

  double max_tri_angle = 0;
  for (int i = 0; i < centers.size(); ++i) {
    for (int j = i + 1; j < centers.size(); ++j) {
      const double tri_angle = CalculateTriangulationAngle(centers[i], centers[j], track.point3d_);
      if (tri_angle > max_tri_angle) {
        max_tri_angle = tri_angle;
        if (max_tri_angle > min_tri_angle_rad) {
          track.angle_ = max_tri_angle;
          return;
        }
      }
    }
  }

  track.angle_ = max_tri_angle;
}

inline void FilterPoint3d(const double max_re, const double min_tri_angle_rad, Map &map, Track &track,
                          int &num_filtered1, int &num_filtered2) {
  double reproj_error_sum = 0.0;
  std::vector<std::pair<int, int>> obs_to_delete;
  obs_to_delete.reserve(8);
  for (auto &[frame_id, p2d_id] : track.observations_) {
    const auto &frame = map.frames_[frame_id];
    // CHECK(frame.registered);
    const auto &camera = map.cameras_[frame.camera_id];
    const double re = Reprojection_Error(frame.Tcw, camera, frame.points[p2d_id], track.point3d_);
    const Eigen::Vector3d p3d = frame.Tcw.q * track.point3d_ + frame.Tcw.t;
    if (re > max_re || p3d.z() < 0.2 || p3d.z() > 100) {
      obs_to_delete.emplace_back(frame_id, p2d_id);
    } else {
      reproj_error_sum += re;
    }
  }

  if (obs_to_delete.size() >= track.observations_.size() - 1) {
    num_filtered1 += track.observations_.size();
    SetTrackOutlier(map, track);
    return;
  } else {
    num_filtered1 += obs_to_delete.size();
    for (const auto &[o_frame_id, o_p2d_id] : obs_to_delete) {
      track.observations_.erase(o_frame_id);
      map.frames_[o_frame_id].track_ids_[o_p2d_id] = -1;
      map.DeleteNumCorHavePoint3D(o_frame_id, o_p2d_id);
    }
    track.error = (reproj_error_sum / track.observations_.size());
  }

  UpdateTrackAngle(map.frames_, track, min_tri_angle_rad);
  if (track.angle_ < min_tri_angle_rad) {
    SetTrackOutlier(map, track);
    num_filtered2++;
  }
}

int Point3dProcessor::FilterPoints3d(Map &map, const double max_re, const double deg) {
  const double min_tri_angle_rad = DegToRad(deg);
  int num_filtered1 = 0, num_filtered2 = 0;
  for (auto &track : map.tracks_) {
    if (!track.outlier) FilterPoint3d(max_re, min_tri_angle_rad, map, track, num_filtered1, num_filtered2);
  }
  printf("Outlier num1: %d Outlier num2: %d\n", num_filtered1, num_filtered2);
  return num_filtered1 + num_filtered2;
}

int Point3dProcessor::FilterPointsFrame(Map &map, const int frame_id, const double max_re, const double deg) {
  const double min_tri_angle_rad = DegToRad(deg);
  const auto &frame = map.frames_[frame_id];

  int num_filtered1 = 0, num_filtered2 = 0;
  for (const auto &track_id : frame.track_ids_) {
    if (track_id == -1) continue;
    auto &track = map.tracks_[track_id];
    FilterPoint3d(max_re, min_tri_angle_rad, map, track, num_filtered1, num_filtered2);
  }
  printf("Outlier num1: %d Outlier num2: %d\n", num_filtered1, num_filtered2);
  return num_filtered1 + num_filtered2;
}

void Point3dProcessor::CheckTrackDepth(const Map &map) {
  for (size_t i = 0; i < map.tracks_.size(); ++i) {
    auto &track = map.tracks_[i];
    if (track.outlier) continue;
    int count = 0;
    for (auto &obs_info : track.observations_) {
      const auto &frame = map.frames_[obs_info.first];
      Eigen::Vector3d p_c = frame.Tcw.q * track.point3d_ + frame.Tcw.t;

      auto &camera = map.cameras_[frame.camera_id];
      double re = Reprojection_Error(frame.Tcw, camera, frame.points[obs_info.second], track.point3d_);
      if (p_c.z() < 0 || p_c.z() > 100) {
        printf("Error:%zu %d %lf %lf\n", i, obs_info.first, p_c.z(), re);
      }
    }
  }
}

void Point3dProcessor::ReTriangulate(Map &map) {
  int count0 = 0, count1 = 0, count2 = 0;
  for (size_t i = 0; i < map.tracks_.size(); ++i) {
    auto &track = map.tracks_[i];
    if (track.outlier) continue;
    count2++;

    bool has_negative_depth = false;
    std::vector<std::pair<int, int>> observations(0);
    for (auto &[t_frame_id, t_p2d_id] : track.observations_) {
      Frame &t_frame = map.frames_[t_frame_id];
      if (!t_frame.registered) continue;
      observations.emplace_back(t_frame_id, t_p2d_id);
      Eigen::Vector3d p_c = t_frame.Tcw.q * track.point3d_ + t_frame.Tcw.t;
      if (p_c.z() < 0) {
        has_negative_depth = true;
        //                printf("%zu %d %lf\n",i,t_frame_id,p_c.z());
      }
    }
    // if (!has_negative_depth) continue;

    std::vector<itslam::matrix<3, 4>> Ps;
    std::vector<itslam::vector<2>> ps;
    // Prepare the data
    for (auto &obs_info : observations) {
      Frame &t_frame = map.frames_[obs_info.first];
      itslam::matrix<3, 3> R = t_frame.Tcw.q.toRotationMatrix();
      itslam::vector<3> T = t_frame.Tcw.t;
      itslam::matrix<3, 4> P;
      P << R, T;
      Ps.push_back(P);
      ps.push_back(t_frame.points_normalized[obs_info.second]);
    }

    //  make new track;
    itslam::vector<3> p;
    std::vector<char> inlier_mask;
    if (triangulate_point_checked(Ps, ps, p)) {
      track.point3d_ = p;
      count0++;
    }
    count1++;
  }

  printf("ReTriangulate %d/%d/%d\n", count0, count1, count2);
}

void Point3dProcessor::ContinueTrack(Map&map,int track_id,double max_re){
    auto&track = map.tracks_[track_id];
    const auto init_obs =  track.observations_;

    std::set<std::pair<int, int>> visted_pair;
    for (const auto &[frame_id, p2d_id] : init_obs) {
      for (const auto &[c_frame_id, c_p2d_id] : map.corr_graph_.frame_node_vec_[frame_id].corrs_vector[p2d_id]) {
        // search connect obs{frame,p2d}
        if (track.observations_.count(c_frame_id) != 0) continue;
        auto &c_frame = map.frames_[c_frame_id];
        if (!c_frame.registered) continue;
        const int track_id2 = c_frame.track_ids_[c_p2d_id];
        std::pair<int, int> pair(c_frame_id, c_p2d_id);
        if (track_id2 != -1 || visted_pair.count(pair) != 0) continue;
        visted_pair.insert(pair);

        // check rpe
        const auto &c_camera = map.cameras_[c_frame.camera_id];
        double re = Reprojection_Error(c_frame.Tcw, c_camera, c_frame.points[c_p2d_id], track.point3d_);
        if (re > max_re) continue;
        ContinueTrackUpdate(track_id, c_frame_id, c_p2d_id, map);
      }
    }
}

void Point3dProcessor::MergeTrack(Map&map,int track_id,double max_re){ 
    auto&track = map.tracks_[track_id];
    const auto init_obs =  track.observations_;
    // merge track
    std::set<int> visted_id;
    for (const auto &[frame_id, p2d_id] : init_obs) {
      for (const auto &[c_frame_id, c_p2d_id] : map.corr_graph_.frame_node_vec_[frame_id].corrs_vector[p2d_id]) {
        // search connected track
        if (track.observations_.count(c_frame_id) != 0) continue;
        auto &frame = map.frames_[c_frame_id];
        if (!frame.registered) continue;
        const int track_id2 = frame.track_ids_[c_p2d_id];
        if (track_id2 == -1 || visted_id.count(track_id2) > 0) continue;
        visted_id.insert(track_id2);
        auto &track2 = map.tracks_[track_id2];

        // judge whether to merge
        bool merge_success = true;
        const double w1 = track.observations_.size(), w2 = track2.observations_.size();
        const Eigen::Vector3d merged_p3d = (w1 * track.point3d_ + w2 * track2.point3d_) / (w1 + w2);
        for (const Track *track_ptr : {&track, &track2}) {
          for (const auto &[t_frame_id, t_p2d_id] : track_ptr->observations_) {
            const auto &t_frame = map.frames_[t_frame_id];
            const auto &t_camera = map.cameras_[t_frame.camera_id];
            double re = Reprojection_Error(t_frame.Tcw, t_camera, t_frame.points[t_p2d_id], merged_p3d);
            if (re > max_re) {
              merge_success = false;
              break;
            }
          }
          if(!merge_success)break;
        }

        // merge two points
        if (merge_success) { 
          track.point3d_ = merged_p3d;
          for (const auto &[t_frame_id, t_p2d_id] : track2.observations_) {
            if (track.observations_.count(t_frame_id) == 0) {
              track.observations_[t_frame_id] = t_p2d_id;
              map.frames_[t_frame_id].track_ids_[t_p2d_id] = track_id;
            } else {
              map.frames_[t_frame_id].track_ids_[t_p2d_id] = -1;
              map.DeleteNumCorHavePoint3D(t_frame_id, t_p2d_id);
            }
          }
          track2.outlier = true;
        }
      }
    } 
}

void Point3dProcessor::MergeTracks(Map &map, const int frame_id, double max_re) {
  int num_merged_track = 0, num_connected_track = 0;
  int num_continue_track = 0, num_connected_meas = 0;
  for (const auto &track_id : map.frames_[frame_id].track_ids_) {
    if (track_id == -1) continue;
    auto &track = map.tracks_[track_id];
    // merge track
    std::set<int> visted_id;
    for (const auto &[_frame_id, p2d_id] : track.observations_) {
      for (const auto &[c_frame_id, c_p2d_id] : map.corr_graph_.frame_node_vec_[_frame_id].corrs_vector[p2d_id]) {
        // search connected track
        if (track.observations_.count(c_frame_id) != 0) continue;
        auto &frame = map.frames_[c_frame_id];
        if (!frame.registered) continue;
        const int track_id2 = frame.track_ids_[c_p2d_id];
        if (track_id2 == -1 || visted_id.count(track_id2) > 0) continue;
        num_connected_track++;
        visted_id.insert(track_id2);
        auto &track2 = map.tracks_[track_id2];

        // judge whether to merge
        const double w1 = track.observations_.size(), w2 = track2.observations_.size();
        const Eigen::Vector3d merged_p3d = (w1 * track.point3d_ + w2 * track2.point3d_) / (w1 + w2);
        bool merge_success = true;
        for (const Track *track_ptr : {&track, &track2}) {
          for (const auto &[t_frame_id, t_p2d_id] : track_ptr->observations_) {
            const auto &t_frame = map.frames_[t_frame_id];
            const auto &t_camera = map.cameras_[t_frame.camera_id];
            double re = Reprojection_Error(t_frame.Tcw, t_camera, t_frame.points[t_p2d_id], merged_p3d);
            if (re > max_re) {
              merge_success = false;
              break;
            }
          }
        }
        // merge two points
        if (merge_success) {
          num_merged_track++;
          track.point3d_ = merged_p3d;
          for (const auto &[t_frame_id, t_p2d_id] : track2.observations_) {
            if (track.observations_.count(t_frame_id) == 0) {
              track.observations_[t_frame_id] = t_p2d_id;
              map.frames_[t_frame_id].track_ids_[t_p2d_id] = track_id;
            } else {
              map.frames_[t_frame_id].track_ids_[t_p2d_id] = -1;
              map.DeleteNumCorHavePoint3D(t_frame_id, t_p2d_id);
            }
          }
          track2.outlier = true;
        }
      }
    }
    // continue track
    std::set<std::pair<int, int>> visted_pair;
    for (const auto &[_frame_id, p2d_id] : track.observations_) {
      for (const auto &[c_frame_id, c_p2d_id] : map.corr_graph_.frame_node_vec_[_frame_id].corrs_vector[p2d_id]) {
        if (track.observations_.count(c_frame_id) != 0) continue;
        auto &frame = map.frames_[c_frame_id];
        if (!frame.registered) continue;
        const int track_id2 = frame.track_ids_[c_p2d_id];
        std::pair<int, int> pair(c_frame_id, c_p2d_id);
        if (track_id2 != -1 || visted_pair.count(pair) != 0) continue;
        num_connected_meas++;
        visted_pair.insert(pair);

        const auto &t_frame = map.frames_[c_frame_id];
        const auto &t_camera = map.cameras_[t_frame.camera_id];
        double re = Reprojection_Error(t_frame.Tcw, t_camera, t_frame.points[c_p2d_id], track.point3d_);
        if (re < max_re) {
          num_continue_track++;
          ContinueTrackUpdate(track_id, c_frame_id, c_p2d_id, map);
        }
      }
    }
  }
  printf("Merge: %d/%d %d/%d \n", num_merged_track, num_connected_track, num_continue_track, num_connected_meas);
}

bool Point3dProcessor::CheckFrameMeasurement(Map &map, int frame_id) {
  bool no_mea = true;
  auto &frame = map.frames_[frame_id];
  for (int i = 0; i < frame.track_ids_.size(); ++i) {
    if (frame.track_ids_[i] == -1) continue;
    no_mea = false;
    break;
  }
  if (no_mea) LOG(ERROR) << "NO Measurement In Frame " << frame.id;
  return no_mea;
}

void Point3dProcessor::CheckFramesMeasurement(Map &map, double th_rpe_lba, double th_angle_lba) {
  for (auto &frame : map.frames_) {
    if (!frame.registered) continue;
    if (!frame.is_keyframe) continue;
    if (CheckFrameMeasurement(map, frame.id)) {
      LOG(ERROR) << "no measurement in KGBA";
      TriangulateFramePoint(map, frame.id, th_angle_lba);
      FilterPointsFrame(map, frame.id, th_rpe_lba, th_angle_lba);
    }
  }
}

void Point3dProcessor::ContinueFrameTracks(const int frame_id, const std::vector<std::pair<int, int>> &cor_2d_3d_ids,
                                           Map &map) {
  auto &frame = map.frames_[frame_id];
  for (const auto &[p2d_id, track_id] : cor_2d_3d_ids) {
    bool skip = false;
    auto &track = map.tracks_[track_id];
    if (track.observations_.count(frame_id) != 0) {
      // two 2D points in an image are associated with the same 3D point
      const int p2d_id1 = track.observations_[frame_id];
      const auto &camera = map.cameras_[frame.camera_id];
      const double re0 = Reprojection_Error(frame.Tcw, camera, frame.points[p2d_id], track.point3d_);
      const double re1 = Reprojection_Error(frame.Tcw, camera, frame.points[p2d_id1], track.point3d_);
      if (re0 < re1) {
        frame.track_ids_[p2d_id1] = -1;
        map.DeleteNumCorHavePoint3D(frame_id, p2d_id1);
      } else {
        skip = true;
      }
    }

    if (!skip) {
      frame.track_ids_[p2d_id] = track_id;
      track.observations_[frame_id] = p2d_id;
      map.AddNumCorHavePoint3D(frame_id, p2d_id);
    }
  }
}