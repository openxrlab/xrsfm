#include "geometry/error_corrector.h"

#include "geometry/pnp.h"

inline std::vector<int> GetMatchedFrameIds(Map &map, int frame_id) {
  std::vector<int> matched_frame_ids;
  for (const int &id : map.frameid2matched_frameids_[frame_id]) {
    if (!map.frames_[id].registered) continue;
    matched_frame_ids.emplace_back(id);
  }
  matched_frame_ids.emplace_back(frame_id);
  return matched_frame_ids;
}

std::vector<std::set<int>> DivideMatchedFrames(Map &map, Frame &frame, Frame &frame2) {
  const std::vector<int> matched_frame_ids = GetMatchedFrameIds(map, frame.id);

  std::map<int, int> id2num_covisible1, id2num_covisible2;
  for (auto &id : matched_frame_ids)
    if (id != frame.id) id2num_covisible1[id] = id2num_covisible2[id] = 0;
  for (auto &track_id : frame.track_ids_) {
    if (track_id == -1) continue;
    for (auto &[t_frame_id, t_p2d_id] : map.tracks_[track_id].observations_) {
      if (id2num_covisible1.count(t_frame_id) != 0) {
        id2num_covisible1[t_frame_id]++;
      }
    }
  }
  for (auto &track_id : frame2.track_ids_) {
    if (track_id == -1) continue;
    for (auto &[t_frame_id, t_p2d_id] : map.tracks_[track_id].observations_) {
      if (id2num_covisible2.count(t_frame_id) != 0) {
        id2num_covisible2[t_frame_id]++;
      }
    }
  }

  std::vector<std::set<int>> cor_frame_ids_vec(2);
  for (auto &id : matched_frame_ids) {
    if (id != frame.id) {
      // printf("%d %d %d\n", id, tmp_set[id], tmp_set1[id]);
      if (id2num_covisible1[id] > id2num_covisible2[id]) {
        cor_frame_ids_vec[0].insert(id);
        printf("0 %d\n", id);
      } else if (id2num_covisible2[id] > id2num_covisible1[id]) {
        cor_frame_ids_vec[1].insert(id);
        printf("1 %d\n", id);
      }
    }
  }
  // CHECK(cor_frame_ids_vec[0].size() > 0);
  // CHECK(cor_frame_ids_vec[1].size() > 0);
  return cor_frame_ids_vec;
}

LoopInfo GetLoopInfo(Map &map, Frame &frame1, Frame &frame2) {
  LoopInfo loop_info;
  loop_info.frame_id = frame1.id;
  loop_info.twc_vec = {frame1.Tcw.inverse(), frame2.Tcw.inverse()};
  loop_info.cor_frame_ids_vec = DivideMatchedFrames(map, frame1, frame2);
 
  // TODO It is a simple method to get scale observation
  int count = 0;
  double depth1 = 0, depth2 = 0;
  for (size_t i = 0; i < frame1.track_ids_.size(); ++i) {
    const int track_id1 = frame1.track_ids_[i], track_id2 = frame2.track_ids_[i];
    if (track_id1 == -1 || track_id2 == -1) continue;
    const auto &track1 = map.tracks_[track_id1], &track2 = map.tracks_[track_id2];

    Eigen::Vector3d p3d1 = frame1.Tcw.q * track1.point3d_ + frame1.Tcw.t;
    Eigen::Vector3d p3d2 = frame2.Tcw.q * track2.point3d_ + frame2.Tcw.t;
    depth1 += p3d1.z();
    depth2 += p3d2.z();
    count++;
  }
  if (count >= 4) {
    loop_info.scale_obs = depth2 / depth1;
  }
  printf("Set SCALE: %lf = %lf / %lf %d\n", loop_info.scale_obs, depth2, depth1, count);
  return loop_info;
}

bool CheckNegtiveDepth(const Map &map, const Frame &frame1, const Frame &frame2) {
  for (auto &track_id : frame1.track_ids_) {
    if (track_id == -1) continue;
    auto &track = map.tracks_[track_id];
    const Eigen::Vector3d p3d = frame2.Tcw.q * track.point3d_ + frame2.Tcw.t;
    if (p3d.z() < 0) {
      return true;
    }
  }
  for (auto &track_id : frame2.track_ids_) {
    if (track_id == -1) continue;
    auto &track = map.tracks_[track_id];
    const Eigen::Vector3d p3d = frame1.Tcw.q * track.point3d_ + frame1.Tcw.t;
    if (p3d.z() < 0) {
      return true;
    }
  }
  return false;
}

inline bool TryLocate(Map &map, const int frame_id, const std::set<int> &local_frame_ids,
                      Point3dProcessor *p3d_processor_) {
  bool reg_success = RegisterNextImageLocal(frame_id, local_frame_ids, map);

  if (!reg_success) {
    bool have_neighbor = false;
    std::vector<int> adjacent_frame_ids = {frame_id - 1, frame_id + 1};
    for (auto &id : adjacent_frame_ids) {
      if (local_frame_ids.count(id) != 0) {
        map.frames_[frame_id].registered = false;
        p3d_processor_->TriangulateFramePoint(map, id, p3d_processor_->th_rpe_lba_);
        map.frames_[frame_id].registered = true;
        have_neighbor = true;
      }
    }
    if (have_neighbor) reg_success = RegisterNextImageLocal(frame_id, local_frame_ids, map);
  }
  return reg_success;
}

inline void MergeTrackLoop(Map &map, Frame &frame1, Frame &frame2) {
  for (size_t i = 0; i < frame1.track_ids_.size(); ++i) {
    const int track_id = frame2.track_ids_[i];  // not add num cor have point 3d
    if (track_id == -1) continue;
    auto &track = map.tracks_[track_id];
    if (track.observations_.count(frame1.id) != 0) continue;  // may compare

    Eigen::Vector3d p3d = frame2.Tcw.q * track.point3d_ + frame2.Tcw.t;
    Eigen::Vector3d p3d1 = p3d.z() * frame1.points_normalized[i].homogeneous();
    Eigen::Vector3d p3d2 = frame1.Tcw.q.inverse() * (p3d1 - frame1.Tcw.t);

    bool merge = false;
    const int track_id1 = frame1.track_ids_[i];
    if (track_id1 != -1) {  // merge
      auto &track1 = map.tracks_[track_id1];
      // track.point3d_ = (track.point3d_ + track1.point3d_) / 2;
      for (const auto &[t_frame_id, t_p2d_id] : track1.observations_) {
        if (track.observations_.count(t_frame_id) == 0) {  // if track have t_frame_id obs, it map injective injective
          track.observations_[t_frame_id] = t_p2d_id;
          map.frames_[t_frame_id].track_ids_[t_p2d_id] = track_id;
        } else {
          map.frames_[t_frame_id].track_ids_[t_p2d_id] = -1;
          map.DeleteNumCorHavePoint3D(t_frame_id, t_p2d_id);
        }
      }
      track1.outlier = true;
      merge = true;
    }

    if (!merge) {
      // track.point3d_ = (track.point3d_ + p3d2) / 2;
      frame1.track_ids_[i] = track_id;
      track.observations_[frame1.id] = i;
      map.AddNumCorHavePoint3D(frame1.id, i);
    }
    // {
    //   printf("%d %d %d\n", track_id, track_id1, frame1.track_ids_[i]);
    //   for (const auto &[t_frame_id, t_p2d_id] : track.observations_) {
    //     printf("%d %d -- ", t_frame_id, t_p2d_id);
    //     auto &frame = map.frames_[t_frame_id];
    //     Eigen::Vector3d p3d = frame.tcw.q * track.point3d_ + frame.tcw.t;
    //     std::cout << p3d.transpose() << "|" << (p3d.head<2>() / p3d.z()).transpose()
    //               << frame.points_normalized[t_p2d_id].transpose() << std::endl;
    //   }
    // }
  }
}

bool ErrorCorrector::CheckAndCorrectPose(Map &map, int frame_id, int iter) {
  std::set<int> bad_matched_frame_ids;
  if (error_detector.CheckAllRelativePose(map, frame_id, bad_matched_frame_ids)) return false;
  if (!TryLocate(map, frame_id, bad_matched_frame_ids, p3d_processor_)) return false;

  auto &frame = map.frames_[frame_id];
  const std::vector<int> matched_frame_ids = GetMatchedFrameIds(map, frame_id);
  for (auto &id : matched_frame_ids) {
    std::cout << "|" << id << std::endl;
  }

  // error_detector.viewerTh_->update_map(map);
  // cv::Mat img(10,10,CV_8U);
  // cv::imshow("",img); 
  // cv::waitKey();

  // PoseGraph
  const double dist = (frame.Tcw.center() - map.tmp_frame.center()).norm();
  const bool negtive_depth = CheckNegtiveDepth(map, map.tmp_frame, frame);
  std::cout << "DIST:  " << dist << std::endl;
  if (dist > 1.5 || negtive_depth) {
    KeyFrameSelection(map, matched_frame_ids, true);
    UpdateByRefFrame(map);
    LoopInfo loop_info = GetLoopInfo(map, frame, map.tmp_frame);
    if (loop_info.cor_frame_ids_vec[0].size() == 0 || loop_info.cor_frame_ids_vec[1].size() == 0) return false;
    if (only_correct_with_sim3_ && loop_info.scale_obs == -1) return false; 
    ba_solver_->ScalePoseGraphUnorder(loop_info, map, true);
  }
  
  // error_detector.viewerTh_->update_map(map); 
  // cv::imshow("",img); 
  // cv::waitKey();


  MergeTrackLoop(map, frame, map.tmp_frame);

  // BA
  if (dist > 0.02) {
    printf("size: %d : ", matched_frame_ids.size());
    for (auto &id : matched_frame_ids) {
      printf("%d ", id);
    }
    printf("\n");
    ba_solver_->KGBA(map, matched_frame_ids, true);
    p3d_processor_->FilterPoints3d(map, p3d_processor_->th_rpe_gba_, p3d_processor_->th_angle_gba_);
    // TODO num_image_reg_pre = num_image_reg;
  }

  error_detector.CheckAllRelativePose(map, frame_id, bad_matched_frame_ids);
  map.tmp_frame.registered = false;

  return true;
}
