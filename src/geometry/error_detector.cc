#include "geometry/error_corrector.h"

bool ErrorDetector::IsGoodRelativePose(const Map &map, const FramePair &fp, std::vector<char> &inlier_mask) { 
  // if (fp.id1 == 294 && fp.id2 == 295) return false;  // seq 06
  // if (fp.id1 <900 &&fp.id2>600) return false;  // seq 07

  constexpr int num_min_matches = 100;// 100 for seq 06
  constexpr double ratio_th = 0.8;
  constexpr double pure_rotation_th = 0.01;
  constexpr double sin_th = std::sin(2.0 * M_PI / 180);
  constexpr double cos_th = std::cos(2.0 * M_PI / 180);

  std::cout<<fp.id1<< " "<<fp.id2<<" "<<fp.matches.size()<<std::endl;
  if ((std::abs(fp.id1 - fp.id2) != 1) && fp.matches.size() < num_min_matches) return true;

  const auto &frame1 = map.frames_[fp.id1], &frame2 = map.frames_[fp.id2];
  const Eigen::Vector3d relative_motion = frame2.center() - frame1.center();
  const double distance = relative_motion.norm();
  const bool is_pure_rotation = distance < pure_rotation_th;
  if (is_pure_rotation) printf("pure rotation, distance: %lf\n", distance);
  const Eigen::Vector3d t12 = relative_motion.normalized();

  inlier_mask.clear();
  int num_matches = 0, num_inliers = 0;
  for (int i = 0; i < fp.matches.size(); ++i) {
    if (!fp.inlier_mask[i]) continue;
    num_matches++;

    bool good_relative_pose = true;
    if (is_pure_rotation) {  // error
      // double cos_theta = ray1.dot(ray2);
      // good_essential = cos_theta > cos_th;
    } else {
      const Eigen::Vector2d p2d1 = frame1.points_normalized[fp.matches[i].id1];
      const Eigen::Vector2d p2d2 = frame2.points_normalized[fp.matches[i].id2];
      const Eigen::Vector3d ray1 = (frame1.qwc() * p2d1.homogeneous()).normalized();
      const Eigen::Vector3d ray2 = (frame2.qwc() * p2d2.homogeneous()).normalized();

      const bool use_ray2 = std::abs(ray1.dot(t12)) > std::abs(ray2.dot(t12));
      // compute deg between ray2 and plane_ray1-t
      const Eigen::Vector3d n = (ray2.cross(t12)).normalized();  // normal line of plane_ray1-t
      const double sin_theta = std::abs(n.dot(ray1));
      const Eigen::Vector3d n1 = (ray1.cross(t12)).normalized();
      const double sin_theta1 = std::abs(n1.dot(ray2));

      good_relative_pose = use_ray2 ? sin_theta < sin_th : sin_theta1 < sin_th;

      if (ray1.dot(ray2) < 0 && ray1.dot(t12) < 0) good_relative_pose = false;
      if (ray1.dot(t12) < 0 && ray2.dot(t12) > ray1.dot(t12) + sin_th) good_relative_pose = false;
      if (ray2.dot(t12) > 0 && ray2.dot(t12) > ray1.dot(t12) + sin_th) good_relative_pose = false;

      if (good_relative_pose) {
        const double deg_r1_r2 = ToDeg(acos(ray1.dot(ray2)));
        if (deg_r1_r2 > 1.0) {
          const double cos_1 = ray1.dot(t12);
          const double cos_2 = ray2.dot(t12);
          // std::cout<<cos_1<<" "<<cos_2<<std::endl;
          if (abs(cos_1) > 1e-5 && abs(cos_2) > 1e-5) {
            const double t1 = acos(cos_1);
            const double t2 = acos(cos_2);
            if (abs(1.0 / tan(t1) - 1.0 / tan(t2)) < 1e-5) good_relative_pose = false;
            const double h = distance / abs(1.0 / tan(t1) - 1.0 / tan(t2));
            if (h > 200 * sin(t1) || h > 200 * sin(t2)) good_relative_pose = false;
          }
        }
      }
    }
    if (good_relative_pose) num_inliers++;
    inlier_mask.emplace_back(good_relative_pose);
  }

  const double ratio = 1.0 * num_inliers / num_matches;
  printf("%d %d %lf %d/%d\n", frame1.id, frame2.id, ratio, num_inliers, num_matches);

  if (ratio < ratio_th) {
    // std::cout << frame1.tcw.q.coeffs().transpose() << " " << frame1.tcw.t.transpose() << std::endl;nn

    return false;
  }
  return true;
}

void ErrorDetector::IsGoodRelativePose_Debug(Map &map, FramePair &fp, const std::vector<char> &inlier_mask) {
  auto &frame1 = map.frames_[fp.id1];
  auto &frame2 = map.frames_[fp.id2];
  cv::Mat image1 = cv::imread(image_dir_ + frame1.name);
  cv::Mat image2 = cv::imread(image_dir_ + frame2.name);
  if (image1.empty() || image2.empty()) {
    printf("%s\n", (image_dir_ + frame1.name).c_str());
  } else {
    DrawFeatureMatches(image1, image2, frame1.points, frame2.points, fp.matches, inlier_mask);
    DrawFeatureMatches1(image1, frame1.points, frame2.points, fp.matches, inlier_mask);
  }
  // frame1.flag_for_view = frame2.flag_for_view = true;
  // viewer_->Draw(map, true);
  // frame1.flag_for_view = frame2.flag_for_view = false;
  frame1.flag_for_view = frame2.flag_for_view = true;
  viewerTh_->update_map(map);
  frame1.flag_for_view = frame2.flag_for_view = false;
  cv::waitKey();
}

bool ErrorDetector::CheckAllRelativePose(Map &map, int frame_id, std::set<int> &bad_matched_frame_ids) {
  bad_matched_frame_ids.clear();
  const auto &frame = map.frames_[frame_id];
  std::map<int, int> id2num_covisible_obs;
  for (const auto &track_id : frame.track_ids_) {
    if (track_id == -1) continue;
    const auto &track = map.tracks_[track_id];
    for (const auto &[t_frame_id, t_p2d_id] : track.observations_) {
      if (id2num_covisible_obs.count(t_frame_id) == 0) {
        id2num_covisible_obs[t_frame_id] = 1;
      } else {
        id2num_covisible_obs[t_frame_id]++;
      }
    }
  }

  int num_good = 0, num_all = 0;
  for (const auto id : map.frameid2framepairids_[frame_id]) {
    auto &fp = map.frame_pairs_[id];
    int num_covise = -1;
    if (fp.id1 == frame_id) {
      num_covise = id2num_covisible_obs[fp.id2];
    } else {
      num_covise = id2num_covisible_obs[fp.id1];
    }
    if (num_covise >= 10) continue;

    const auto &frame1 = map.frames_[fp.id1];
    const auto &frame2 = map.frames_[fp.id2];
    // std::cout<<fp.id1<< " "<<fp.id2<<" "<<num_covise<<" "<<fp.matches.size()<<std::endl;
    if (frame1.registered && frame2.registered){// && frame1.is_keyframe && frame2.is_keyframe) {
    // std::cout<<fp.id1<< " 1 "<<fp.id2<<" "<<num_covise<<" "<<fp.matches.size()<<std::endl; bad key frame selection
      ++num_all;
      std::vector<char> inlier_mask;
      if (IsGoodRelativePose(map, fp, inlier_mask)) {
        ++num_good;
      } else {
        int bad_neighbor_id = fp.id1 == frame_id ? fp.id2 : fp.id1;
        bad_matched_frame_ids.insert(bad_neighbor_id);
        // if (debug_) IsGoodRelativePose_Debug(map, fp, inlier_mask);
      }
    }
  }

  if (num_good < num_all) {
    LOG(WARNING) << "Bad Relative Pose\n";
    return false;
  }
  return true;
}

void ErrorDetector::StoreRelativePose(Map &map, int frame_id, std::ofstream &file) {
  for (const auto id : map.frameid2framepairids_[frame_id]) {
    auto &fp = map.frame_pairs_[id];
    const auto &frame1 = map.frames_[fp.id1];
    const auto &frame2 = map.frames_[fp.id2];
    if (frame1.registered && frame2.registered && frame1.is_keyframe && frame2.is_keyframe) {
      file << id << std::endl;
      file << frame1.Tcw.q.coeffs().transpose() << " " << frame1.Tcw.t.transpose() << std::endl;
      file << frame2.Tcw.q.coeffs().transpose() << " " << frame2.Tcw.t.transpose() << std::endl;
    }
  }
  file << "-1" << std::endl;
}
