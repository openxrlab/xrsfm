//
// Created by SENSETIME\yezhichao1 on 2020/10/21.
//

#include "ba_solver.h"

#include "cost_factor_ceres.h"
#include "geometry/colmap/base/triangulation.h"
#include "geometry/colmap/util/math.h"
#include "utility/timer.h"

namespace xrsfm{
std::vector<int> num_cov;
double weight_o = 0;

ceres::Solver::Options BASolver::InitSolverOptions() {
  ceres::Solver::Options solver_options;
  solver_options.num_threads = 8;
  solver_options.max_num_iterations = 100;
  solver_options.linear_solver_type = ceres::SPARSE_SCHUR;
  solver_options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
  return solver_options;
}

void AddCovisibilityEdge(ceres::Problem &problem, Map &map, std::vector<double> &s_vec, std::vector<Pose> &twc_vec,
                         bool use_key) {
  num_cov.assign(map.frames_.size(), 0);
  for (auto &frame : map.frames_) {
    if (!frame.registered) continue;
    if (use_key && !frame.is_keyframe) continue;

    auto &s1 = s_vec[frame.id];
    auto &pose1 = twc_vec[frame.id];

    int count = 0;
    for (const auto &cor_id : map.frameid2covisible_frameids_[frame.id]) {
      // assert every key frame have at least a key frame neighbor
      if (frame.id <= cor_id) continue;
      if (use_key && !map.frames_[cor_id].is_keyframe) continue;

      auto &s2 = s_vec[cor_id];
      auto &pose2 = twc_vec[cor_id];
      Eigen::Quaterniond q_mea = pose1.q.inverse() * pose2.q;
      Eigen::Vector3d p_mea = pose1.q.inverse() * (pose2.t - pose1.t);
      ceres::CostFunction *cost_function = new PoseGraph3dCost1(q_mea, p_mea, weight_o);
      problem.AddResidualBlock(cost_function, nullptr, pose1.q.coeffs().data(), pose1.t.data(), pose2.q.coeffs().data(),
                               pose2.t.data(), &s1, &s2);
      count++;
      num_cov[frame.id]++;
      num_cov[cor_id]++;
    }
  }
}

void AddLoopEdge(ceres::Problem &problem, Map &map, const LoopInfo &loop_info, std::vector<double> &s_vec,
                 std::vector<double> &s_vec_loop, std::vector<Pose> &twc_vec, bool use_key) {
  auto &pose1 = twc_vec[loop_info.frame_id];
  for (size_t i = 0; i < loop_info.cor_frame_ids_vec.size(); ++i) {
    auto &s1 = s_vec_loop[i];
    auto &pose1_mea = loop_info.twc_vec[i];
    int count = 0;
    for (const auto &cor_id : loop_info.cor_frame_ids_vec[i]) {
      if (use_key && !map.frames_[cor_id].is_keyframe) continue;
      auto &s2 = s_vec[cor_id];
      auto &pose2 = twc_vec[cor_id];
      Eigen::Quaterniond q_mea = pose1_mea.q.inverse() * pose2.q;
      Eigen::Vector3d p_mea = pose1_mea.q.inverse() * (pose2.t - pose1_mea.t);

      ceres::CostFunction *cost_function = new PoseGraph3dCost1(q_mea, p_mea, weight_o);
      problem.AddResidualBlock(cost_function, nullptr, pose1.q.coeffs().data(), pose1.t.data(), pose2.q.coeffs().data(),
                               pose2.t.data(), &s1, &s2);
      count++;
      num_cov[cor_id]++;
      num_cov[loop_info.frame_id]++;
    }
    printf("loop_cor: %d num_edge: %d\n", i, count);
  }
}

void BASolver::ScalePoseGraphUnorder(const LoopInfo &loop_info, Map &map, bool use_key) {
  // assign the best frame for each map points
  for (auto &track : map.tracks_) {
    if (track.outlier) continue;
    int best_frame_id = -1;
    double best_depth = -1.0;
    bool is_key = false;
    for (const auto &[frame_id, p2d_id] : track.observations_) {
      if (frame_id == loop_info.frame_id) continue;
      const auto &frame = map.frames_[frame_id];
      const double depth = (frame.Tcw.q * track.point3d_ + frame.Tcw.t).z();
      if (best_frame_id == -1) {
        best_frame_id = frame_id;
        best_depth = depth;
        is_key = frame.is_keyframe;
      } else {
        if (depth < 0) continue;

        if ((!is_key && frame.is_keyframe) || (best_depth < 0) ||
            (depth < best_depth && !(is_key && !frame.is_keyframe))) {
          best_frame_id = frame_id;
          best_depth = depth;
          is_key = frame.is_keyframe;
        }
      }
    }
    track.ref_id = best_frame_id;
    track.depth = best_depth;
    if (track.depth < 0) {
      std::cout << "!!! negative depth\n";
      const auto &it = track.observations_.begin();
      const int track_id = map.frames_[it->first].track_ids_[it->second];
      printf("-%d %d %lf\n", track_id, track.ref_id, track.depth);
      for (const auto &[frame_id, p2d_id] : track.observations_) {
        const auto &frame = map.frames_[frame_id];
        const double depth = (frame.Tcw.q * track.point3d_ + frame.Tcw.t).z();
        printf("%d %d %lf\n", track_id, frame_id, depth);
      }
    }
    if (track.ref_id == -1) std::cout << "!!! no frame_id\n";
  }

  // prepare data
  size_t num_frames = map.frames_.size();
  std::vector<Pose> twc_vec(num_frames);
  std::vector<double> s_vec(twc_vec.size(), 1);
  std::vector<double> s_vec_loop(loop_info.cor_frame_ids_vec.size(), 1);
  for (auto &frame : map.frames_) {
    if (frame.registered) {
      twc_vec[frame.id] = frame.Tcw.inverse();
    }
  }

  constexpr double max_th = 0.1;
  if (abs(loop_info.scale_obs - 1) < max_th) {
    weight_o = 1 - abs(loop_info.scale_obs - 1) / max_th;
    printf("weight scale: %lf\n", weight_o);
  } else {
    weight_o = 0.0;
  }

  ceres::Problem problem;
  // covisibility edge
  AddCovisibilityEdge(problem, map, s_vec, twc_vec, use_key);
  // loop edge
  AddLoopEdge(problem, map, loop_info, s_vec, s_vec_loop, twc_vec, use_key);

  if (loop_info.scale_obs != -1) {
    printf("s12:%lf %zu %zu\n", loop_info.scale_obs, loop_info.cor_frame_ids_vec[0].size(),
           loop_info.cor_frame_ids_vec[1].size());
    ceres::CostFunction *cost_function = new ScaleCost(loop_info.scale_obs);
    problem.AddResidualBlock(cost_function, nullptr, &s_vec_loop[0], &s_vec_loop[1]);
  }

  for (auto &frame : map.frames_) {
    if (!frame.registered) continue;
    if (use_key && !frame.is_keyframe) continue;
    if (num_cov[frame.id] == 0) {
      printf("%d no covisiblity\n", frame.id);
      continue;
    }
    problem.SetParameterization(twc_vec[frame.id].q.coeffs().data(), new QuatParam);
    if (frame.id != loop_info.frame_id) {
      problem.SetParameterLowerBound(&s_vec[frame.id], 0, 0.2);
    }
    problem.SetParameterBlockConstant(twc_vec[frame.id].q.coeffs().data());//may bug
  }
  problem.SetParameterLowerBound(&s_vec_loop[0], 0, 0.2);
  problem.SetParameterLowerBound(&s_vec_loop[1], 0, 0.2);

  problem.SetParameterBlockConstant(&s_vec[map.init_id1]);
  problem.SetParameterBlockConstant(&s_vec[map.init_id2]);
  problem.SetParameterBlockConstant(twc_vec[map.init_id1].t.data());
  problem.SetParameterBlockConstant(twc_vec[map.init_id2].t.data());

  ceres::Solver::Options solver_options = InitSolverOptions();
  solver_options.minimizer_progress_to_stdout = true;
  solver_options.initial_trust_region_radius = 1e16;
  // solver_options.linear_solver_type = ceres::ITERATIVE_SCHUR;
  solver_options.trust_region_strategy_type = ceres::DOGLEG;
  ceres::Solver::Summary summary;

  ceres::Solve(solver_options, &problem, &summary);
  std::cout << summary.BriefReport() << "\n";

  if (!use_key) {
    for (size_t i = 0; i < num_frames; ++i) {
      if (i % (num_frames / 10) == 0 || s_vec[i] < 0) std::cout << i << " " << s_vec[i] << std::endl;
    }
    for (size_t i = 0; i < num_frames; ++i) {
      map.frames_[i].Tcw = twc_vec[i].inverse();
    }
  } else {
    int num_keyframe = 0;
    for (const auto &frame : map.frames_) {
      if (frame.registered && frame.is_keyframe) num_keyframe++;
    }
    int count = 0;
    for (size_t i = 0; i < num_frames; ++i) {
      auto &frame = map.frames_[i];
      if (frame.registered && frame.is_keyframe) {
        if (count % (num_keyframe / 10) == 0 || s_vec[i] < 0) std::cout << i << " " << s_vec[i] << std::endl;
        count++;
      }
    }

    for (size_t i = 0; i < num_frames; ++i) {
      auto &frame = map.frames_[i];
      if (frame.registered && frame.is_keyframe) {
        frame.tcw_old = frame.Tcw;
        frame.Tcw = twc_vec[i].inverse();
      }
    }
    for (size_t i = 0; i < num_frames; ++i) {
      auto &frame = map.frames_[i];
      if (frame.registered && !frame.is_keyframe) {
        const auto &ref_frame = map.frames_[frame.ref_id];
        CHECK(ref_frame.is_keyframe);
        CHECK(ref_frame.id != loop_info.frame_id);
        s_vec[i] = s_vec[frame.ref_id];
        Pose tcc2 = frame.Tcw.mul(ref_frame.tcw_old.inverse());
        frame.Tcw = tcc2.scale(s_vec[frame.ref_id]).mul(ref_frame.Tcw);
      }
    }
  }

  std::cout << s_vec_loop[0] << std::endl;
  std::cout << s_vec_loop[1] << std::endl;

  for (auto &track : map.tracks_) {
    if (track.outlier) continue;
    int frame_id = track.ref_id;
    int p2d_id = track.observations_[frame_id];
    Pose tcw = map.frames_[frame_id].Tcw;
    Eigen::Vector2d p2d = map.frames_[frame_id].points_normalized[p2d_id];
    track.point3d_ = tcw.q.inverse() * (s_vec[frame_id] * track.depth * p2d.homogeneous() - tcw.t);
  }
}

inline void BASolver::SetUp(ceres::Problem &problem, Map &map, Frame &frame) {
  const double sigma = std::sqrt(5.99) / map.cameras_[frame.camera_id].fx();
  int num_mea = 0;
  for (int i = 0; i < frame.track_ids_.size(); ++i) {
    if (frame.track_ids_[i] == -1) continue;
    Track &track = map.tracks_[frame.track_ids_[i]];
    ceres::CostFunction *cost_function = new ProjectionCost(frame.points_normalized[i], sigma);
    problem.AddResidualBlock(cost_function, nullptr, frame.Tcw.q.coeffs().data(), frame.Tcw.t.data(),
                             track.point3d_.data());
    num_mea++;
  }
  if (num_mea == 0) {
    LOG(ERROR) << "BA: NO Measurement In Frame " << frame.id;
  } else {
    problem.SetParameterization(frame.Tcw.q.coeffs().data(), new QuatParam);
  }
}

inline void BASolver::SetUpLBA(ceres::Problem &problem, Map &map, Frame &frame, int frame_id) {
  // const double sigma = std::sqrt(5.99) / map.cameras_[frame.camera_id].fx();
  const double sigma = 1000;
  int num_mea = 0;
  for (int i = 0; i < frame.track_ids_.size(); ++i) {
    if (frame.track_ids_[i] == -1) continue;
    num_mea++;

    Track &track = map.tracks_[frame.track_ids_[i]];
    ceres::CostFunction *cost_function = new ProjectionCost(frame.points_normalized[i], sigma);
    problem.AddResidualBlock(cost_function, nullptr, frame.Tcw.q.coeffs().data(), frame.Tcw.t.data(),
                             track.point3d_.data());

    if (track.angle_ > 5 || track.observations_.count(frame_id) == 0) {
      problem.SetParameterBlockConstant(track.point3d_.data());
    }
  }
  if (num_mea == 0) {
    LOG(ERROR) << "LBA: NO Measurement In Frame " << frame.id;
  } else {
    problem.SetParameterization(frame.Tcw.q.coeffs().data(), new QuatParam);
  }
}

std::vector<int> FindLocalBundle(const int frame_id, Map &map, const size_t num_images = 4) {
  Frame &frame = map.frames_[frame_id];
  int num_p3d = 0;
  std::unordered_map<int, int> covisiblity;
  for (auto &track_id : frame.track_ids_) {
    if (track_id == -1) continue;
    const auto &track = map.tracks_[track_id];
    num_p3d++;
    for (const auto &[t_frame_id, t_p2d_id] : track.observations_) {
      if (t_frame_id != frame.id) {
        covisiblity[t_frame_id] += 1;
      }
    }
  }
  std::vector<std::pair<int, int>> cov_vec(covisiblity.begin(), covisiblity.end());
  std::sort(cov_vec.begin(), cov_vec.end(),
            [](const std::pair<int, int> &a, const std::pair<int, int> &b) { return a.second > b.second; });

  const size_t num_eff_images = std::min(num_images, cov_vec.size() + 1);
  std::vector<int> local_bundle_image_ids;
  local_bundle_image_ids.reserve(num_eff_images);
  local_bundle_image_ids.push_back(frame_id);
  if (cov_vec.size() + 1 == num_eff_images) {
    for (const auto &[t_frame_id, num_cov] : cov_vec) {
      local_bundle_image_ids.push_back(t_frame_id);
    }
    return local_bundle_image_ids;
  }

  const double min_tri_angle_rad = 6 * 0.01745329;
  const std::array<std::pair<double, double>, 8> selection_thresholds = {
      std::make_pair(min_tri_angle_rad / 1.0, 0.6 * num_p3d), std::make_pair(min_tri_angle_rad / 1.5, 0.6 * num_p3d),
      std::make_pair(min_tri_angle_rad / 2.0, 0.5 * num_p3d), std::make_pair(min_tri_angle_rad / 2.5, 0.4 * num_p3d),
      std::make_pair(min_tri_angle_rad / 3.0, 0.3 * num_p3d), std::make_pair(min_tri_angle_rad / 4.0, 0.2 * num_p3d),
      std::make_pair(min_tri_angle_rad / 5.0, 0.1 * num_p3d), std::make_pair(min_tri_angle_rad / 6.0, 0.1 * num_p3d)};

  const Eigen::Vector3d proj_center = frame.Tcw.center();
  std::vector<Eigen::Vector3d> shared_points3D;
  shared_points3D.reserve(num_p3d);
  std::vector<double> tri_angles(cov_vec.size(), -1.0);
  std::vector<char> used_overlapping_images(cov_vec.size(), false);

  for (const auto &selection_threshold : selection_thresholds) {
    for (size_t frame_id = 0; frame_id < cov_vec.size(); ++frame_id) {
      if (cov_vec[frame_id].second < selection_threshold.second) break;
      if (used_overlapping_images[frame_id]) continue;

      const auto &frame_overlap = map.frames_[cov_vec[frame_id].first];
      const Eigen::Vector3d proj_center_overlap = frame_overlap.Tcw.center();

      // In the first iteration, compute the triangulation angle. In later
      // iterations, reuse the previously computed value.
      double &tri_angle = tri_angles[frame_id];
      if (tri_angle < 0.0) {
        // Collect the commonly observed 3D points.
        shared_points3D.clear();
        for (auto &track_id : frame.track_ids_) {
          if (track_id == -1) continue;
          const auto &track = map.tracks_[track_id];
          shared_points3D.push_back(track.point3d_);
        }

        // Calculate the triangulation angle at a certain percentile.
        const double kTriangulationAnglePercentile = 75;
        tri_angle =
            colmap::Percentile(colmap::CalculateTriangulationAngles(proj_center, proj_center_overlap, shared_points3D),
                               kTriangulationAnglePercentile);
      }

      // Check that the image has sufficient triangulation angle.
      if (tri_angle >= selection_threshold.first) {
        local_bundle_image_ids.push_back(frame_overlap.id);
        used_overlapping_images[frame_id] = true;
        // Check if we already collected enough images.
        if (local_bundle_image_ids.size() >= num_eff_images) {
          break;
        }
      }
    }

    // Check if we already collected enough images.
    if (local_bundle_image_ids.size() >= num_eff_images) {
      break;
    }
  }

  return local_bundle_image_ids;
}

std::vector<int> CovisibilityNeibors(const int frame_id, Map &map, const size_t num_images = 4) {
  Frame &frame = map.frames_[frame_id];
  std::unordered_map<int, int> covisiblity;
  for (auto &track_id : frame.track_ids_) {
    if (track_id == -1) continue;
    const auto &track = map.tracks_[track_id];
    for (const auto &[t_frame_id, t_p2d_id] : track.observations_) {
      covisiblity[t_frame_id] += 1;
    }
  }
  std::vector<std::pair<int, int>> cov_vec(covisiblity.begin(), covisiblity.end());
  std::sort(cov_vec.begin(), cov_vec.end(),
            [](const std::pair<int, int> &a, const std::pair<int, int> &b) { return a.second > b.second; });

  std::vector<int> local_bundle_image_ids(0);
  for (const auto &[t_frame_id, num_cov] : cov_vec) {
    local_bundle_image_ids.push_back(t_frame_id);
    if (local_bundle_image_ids.size() == num_images) break;
  }
  return local_bundle_image_ids;
}

void BASolver::LBA(int frame_id, Map &map) {//TODO boost 
  // find local frames
  std::vector<int> local_frame_ids1 = CovisibilityNeibors(frame_id, map);
  std::vector<int> local_frame_ids2 = FindLocalBundle(frame_id, map);
  std::set<int> local_frame_ids;
  for (auto &id : local_frame_ids1) {
    local_frame_ids.insert(id);
  }
  for (auto &id : local_frame_ids2) {
    local_frame_ids.insert(id);
  } 

  // set up problem
  ceres::Problem problem;
  printf("LBA: ");
  for (const int &id : local_frame_ids) {
    printf(" %d", id); 
    SetUpLBA(problem, map, map.frames_[id], frame_id);
  }
  printf("\n");

  // fix poses
  //TOOD here bug
  int fix_num = 0;
  if (local_frame_ids.count(map.init_id1) != 0) {
    problem.SetParameterBlockConstant(map.frames_[map.init_id1].Tcw.t.data());
    fix_num++;
  }
  if (local_frame_ids.count(map.init_id2) != 0) {
    problem.SetParameterBlockConstant(map.frames_[map.init_id2].Tcw.t.data());
    fix_num++;
  }
  if (fix_num == 0) {
    if (local_frame_ids2.size() >= 2) {
      problem.SetParameterBlockConstant(map.frames_[local_frame_ids2[local_frame_ids2.size() - 1]].Tcw.t.data());
      problem.SetParameterBlockConstant(map.frames_[local_frame_ids2[local_frame_ids2.size() - 2]].Tcw.t.data());
    } else if (local_frame_ids1.size() >= 2) {
      problem.SetParameterBlockConstant(map.frames_[local_frame_ids1[local_frame_ids1.size() - 1]].Tcw.t.data());
      problem.SetParameterBlockConstant(map.frames_[local_frame_ids1[local_frame_ids1.size() - 2]].Tcw.t.data());
    } else {
      printf("!!!LBA only one frame\n");
      problem.SetParameterBlockConstant(map.frames_[frame_id].Tcw.t.data());
    }
  }

  // for (const int &id : local_frame_ids) {
  //   if (set_all_pose_const || (num_frames_set_const_ != -1 && id <= num_frames_set_const_)) {
  //     auto &frame = map.frames_[id];
  //     if (!frame.registered && !frame.is_keyframe) continue;
  //     problem.SetParameterBlockConstant(frame.Tcw.q.coeffs().data());
  //     problem.SetParameterBlockConstant(frame.Tcw.t.data());
  //   }
  // }
 
  ceres::Solver::Options solver_options = InitSolverOptions(); 
  solver_options.max_num_iterations = 5;
  solver_options.function_tolerance = 1e-4;
  solver_options.parameter_tolerance = 1e-5;
  ceres::Solver::Summary summary;
  ceres::Solve(solver_options, &problem, &summary);

}

void BASolver::GBA(Map &map, bool accurate, bool fix_all_frames) {
  // set up problem
  ceres::Problem problem;
  for (auto &frame : map.frames_) {
    if (!frame.registered) continue;
    SetUp(problem, map, frame);
  }

  // fix poses
  if (!fix_all_frames) {
    problem.SetParameterBlockConstant(map.frames_[map.init_id1].Tcw.t.data());
    problem.SetParameterBlockConstant(map.frames_[map.init_id2].Tcw.t.data());
  } else {
    for (auto &frame : map.frames_) {
      if (!frame.registered) continue;
      problem.SetParameterBlockConstant(frame.Tcw.q.coeffs().data());
      problem.SetParameterBlockConstant(frame.Tcw.t.data());
    }
  }

  ceres::Solver::Options solver_options = InitSolverOptions();
  solver_options.minimizer_progress_to_stdout = true;
  if (accurate) {
    solver_options.max_num_iterations = 50;
    solver_options.function_tolerance = 1e-5;
    solver_options.parameter_tolerance = 1e-6;
  } else {
    solver_options.max_num_iterations = 20;
    solver_options.function_tolerance = 1e-4;
    solver_options.parameter_tolerance = 1e-5;
  }
  ceres::Solver::Summary summary;
  ceres::Solve(solver_options, &problem, &summary);
  std::cout << summary.BriefReport() << "\n";
}

void BASolver::KGBA(Map &map, const std::vector<int> fix_key_frame_ids, const bool order_frames) {
  KeyFrameSelection(map, fix_key_frame_ids, order_frames); 
  int num_rf = 0, num_kf = 0, num_mea = 0;

  ceres::Problem problem;
  for (auto &frame : map.frames_) {
    if (!frame.registered) continue;
    num_rf++;
    if (!frame.is_keyframe) continue;
    num_kf++;
    SetUp(problem, map, frame);
  }

  problem.SetParameterBlockConstant(map.frames_[map.init_id1].Tcw.t.data());
  problem.SetParameterBlockConstant(map.frames_[map.init_id2].Tcw.t.data());

  // if (set_all_pose_const) {
  //   for (auto &frame : map.frames_) {
  //     if (!frame.registered) continue;
  //     if (!frame.is_keyframe) continue;
  //     problem.SetParameterBlockConstant(frame.Tcw.q.coeffs().data());
  //     problem.SetParameterBlockConstant(frame.Tcw.t.data());
  //   }
  // }

  // if (num_frames_set_const_ != -1) {  
  //   for (auto &frame : map.frames_) {
  //     if (frame.id <= num_frames_set_const_) {
  //       if (!frame.registered) continue;
  //       if (!frame.is_keyframe) continue;
  //       problem.SetParameterBlockConstant(frame.Tcw.q.coeffs().data());
  //       problem.SetParameterBlockConstant(frame.Tcw.t.data());
  //     }
  //   }
  // }

  ceres::Solver::Options solver_options = InitSolverOptions();
  solver_options.minimizer_progress_to_stdout = true;
  solver_options.initial_trust_region_radius = 1e6;
  solver_options.max_num_iterations = 20;
  solver_options.function_tolerance = 1e-4;
  solver_options.parameter_tolerance = 1e-5;
  ceres::Solver::Summary summary;
  ceres::Solve(solver_options, &problem, &summary);
  std::cout << summary.BriefReport() << "\n";

  printf("kf: %d/%d\n", num_kf, num_rf);
  UpdateByRefFrame(map);
}
}
 