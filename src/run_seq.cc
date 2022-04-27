
#include <fstream>

#include "3rdparty/json/json.hpp"
#include "geometry/essential.h"
#include "geometry/pnp.h"
#include "geometry/track_processor.h"
#include "base/camera.h"
#include "base/map.h"
#include "geometry/map_initializer.h"
#include "optimization/ba_solver.h"
#include "utility/io_ecim.hpp"
#include "utility/timer.h"
#include "utility/view.h"

Timer timer_gba_loop("Time Loop GBA %.6lfs\n");

bool debug = false;
std::string image_dir;
std::set<int> bad_essential_frame_ids;

Viewer viewer;
BASolver ba_solver;
Point3dProcessor p3d_processor;

int num_image_reg = 2, num_image_reg_pre = 2;

constexpr double th_rpe_lba = 16, th_angle_lba = 1.0;
constexpr double th_rpe_gba = 8, th_angle_gba = 2.0;

void PreProcess(const std::string dir_path, Map &map) {
  std::vector<Frame> frames;
  std::vector<Camera> cameras;
  std::vector<FramePair> frame_pairs;
  std::vector<std::tuple<std::string, int, int>> image_infos;
  ReadFrames(dir_path + "frame_colmap.bin", frames);
  ReadFramePairs(dir_path + "fp_colmap.bin", frame_pairs);
  ReadExtraColmap(dir_path + "frame_extra_colmap.bin", image_infos);
  std::map<int, int> id_frame2id;
  for (size_t i = 0; i < frames.size(); ++i) id_frame2id[frames[i].id] = i;
  for (const auto &[name, id_frame, id_camera] : image_infos) {
    CHECK(id_camera != -1) << "exit id_camera==-1";
    frames[id_frame2id[id_frame]].name = name;
    frames[id_frame2id[id_frame]].camera_id = 0;
  }
  // set map camera
  // Camera seq(0, 605.718555, 605.718555, 640, 360, -0.082443);
  Camera seq(0, 885, 885, 480, 270, -0.01);
  cameras.emplace_back(seq);

  // reorder
  std::sort(frames.begin(), frames.end(), [](const Frame &a, const Frame &b) { return a.id < b.id; });
  std::cout << frames.size() << " - " << frames.back().id << std::endl;

  std::map<int, int> id2reorderid_frame;
  for (int i = 0; i < frames.size(); ++i) {
    auto &frame = frames[i];
    id2reorderid_frame[frame.id] = i;
    frame.id_colmap = frame.id;
    frame.id = i;
    for (int k = 0; k < frame.points.size(); ++k) {
      ImageToNormalized(cameras[0], frame.points[k], frame.points_normalized[k]);
    }
  }
  std::vector<FramePair> frame_pairs1;
  for (auto &fp : frame_pairs) {
    if (id2reorderid_frame.count(fp.id1) != 0 && id2reorderid_frame.count(fp.id2) != 0) {
      fp.id1 = id2reorderid_frame.at(fp.id1);
      fp.id2 = id2reorderid_frame.at(fp.id2);
      if (fp.matches.size() >= 30) {
        frame_pairs1.emplace_back(fp);
      }
    } else {
      std::cout << "error" << fp.id1 << " - " << fp.id2 << std::endl;
    }
  }

  map.frames_ = frames;
  map.cameras_ = cameras;
  map.frame_pairs_ = frame_pairs1;
  map.RemoveRedundancyPoints();
  map.Init();
  for (const auto &frame : map.frames_) {
    int num = frame.points_normalized.size();
    if (num < 200) {
      printf("frame: %d too little feature: %d name: %s\n", num, frame.id, frame.name.c_str());
    }
  }
}

bool IsGoodRelativePose(Map &map, FramePair &fp) {
  constexpr int num_min_matches = 100;
  constexpr double ratio_th = 0.75;
  constexpr double pure_rotation_th = 0.05;
  constexpr double sin_th = sin(2.0 * M_PI / 180);
  constexpr double cos_th = cos(2.0 * M_PI / 180);

  if ((std::abs(fp.id1 - fp.id2) != 1) && fp.matches.size() < num_min_matches) return true;

  auto &frame1 = map.frames_[fp.id1];
  auto &frame2 = map.frames_[fp.id2];
  const Eigen::Vector3d t12 = frame2.center() - frame1.center();
  const bool is_pure_rotation = t12.norm() < pure_rotation_th;
  if (is_pure_rotation) printf("pure rotation %lf\n", t12.norm());

  int num_matches = 0, num_inliers = 0;
  std::vector<char> inlier_mask;
  for (int i = 0; i < fp.matches.size(); ++i) {
    if (!fp.inlier_mask[i]) continue;
    num_matches++;
    Eigen::Vector2d p2d1 = frame1.points_normalized[fp.matches[i].id1];
    Eigen::Vector2d p2d2 = frame2.points_normalized[fp.matches[i].id2];
    Eigen::Vector3d ray1 = (frame1.qwc() * p2d1.homogeneous()).normalized();
    Eigen::Vector3d ray2 = (frame2.qwc() * p2d2.homogeneous()).normalized();

    bool good_essential = true;
    if (is_pure_rotation) {  // error
      // double cos_theta = ray1.dot(ray2);
      // good_essential = cos_theta > cos_th;
    } else {
      bool use_ray2 = std::abs(ray1.dot(t12)) > std::abs(ray2.dot(t12));
      // compute deg between ray2 and plane_ray1-t
      Eigen::Vector3d n = (ray2.cross(t12)).normalized();  // norm of plane_ray1-t
      double sin_theta = std::abs(n.dot(ray1));

      Eigen::Vector3d n1 = (ray1.cross(t12)).normalized();  // norm of plane_ray1-t
      double sin_theta1 = std::abs(n1.dot(ray2));
      if (use_ray2) {
        good_essential = sin_theta < sin_th;
      } else {
        good_essential = sin_theta1 < sin_th;
      }

      if (ray1.dot(ray2) < 0 && ray1.dot(t12) < 0) good_essential = false;
      if (ray1.dot(ray2) < 0.999 && ray1.dot(t12) < 0 && ray2.dot(t12) > ray1.dot(t12)) good_essential = false;
      if (ray1.dot(ray2) < 0.99 && ray2.dot(t12) > 0 && ray2.dot(t12) > ray1.dot(t12)) good_essential = false;
      if (fp.id1 == 3703 && fp.id2 == 3706) {
        // good_essential = false;
        printf("%lf %lf %lf|%lf %lf %lf\n", sin_th, sin_theta, sin_theta1, ray1.dot(ray2), ray1.dot(t12),
               ray2.dot(t12));
        // if (ray1.dot(ray2) < 0 && ray1.dot(t12) < 0) printf("1\n");
        // if (ray1.dot(ray2) < 0.999 && ray1.dot(t12) < 0 && ray2.dot(t12) > ray1.dot(t12)) printf("2\n");
        // if (ray1.dot(ray2) < 0.99 && ray2.dot(t12) > 0 && ray2.dot(t12) < ray1.dot(t12)) printf("3\n");
      }
    }
    if (good_essential) num_inliers++;
    inlier_mask.emplace_back(good_essential);
  }

  const double ratio = 1.0 * num_inliers / num_matches;
  printf("%d %d %lf %d/%d\n", frame1.id, frame2.id, ratio, num_inliers, num_matches);
  if (ratio < ratio_th) {
    if (false) {
      cv::Mat image1 = cv::imread(image_dir + frame1.name);
      cv::Mat image2 = cv::imread(image_dir + frame2.name);
      if (image1.empty() || image2.empty()) {
        printf("%s\n", (image_dir + frame1.name).c_str());
      } else {
        DrawFeatureMatches(image1, image2, frame1.points, frame2.points, fp.matches, inlier_mask);
        DrawFeatureMatches1(image1, frame1.points, frame2.points, fp.matches, inlier_mask);
      }
      cv::waitKey();
      frame1.flag_for_view = frame2.flag_for_view = true;
      viewer.Draw(map, true);
      frame1.flag_for_view = frame2.flag_for_view = false;
    }
    return false;
  }
  return true;
}

bool CheckAllEssential1(Map &map, int frame_id) {
  bad_essential_frame_ids.clear();
  int num_inlier = 0, num_all = 0;
  for (const auto id : map.frameid2framepairids_[frame_id]) {
    auto &fp = map.frame_pairs_[id];
    const auto &frame1 = map.frames_[fp.id1];
    const auto &frame2 = map.frames_[fp.id2];
    if (frame1.registered && frame2.registered && frame1.is_keyframe && frame2.is_keyframe) {
      if (IsGoodRelativePose(map, fp)) {
        ++num_inlier;
      } else {
        bad_essential_frame_ids.insert(fp.id1 == frame_id ? fp.id2 : fp.id1);
      }
      ++num_all;
    }
  }
  if (num_inlier < num_all) {
    std::cerr << "Bad Registeration\n";
    return false;
  }
  return true;
}

inline std::vector<int> GetMatchedFrameIds(Map &map, int frame_id) {
  std::vector<int> matched_frame_id;
  for (const int &id : map.frameid2matched_frameids_[frame_id]) {
    if (!map.frames_[id].is_keyframe) continue;
    matched_frame_id.emplace_back(id);
  }
  matched_frame_id.emplace_back(frame_id);
  return matched_frame_id;
}

std::vector<std::set<int>> DivideMatchedFrames(Map &map, Frame &next_frame, Frame &tmp_frame) {
  const std::vector<int> matched_frame_id = GetMatchedFrameIds(map, next_frame.id);
  std::map<int, int> tmp_set, tmp_set1;
  for (auto &id : matched_frame_id)
    if (id != next_frame.id) tmp_set[id] = tmp_set1[id] = 0;

  for (auto &track_id : next_frame.track_ids_) {
    if (track_id == -1) continue;
    if (map.tracks_[track_id].outlier) continue;
    for (auto &[t_frame_id, t_p2d_id] : map.tracks_[track_id].observations_) {
      if (tmp_set.count(t_frame_id) != 0) {
        tmp_set[t_frame_id]++;
      }
    }
  }
  for (auto &track_id : tmp_frame.track_ids_) {
    if (track_id == -1) continue;
    if (map.tracks_[track_id].outlier) continue;
    for (auto &[t_frame_id, t_p2d_id] : map.tracks_[track_id].observations_) {
      if (tmp_set1.count(t_frame_id) != 0) {
        tmp_set1[t_frame_id]++;
      }
    }
  }
  std::vector<std::set<int>> cor_frame_ids_vec(2);
  for (auto &id : matched_frame_id) {
    if (id != next_frame.id) {
      // printf("%d %d %d\n", id, tmp_set[id], tmp_set1[id]);
      if (tmp_set[id] > tmp_set1[id]) {
        cor_frame_ids_vec[0].insert(id);
        printf("0 %d\n", id);
      } else if (tmp_set1[id] > tmp_set[id]) {
        cor_frame_ids_vec[1].insert(id);
        printf("1 %d\n", id);
      }
    }
  }

  // cor_frame_ids_vec[0].insert(125);
  // cor_frame_ids_vec[0].insert(155);
  // map.frames_[125].is_keyframe = true;
  // map.frames_[155].is_keyframe = true;

  // CHECK(cor_frame_ids_vec[0].size() > 0);
  // CHECK(cor_frame_ids_vec[1].size() > 0);
  return cor_frame_ids_vec;
}

LoopInfo GetLoopInfo(Map &map, Frame &frame1, Frame &frame2) {
  LoopInfo loop_info;
  loop_info.frame_id = frame1.id;
  loop_info.twc_vec.emplace_back(frame1.Tcw.inverse());
  loop_info.twc_vec.emplace_back(frame2.Tcw.inverse());
  loop_info.cor_frame_ids_vec = DivideMatchedFrames(map, frame1, frame2);

  int count = 0;
  double depth1 = 0, depth2 = 0;
  for (size_t i = 0; i < frame1.track_ids_.size(); ++i) {
    const int track_id1 = frame1.track_ids_[i];
    if (track_id1 == -1) continue;
    auto &track1 = map.tracks_[track_id1];
    if (track1.outlier) continue;

    const int track_id2 = frame2.track_ids_[i];
    if (track_id2 == -1) continue;
    auto &track2 = map.tracks_[track_id2];
    if (track2.outlier) continue;
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

inline bool TryLocate(Map &map, const int frame_id, const int th_angle_lba, Pose &tcw,
                      std::vector<std::pair<int, int>> &cor_2d_3d_ids) {
  bool reg_success = RegisterNextImageLocal(frame_id, bad_essential_frame_ids, map);

  if (!reg_success) {
    bool have_neighbor = false;
    std::vector<int> adjacent_frame_ids = {frame_id - 1, frame_id + 1};
    for (auto &id : adjacent_frame_ids) {
      if (bad_essential_frame_ids.count(id) != 0) {
        map.frames_[frame_id].registered = false;
        p3d_processor.TriangulateFramePoint(map, id, th_angle_lba);
        map.frames_[frame_id].registered = true;
        have_neighbor = true;
      }
    }
    if (have_neighbor) reg_success = RegisterNextImageLocal(frame_id, bad_essential_frame_ids, map);
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

inline void CorrectRelativePosition(Map &map, int frame_id, int iter) {
  if (!CheckAllEssential1(map, frame_id)) {
    std::string name = std::to_string(iter) + "_" + std::to_string(frame_id);
    Pose tcw;
    std::vector<std::pair<int, int>> cor_2d_3d_ids;
    auto &next_frame = map.frames_[frame_id];
    if (TryLocate(map, frame_id, th_angle_lba, tcw, cor_2d_3d_ids)) {
      SaveSfMState("seq_" + name + ".bin", iter, map);
      viewer.Draw(map, debug);

      std::vector<int> matched_frame_id = GetMatchedFrameIds(map, frame_id);
      const double dist = (next_frame.Tcw.center() - map.tmp_frame.center()).norm();
      const bool negtive_depth = CheckNegtiveDepth(map, map.tmp_frame, next_frame);

      // map.LogFrameReprojectError1(frame_id);

      // for (auto &[frame_id, p2d_id] : track.observations_) {
      //   auto &frame = map.frames_[frame_id];
      //   Eigen::Vector3d p_c = frame.tcw.q * track.point3d_ + frame.tcw.t;
      //   double z = p_c.z();
      //   Eigen::Vector2d res = p_c.hnormalized() - frame.points_normalized[p2d_id];
      //   printf("-%d %d re %lf %lf\n", frame_id, p2d_id, res.norm() * 885, z);
      // }
      if (dist > 1.5 || negtive_depth) {
        UpdateKey(map, matched_frame_id, true);
        UpdateByRefFrame(map);
        LoopInfo loop_info = GetLoopInfo(map, next_frame, map.tmp_frame);
        if (loop_info.cor_frame_ids_vec[0].size() == 0 || loop_info.cor_frame_ids_vec[1].size() == 0) return;
        ba_solver.ScalePoseGraphUnorder(loop_info, map, true);
        viewer.Draw(map, debug);
      }

      MergeTrackLoop(map, next_frame, map.tmp_frame);
      p3d_processor.CheckTrackDepth(map);
      if (dist > 0.02) {
        timer_gba_loop.resume();
        ba_solver.KGBA(map, matched_frame_id, true);
        timer_gba_loop.stop();
        p3d_processor.FilterPoints3d(map, th_rpe_gba, th_angle_gba);
        num_image_reg_pre = num_image_reg;
      }

      viewer.Draw(map, debug);
      CheckAllEssential1(map, frame_id);
      map.tmp_frame.registered = false;
    } else {
      SaveSfMState("_seq_" + name + ".bin", iter, map);
    }
  }
}

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);

  nlohmann::json config_json;
  std::ifstream ifs("../config_seq.json");
  ifs >> config_json;
  ifs.close();

  const std::string data_path = config_json["data_path"];
  const std::string seq_name = config_json["seq_name"];
  const std::string output_path = config_json["output_path"];
  const double camera_size = config_json["camera_size"];
  image_dir = data_path + seq_name + "/images/";
  debug = (config_json["debug"] == 1);

  Timer timer_tot("Time Total %.6lfs\n");
  Timer timer_reg("Time reg %.6lfs\n");
  Timer timer_tri("Time tri %.6lfs\n");
  Timer timer_fil("Time fil %.6lfs\n");
  Timer timer_che("Time check %.6lfs\n");
  Timer timer_lba("Time LBA %.6lfs\n");
  Timer timer_gba("Time GBA %.6lfs\n");
  Timer timer_merge("Time loop %.6lfs\n");
  std::vector<Timer *> timer_vec = {&timer_tot, &timer_reg, &timer_tri, &timer_fil,
                                    &timer_che, &timer_lba, &timer_gba, &timer_merge};
  Map map;

  viewer.Init();
  viewer.camera_size_ = camera_size;
  PreProcess(data_path + seq_name + "/model/", map);

  FramePair init_frame_pair;
  FindInitFramePair(map, init_frame_pair);
  // FindPair(map.frame_pairs_, 0, 5, init_frame_pair);
  InitializeMap(map, init_frame_pair);
  ba_solver.GBA(map);
  viewer.Draw(map, false);

  for (int iter = 0; iter < map.frames_.size(); iter++) {
    // if (iter == 0) {
    //   LoadSfMState("seq_1118_1120.bin", iter, map);
    //   num_image_reg = num_image_reg_pre = iter;
    //   map.DeregistrationFrame(1120);
    //   // viewer.DrawGraph(map, true);
    // }

    if (iter % 1000 == 0) {
      SaveSfMState("seq_" + std::to_string(iter) + ".bin", iter, map);
    }

    timer_tot.resume();

    timer_reg.resume();
    printf("-----------------------------------------------\n");
    // select next frame
    auto [frame_id, num_p3d] = map.MaxPoint3dFrameIdSeq();
    if (num_p3d < 20) frame_id = map.MaxPoint3dFrameId();
    if (frame_id == -1) break;
    printf("Iter %d %d %s\n", iter, frame_id, map.frames_[frame_id].name.c_str());
    if (!RegisterImage(frame_id, map)) break;
    timer_reg.stop();

    CorrectRelativePosition(map, frame_id, iter);

    TIMING(timer_tri, p3d_processor.TriangulateFramePoint(map, frame_id, th_angle_lba));

    TIMING(timer_fil, p3d_processor.FilterPointsFrame(map, frame_id, th_rpe_lba, th_angle_lba));

    if (p3d_processor.CheckFrameMeasurement(map, frame_id)) continue;

    TIMING(timer_lba, ba_solver.LBA(frame_id, map));

    TIMING(timer_merge, p3d_processor.MergeTracks(map, frame_id, th_rpe_lba));

    TIMING(timer_fil, p3d_processor.FilterPointsFrame(map, frame_id, th_rpe_lba, th_angle_lba));

    timer_gba.resume();
    if (num_image_reg++ > 1.2 * num_image_reg_pre) {
      TIMING(timer_che, p3d_processor.CheckTrackDepth(map));
      p3d_processor.CheckFramesMeasurement(map, th_rpe_lba, th_angle_lba);
      ba_solver.KGBA(map, std::vector<int>(0), true);
      TIMING(timer_fil, p3d_processor.FilterPoints3d(map, th_rpe_gba, th_angle_gba));
      num_image_reg_pre = num_image_reg;
    }
    timer_gba.stop();

    UpdateCovisiblity(map, frame_id);
    timer_tot.stop();

    viewer.Draw(map, false);
    map.frames_[frame_id].flag_for_view = false;
  }

  for (auto &timer_ptr : timer_vec) {
    timer_ptr->print();
  }
  timer_gba_loop.print();
  map.LogReprojectError();

  WriteCamerasBinary(output_path + "cameras.bin", map.cameras_);
  WriteImagesBinary(output_path + "images.bin", map.frames_);
  WritePoints3DBinary(output_path + "points3D.bin", map.tracks_);
  viewer.DrawGraph(map, true);
  viewer.DrawKey(map, true);
  return 0;
}
