
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

// future work
// 1. if th_angle_lba <1.0 there will be many Wrong Registration
// we can use confidence of point to boost
// 2. when there no 3d points SIM3 canot be compute
// motion averaging may be a good idea
// 3. if inlier number is few there may be a bug
// 4. big rotation sometimes cause problem

// match frame connected  use local ba
// only a frame use local ba 3630

bool debug = false;
constexpr double th_rpe_lba = 16, th_angle_lba = 2.0;
constexpr double th_rpe_gba = 8, th_angle_gba = 2.0;
int num_image_reg = 2, num_image_reg_pre = 2;

Viewer viewer;
BASolver ba_solver;
Point3dProcessor p3d_processor;
std::string dir_path;
std::string dir_path_unorder;
std::string image_dir;
std::set<int> bad_essential_frame_ids;
Timer timer_gba_loop("Time Loop GBA %.6lfs\n");

void PreProcess(Map &map) {
  std::vector<Frame> frames;
  std::vector<Camera> cameras;
  std::vector<FramePair> frame_pairs;
  std::vector<std::tuple<std::string, int, int>> image_infos;
  ReadFrames(dir_path + "frame_colmap.bin", frames);
  ReadFramePairs(dir_path + "fp_colmap.bin", frame_pairs);
  ReadCamerasColmap(dir_path + "camera_colmap.bin", cameras);
  ReadExtraColmap(dir_path + "frame_extra_colmap.bin", image_infos);
  std::map<int, int> id_frame2id, id_camera2id_mix;
  for (size_t i = 0; i < frames.size(); ++i) id_frame2id[frames[i].id] = i;
  for (size_t i = 0; i < cameras.size(); ++i) id_camera2id_mix[cameras[i].id] = i;

  // set map camera
  std::map<int, Camera> cameras_unorder;
  std::vector<std::tuple<std::string, int, int>> image_infos_unorder;
  ReadCameraColmapTXT(dir_path_unorder + "cameras.txt", cameras_unorder);
  ReadExtraColmap(dir_path_unorder + "frame_extra_colmap.bin", image_infos_unorder);
  std::map<std::string, Camera> name2camera_uno;
  for (const auto &[name, id_frame, id_camera] : image_infos_unorder) {
    if (cameras_unorder.count(id_camera) != 0) name2camera_uno[name] = cameras_unorder[id_camera];
  }

  Camera seq(6000, 605.718555, 605.718555, 640, 360, -0.082443);

  std::vector<Camera> cameras_new;
  for (auto &[name, id_frame, id_camera] : image_infos) {
    // 4695 images_unorder/3801576506_65c6b35a0d_o.jpg
    CHECK(id_camera != -1) << "exit id_camera==-1";
    auto &frame = frames[id_frame2id[id_frame]];
    frame.name = name;
    frame.camera_id = id_camera;
    // std::cout << name << " " << id_camera << std::endl;
    // if (name.find("images_seq/") != -1) {
    //   frames[id_frame2id[id_frame]].camera_id = 6000;
    // } else if (name.find("images_unorder/") != -1) {
    //   frames[id_frame2id[id_frame]].camera_id = id_camera;
    //   const size_t num = sizeof("images_unorder/") - 1;
    //   const std::string name1 = name.substr(num, name.size() - num);
    //   auto &camera = cameras[id_camera2id_mix[id_camera]];
    //   if (name2camera_uno.count(name1) != 0) {
    //     const auto &camera_update = name2camera_uno[name1];
    //     camera.camera_params = camera_update.camera_params;
    //     camera.distort_params = camera_update.distort_params;
    //   }
    //   cameras_new.emplace_back(camera);
    // }
    if (name.find(".png") != -1) {
      // frame.name = "images_seq/" + name;
      frame.name = name;
      frame.camera_id = 6000;
    } else if (name.find(".jpg") != -1) {
      // frame.name = "images_unorder/" + name;
      frame.name = name;
      auto &camera = cameras[id_camera2id_mix[id_camera]];
      if (name2camera_uno.count(name) != 0) {
        const auto &camera_update = name2camera_uno[name];
        camera.camera_params = camera_update.camera_params;
        camera.distort_params = camera_update.distort_params;
      }
      cameras_new.emplace_back(camera);
    }
  }
  cameras_new.emplace_back(seq);
  cameras = cameras_new;

  // reorder
  std::sort(cameras.begin(), cameras.end(), [](const Camera &a, const Camera &b) { return a.id < b.id; });
  std::sort(frames.begin(), frames.end(), [](const Frame &a, const Frame &b) { return a.id < b.id; });
  printf("camera size: %zu frame size: %zu\n", cameras.size(), frames.size());

  // std::cout<< cameras[cameras.size() - 2].id << std::endl;
  // std::cout < < < < " - " << frames.back().id << std::endl;

  std::map<int, int> id2reorderid_camera;
  for (int i = 0; i < cameras.size(); ++i) {
    id2reorderid_camera[cameras[i].id] = i;
    cameras[i].id = i;
  }

  std::map<int, int> id2reorderid_frame;
  for (int i = 0; i < frames.size(); ++i) {
    auto &frame = frames[i];
    id2reorderid_frame[frame.id] = i;
    frame.id_colmap = frame.id;
    frame.id = i;
    CHECK(id2reorderid_camera.count(frame.camera_id) == 1) << frame.id_colmap << " " << frame.camera_id;
    const int camera_id = id2reorderid_camera[frame.camera_id];
    frame.camera_id = camera_id;
    for (int k = 0; k < frame.points.size(); ++k) {
      ImageToNormalized(cameras[camera_id], frame.points[k], frame.points_normalized[k]);
    }
  }

  for (auto &fp : frame_pairs) {
    if (id2reorderid_frame.count(fp.id1) != 0 && id2reorderid_frame.count(fp.id2) != 0) {
      fp.id1 = id2reorderid_frame.at(fp.id1);
      fp.id2 = id2reorderid_frame.at(fp.id2);
    } else {
      std::cout << "error" << fp.id1 << " - " << fp.id2 << std::endl;
    }
  }

  map.frames_ = frames;
  map.cameras_ = cameras;
  map.frame_pairs_ = frame_pairs;
  map.RemoveRedundancyPoints();
  map.Init();
}

bool IsGoodRelativePose(Map &map, FramePair &fp) {
  constexpr int num_min_matches = 100;
  constexpr double ratio_th = 0.75;
  constexpr double pure_rotation_th = 0.05;
  constexpr double sin_th = sin(2.0 * M_PI / 180);
  constexpr double cos_th = cos(2.0 * M_PI / 180);

  if ((std::abs(fp.id1 - fp.id2) != 1) && fp.matches.size() < num_min_matches) return true;

  auto &frame1 = map.frames_[fp.id1], &frame2 = map.frames_[fp.id2];

  const Eigen::Vector3d relative_motion = frame2.center() - frame1.center();
  const double distance = relative_motion.norm();
  const bool is_pure_rotation = distance < pure_rotation_th;
  if (is_pure_rotation) printf("pure rotation %lf\n", relative_motion.norm());
  const Eigen::Vector3d t12 = relative_motion.normalized();

  std::vector<char> inlier_mask;
  int num_matches = 0, num_inliers = 0;
  for (int i = 0; i < fp.matches.size(); ++i) {
    if (!fp.inlier_mask[i]) continue;
    num_matches++;

    bool good_relative_pose = true;

    Eigen::Vector2d p2d1 = frame1.points_normalized[fp.matches[i].id1];
    Eigen::Vector2d p2d2 = frame2.points_normalized[fp.matches[i].id2];
    Eigen::Vector3d ray1 = (frame1.qwc() * p2d1.homogeneous()).normalized();
    Eigen::Vector3d ray2 = (frame2.qwc() * p2d2.homogeneous()).normalized();

    if (is_pure_rotation) {  // error
      // double cos_theta = ray1.dot(ray2);
      // good_relative_pose = cos_theta > cos_th;
    } else {
      bool use_ray2 = std::abs(ray1.dot(t12)) > std::abs(ray2.dot(t12));
      // compute deg between ray2 and plane_ray1-t
      Eigen::Vector3d n = (ray2.cross(t12)).normalized();  // norm of plane_ray1-t
      double sin_theta = std::abs(n.dot(ray1));
      Eigen::Vector3d n1 = (ray1.cross(t12)).normalized();  // norm of plane_ray1-t
      double sin_theta1 = std::abs(n1.dot(ray2));

      good_relative_pose = use_ray2 ? sin_theta < sin_th : sin_theta1 < sin_th;

      if (ray1.dot(ray2) < 0 && ray1.dot(t12) < 0) good_relative_pose = false;
      if (ray1.dot(t12) < 0 && ray2.dot(t12) > ray1.dot(t12) + sin_th) good_relative_pose = false;
      if (ray2.dot(t12) > 0 && ray2.dot(t12) > ray1.dot(t12) + sin_th) good_relative_pose = false;

      // if (ray1.dot(ray2) < 0 && ray1.dot(t12) < 0) good_relative_pose = false;
      // if (ray1.dot(ray2) < 0.999 && ray1.dot(t12) < 0 && ray2.dot(t12) > ray1.dot(t12)) good_relative_pose = false;
      // if (ray1.dot(ray2) < 0.99 && ray2.dot(t12) > 0 && ray2.dot(t12) > ray1.dot(t12)) good_relative_pose = false;
    }
    if (good_relative_pose) num_inliers++;
    inlier_mask.emplace_back(good_relative_pose);
  }

  const double ratio = 1.0 * num_inliers / num_matches;
  printf("%d %d %lf %d/%d\n", frame1.id, frame2.id, ratio, num_inliers, num_matches);
  if (ratio < ratio_th) {
    if (debug) {
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
    if (frame1.registered && frame2.registered && frame1.is_keyframe && frame2.is_keyframe &&
        frame1.camera_id == frame2.camera_id) {
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
    if (!map.frames_[id].registered) continue;
    matched_frame_id.emplace_back(id);
  }
  matched_frame_id.emplace_back(frame_id);
  return matched_frame_id;
}

std::vector<std::set<int>> GetLoopCorFrame(Map &map, Frame &next_frame, Frame &tmp_frame) {
  const std::vector<int> matched_frame_id = GetMatchedFrameIds(map, next_frame.id);
  std::map<int, int> tmp_set, tmp_set1;
  for (auto &id : matched_frame_id)
    if (id != next_frame.id) tmp_set[id] = tmp_set1[id] = 0;

  for (auto &track_id : next_frame.track_ids_) {
    if (track_id == -1) continue;
    for (auto &[t_frame_id, t_p2d_id] : map.tracks_[track_id].observations_) {
      if (tmp_set.count(t_frame_id) != 0) {
        tmp_set[t_frame_id]++;
      }
    }
  }
  for (auto &track_id : tmp_frame.track_ids_) {
    if (track_id == -1) continue;
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
      // greater than 3 to avoid noise
      if (tmp_set[id] > tmp_set1[id] && tmp_set[id] >= 3) {
        cor_frame_ids_vec[0].insert(id);
        printf("0 %d %d %d\n", id, tmp_set[id], tmp_set1[id]);
      } else if (tmp_set1[id] > tmp_set[id] && tmp_set1[id] >= 3) {
        cor_frame_ids_vec[1].insert(id);
        printf("1 %d %d %d\n", id, tmp_set[id], tmp_set1[id]);
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
  loop_info.twc_vec.emplace_back(frame1.Tcw.inverse());
  loop_info.twc_vec.emplace_back(frame2.Tcw.inverse());
  loop_info.cor_frame_ids_vec = GetLoopCorFrame(map, frame1, frame2);

  int count = 0;
  double depth1 = 0, depth2 = 0;
  for (size_t i = 0; i < frame1.track_ids_.size(); ++i) {
    const int track_id1 = frame1.track_ids_[i];
    if (track_id1 == -1) continue;
    auto &track1 = map.tracks_[track_id1];

    const int track_id2 = frame2.track_ids_[i];
    if (track_id2 == -1) continue;
    auto &track2 = map.tracks_[track_id2];
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
      if (!track1.outlier) {
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

inline void CorrectRelativePosition(Map &map, int frame_id, int iter, bool loop_found) {
  Pose tcw;
  std::vector<std::pair<int, int>> cor_2d_3d_ids;
  auto &next_frame = map.frames_[frame_id];
  std::string name = std::to_string(iter) + "_" + std::to_string(frame_id);
  bool can_error_correct = loop_found;
  if (!can_error_correct) {
    if (!CheckAllEssential1(map, frame_id)) {
      if (TryLocate(map, frame_id, th_angle_lba, tcw, cor_2d_3d_ids)) {
        can_error_correct = true;
      } else {
        SaveSfMState("_seq_" + name + ".bin", iter, map);
        if (bad_essential_frame_ids.count(frame_id - 1) || bad_essential_frame_ids.count(frame_id + 1)) {
          map.DeregistrationFrame(frame_id);
          map.frames_[frame_id].registered_fail = true;
        }
      }
    }
  }
  if (can_error_correct) {
    SaveSfMState("seq_" + name + ".bin", iter, map);
    viewer.Draw(map, debug);

    std::vector<int> matched_frame_id = GetMatchedFrameIds(map, frame_id);
    const double dist = (next_frame.Tcw.center() - map.tmp_frame.center()).norm();
    const bool negtive_depth = CheckNegtiveDepth(map, map.tmp_frame, next_frame);
    std::cout << "DIST:  " << dist << std::endl;
    if (dist > 1.5 || negtive_depth) {
      UpdateKey(map, matched_frame_id, true);
      UpdateByRefFrame(map);
      LoopInfo loop_info = GetLoopInfo(map, next_frame, map.tmp_frame);
      if (loop_info.cor_frame_ids_vec[0].size() == 0 || loop_info.cor_frame_ids_vec[1].size() == 0) return;
      ba_solver.ScalePoseGraphUnorder(loop_info, map, true);
      viewer.Draw(map, debug);
    }
    MergeTrackLoop(map, next_frame, map.tmp_frame);

    if (dist > 0.02) {
      timer_gba_loop.resume();
      printf("size: %d : ", matched_frame_id.size());
      for (auto &id : matched_frame_id) {
        printf("%d ", id);
      }
      printf("\n");
      ba_solver.KGBA(map, matched_frame_id, false);
      timer_gba_loop.stop();
      p3d_processor.FilterPoints3d(map, th_rpe_gba, th_angle_gba);
      num_image_reg_pre = num_image_reg;
    }

    viewer.Draw(map, debug);
    CheckAllEssential1(map, frame_id);
    map.tmp_frame.registered = false;
  }
}

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  // load config
  nlohmann::json config_json;
  std::ifstream ifs("../config_mix.json");
  ifs >> config_json;

  dir_path = config_json["dir_path"];
  dir_path_unorder = config_json["dir_path_unorder"];
  const std::string output_path = config_json["output_path"];
  const double camera_size = config_json["camera_size"];
  debug = (config_json["debug"] == 1);
  image_dir = "/home/yzc/Data/SfM/Roman_Forum_MIX/images/";

  TimerArray timer;
  viewer.Init();
  viewer.camera_size_ = camera_size;

  Map map;
  PreProcess(map);

  FramePair init_frame_pair;
  FindInitFramePair(map, init_frame_pair);
  InitializeMap(map, init_frame_pair);
  ba_solver.GBA(map);

  for (int iter = 0; iter < map.frames_.size(); iter++) {
    viewer.Draw(map, false);

    timer.tot.resume();
    printf("-----------------------------------------------\n");
    auto [frame_id, num_p3d] = map.MaxPoint3dFrameIdSeq();
    printf(" %d %d \n", frame_id, num_p3d);
    if (num_p3d < 30) frame_id = map.MaxPoint3dFrameId();
    printf("iter %d %d %s\n", iter, frame_id, map.frames_[frame_id].name.c_str());
    if (frame_id == -1) break;

    auto &next_frame = map.frames_[frame_id];
    // if (map.cameras_[next_frame.camera_id].distort_params[0] == 0) {
    //   LOG(ERROR) << next_frame.camera_id;
    //   map.cameras_[next_frame.camera_id].log();
    // }

    TIMING(timer.reg, if (!RegisterImage(frame_id, map)) break);

    CorrectRelativePosition(map, frame_id, iter, false);
    if (next_frame.registered_fail) continue;

    TIMING(timer.tri, p3d_processor.TriangulateFramePoint(map, frame_id, th_angle_lba));
    TIMING(timer.fil, p3d_processor.FilterPointsFrame(map, frame_id, th_rpe_lba, th_angle_lba));
    if (p3d_processor.CheckFrameMeasurement(map, frame_id)) {
      map.DeregistrationFrame(frame_id);
      next_frame.registered_fail = true;
      continue;
    }
    TIMING(timer.lba, ba_solver.LBA(frame_id, map));
    TIMING(timer.merge, p3d_processor.MergeTracks(map, frame_id, th_rpe_lba));
    TIMING(timer.fil, p3d_processor.FilterPointsFrame(map, frame_id, th_rpe_lba, th_angle_lba));

    num_image_reg++;
    if (num_image_reg > 1.2 * num_image_reg_pre) {
      TIMING(timer.che, p3d_processor.CheckTrackDepth(map));  // 1.6
      p3d_processor.CheckFramesMeasurement(map, th_rpe_lba, th_angle_lba);
      TIMING(timer.gba, ba_solver.KGBA(map, std::vector<int>(0), false));
      TIMING(timer.fil, p3d_processor.FilterPoints3d(map, th_rpe_gba, th_angle_gba));  // 8
      num_image_reg_pre = num_image_reg;
    }

    UpdateCovisiblity(map, frame_id);
    timer.tot.stop();

    map.frames_[frame_id].flag_for_view = false;
  }

  ba_solver.GBA(map);
  SaveSfMState("seq_end.bin", map.frames_.size(), map);

  for (auto &timer_ptr : timer.timer_vec) {
    timer_ptr->print();
  }

  map.LogReprojectError();
  viewer.DrawGraph(map, true);
  viewer.DrawKey(map, true);
  WriteCamerasBinary(output_path + "cameras.bin", map.cameras_);
  WriteImagesBinary(output_path + "images.bin", map.frames_);
  WritePoints3DBinary(output_path + "points3D.bin", map.tracks_);
  return 0;
}
