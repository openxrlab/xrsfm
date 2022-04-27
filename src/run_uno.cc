
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
#include "utility/viewer.h"

bool debug = false;
constexpr double th_rpe_lba = 8, th_angle_lba = 2.0;
constexpr double th_rpe_gba = 4, th_angle_gba = 1.5;

std::string dir_path;
std::string image_dir;
std::set<int> bad_essential_frame_ids;

void PreProcess(const std::string dir_path, Map &map) {
  std::vector<Camera> cameras_refine;
  std::vector<std::tuple<std::string, int, int>> image_infos;
  ReadFrames(dir_path + "frame_colmap.bin", map.frames_);
  ReadFramePairs(dir_path + "fp_colmap.bin", map.frame_pairs_);
  ReadCamerasColmap(dir_path + "camera_colmap.bin", map.cameras_);
  ReadExtraColmap(dir_path + "frame_extra_colmap.bin", image_infos);
  ReadCameraColmapTXT(dir_path + "cameras.txt", cameras_refine);
  std::map<int, int> id_frame2id, id_camera2id_refine;
  for (size_t i = 0; i < map.frames_.size(); ++i) id_frame2id[map.frames_[i].id] = i;
  for (size_t i = 0; i < cameras_refine.size(); ++i) id_camera2id_refine[cameras_refine[i].id] = i;

  for (auto &[name, id_frame, id_camera] : image_infos) {
    CHECK(id_camera != -1) << "exit id_camera==-1";
    map.frames_[id_frame2id[id_frame]].name = name;
    map.frames_[id_frame2id[id_frame]].camera_id = id_camera;
  }

  for (auto &camera : map.cameras_) {
    if (id_camera2id_refine.count(camera.id) != 0) camera = cameras_refine[id_camera2id_refine[camera.id]];
  }

  // reorder the frame id and frame pair & compute points_normalized
  std::sort(map.cameras_.begin(), map.cameras_.end(), [](const Camera &a, const Camera &b) { return a.id < b.id; });
  std::sort(map.frames_.begin(), map.frames_.end(), [](const Frame &a, const Frame &b) { return a.id < b.id; });
  std::cout << map.cameras_.size() << " - " << map.cameras_.back().id << std::endl;
  std::cout << map.frames_.size() << " - " << map.frames_.back().id << std::endl;
  std::map<int, int> id2reorderid_camera;
  for (int i = 0; i < map.cameras_.size(); ++i) {
    id2reorderid_camera[map.cameras_[i].id] = i;
    map.cameras_[i].id = i;
  }

  std::map<int, int> id2reorderid_frame;
  for (int i = 0; i < map.frames_.size(); ++i) {
    auto &frame = map.frames_[i];
    id2reorderid_frame[frame.id] = i;
    frame.id_colmap = frame.id;
    frame.id = i;
    CHECK(id2reorderid_camera.count(frame.camera_id) == 1) << frame.id_colmap << " " << frame.camera_id;
    const int camera_id = id2reorderid_camera[frame.camera_id];
    frame.camera_id = camera_id;
    for (int k = 0; k < frame.points.size(); ++k) {
      ImageToNormalized(map.cameras_[camera_id], frame.points[k], frame.points_normalized[k]);
    }
  }
  
  for (auto &fp : map.frame_pairs_) {
    if (id2reorderid_frame.count(fp.id1) != 0 && id2reorderid_frame.count(fp.id2) != 0) {
      fp.id1 = id2reorderid_frame.at(fp.id1);
      fp.id2 = id2reorderid_frame.at(fp.id2);
    } else {
      std::cout << "error" << fp.id1 << " - " << fp.id2 << std::endl;
    }
  }
  map.RemoveRedundancyPoints();
  map.Init();
}

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  std::string config_path = "./config_uno.json";
  if (argc == 2) {
    config_path = argv[1];
  }

  nlohmann::json config_json;
  std::ifstream ifs(config_path);
  ifs >> config_json;
  ifs.close();

  const std::string data_path = config_json["data_path"];
  const std::string seq_name = config_json["seq_name"];
  const std::string output_path = config_json["output_path"];
  const double camera_size = config_json["camera_size"];
  // debug = (config_json["debug"] == 1);
  image_dir = data_path + seq_name + "/images/";

  Timer timer_tot("Time Total %.6lfs\n");
  Timer timer_reg("Time reg %.6lfs\n");
  Timer timer_tri("Time tri %.6lfs\n");
  Timer timer_fil("Time fil %.6lfs\n");
  Timer timer_mer("Time merge %.6lfs\n");
  Timer timer_che("Time check %.6lfs\n");
  Timer timer_lba("Time LBA %.6lfs\n");
  Timer timer_gba("Time GBA %.6lfs\n");
  std::vector<Timer *> timer_vec = {&timer_tot, &timer_reg, &timer_tri, &timer_fil,
                                    &timer_mer, &timer_che, &timer_lba, &timer_gba};
  Map map;
  BASolver ba_solver;
  Point3dProcessor p3d_processor;
  ViewerThread viewer;
  viewer.start();

  PreProcess(data_path + seq_name + "/model/", map);

  FramePair init_frame_pair;
  FindInitFramePair(map, init_frame_pair);
  InitializeMap(map, init_frame_pair);
  ba_solver.GBA(map);

  int num_image_reg = 2, num_image_reg_pre = 2;
  const int MAX_ITER = map.frames_.size();
  for (int iter = 0; iter < MAX_ITER; iter++) {    
    viewer.update_map(map);
    timer_tot.resume();
    printf("-----------------------------------------------\n");
    const int frame_id = map.MaxPoint3dFrameId();
    printf("Iter %d %d %s\n", iter, frame_id, map.frames_[frame_id].name.c_str());
    if (frame_id == -1) break;

    TIMING(timer_reg, if (!RegisterImage(frame_id, map)) break);

    TIMING(timer_tri, p3d_processor.TriangulateFramePoint(map, frame_id, th_angle_lba));
    TIMING(timer_fil, p3d_processor.FilterPointsFrame(map, frame_id, th_rpe_lba, th_angle_lba));
    if (p3d_processor.CheckFrameMeasurement(map, frame_id)) continue;
    TIMING(timer_lba, ba_solver.LBA(frame_id, map));
    TIMING(timer_mer, p3d_processor.MergeTracks(map, frame_id, th_rpe_lba));  // should speed up
    TIMING(timer_fil, p3d_processor.FilterPointsFrame(map, frame_id, th_rpe_lba, th_angle_lba));

    if (num_image_reg++ > 1.2 * num_image_reg_pre) {
      TIMING(timer_che, p3d_processor.CheckTrackDepth(map));  // 1.6
      TIMING(timer_gba, ba_solver.KGBA(map, std::vector<int>(0), false));
      // TIMING(timer_gba, ba_solver.GBA(map));
      TIMING(timer_fil, p3d_processor.FilterPoints3d(map, th_rpe_gba, th_angle_gba));
      num_image_reg_pre = num_image_reg;
    }

    UpdateCovisiblity(map, frame_id);
    timer_tot.stop();
    // map.LogReprojectError();
  }
  // timer_tot.print();
  // map.LogReprojectError();
  // timer_tot.resume();
  // p3d_processor.FilterPoints3d(map, th_rpe_gba, th_angle_gba);
  // ba_solver.KGBA(map, std::vector<int>(0), false);
  // // ba_solver.GBA(map);
  // p3d_processor.FilterPoints3d(map, th_rpe_gba, th_angle_gba);
  // timer_tot.stop();

  // SaveSfMState("uno_end.bin", map.frames_.size(), map);
  for (auto &timer_ptr : timer_vec) {
    timer_ptr->print();
  }
  // int iter = 0;
  // LoadSfMState("uno_end.bin", iter, map);
  // WriteColMapDataBinary("/home/yzc/Data/SfM/ECIM/level_result/", map);
  // map.LogReprojectError();
  // viewer.DrawGraph(map, true);
  // viewer.DrawKey(map, true);
  // WriteColMapDataBinary(output_path + seq_name + '/', map);
  viewer.stop();
  return 0;
}
