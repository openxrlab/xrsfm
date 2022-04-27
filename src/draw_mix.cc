
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

  int iter = 0;
  LoadSfMState("seq_end.bin", iter, map);

  int last_id = -1;
  for (const auto &frame : map.frames_) {
    bool is_seq = frame.camera_id == map.cameras_.back().id;
    if (!is_seq) continue;

    std::string tmp = frame.name.substr(0, frame.name.size() - 4);
    int id = std::stoi(tmp);
    if (id != last_id + 1) std::cout << id << std::endl;
    if (!frame.registered) std::cout << id << " -" << std::endl;
    last_id = id;
  }

  viewer.DrawColor(map, true);

  return 0;
}
