
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
#include "utility/io_feature.hpp"
#include "utility/timer.h"
#include "utility/viewer.h"
 

void PreProcess(const std::string dir_path, Map& map) {
  std::vector<Frame> frames;
  std::vector<Camera> cameras;
  std::vector<FramePair> frame_pairs;
  std::vector<std::string> image_names;   
  std::map<std::string,int> name2cid;
  LoadImageNames(dir_path+"/images/", image_names);
  ReadFeatures(dir_path + "/open/ftr.bin", frames, true);
  // ReadFramePairs(dir_path + "/open/nv25_fp.bin", frame_pairs);
  ReadFramePairs(dir_path + "/open/ours_fp.bin", frame_pairs);
  ReadCameraInfo(dir_path+"/open/camera_info.txt",name2cid,cameras);
  assert(image_names.size()==frames.size());
  assert(cameras.size()==frames.size());

  for (int i =0;i<frames.size();++i) {
    auto& frame = frames.at(i);
    frame.id = i;
    frame.name = image_names.at(i);
    frame.camera_id = name2cid.at(frame.name);
    const auto & camera = cameras.at(frame.camera_id);
    const int num_points = frame.keypoints_.size();
    frame.points.clear();
    frame.points_normalized.clear();
    for (const auto& kpt : frame.keypoints_) {
      const auto& pt = kpt.pt;
      Eigen::Vector2d ept(pt.x, pt.y), eptn;
      ImageToNormalized(camera, ept, eptn);
      frame.points.emplace_back(ept);
      frame.points_normalized.emplace_back(eptn);
    }
    frame.track_ids_.assign(num_points, -1);
  }

  map.frames_ = frames;
  map.cameras_ = cameras;
  map.frame_pairs_ = frame_pairs;
  map.RemoveRedundancyPoints();
  map.Init();
  std::cout << "PreProcess Done!" << std::endl;
  printf("Num Frames: %d Num Pairs %d\n",map.frames_.size(),map.frame_pairs_.size());
}

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  // 1.Read Config
  std::string config_path = "./config_uno.json";
  if (argc == 2) {
    config_path = argv[1];
  }
  auto config_json = LoadJSON(config_path);

  const std::string data_path = config_json["data_path"];
  const std::string seq_name = config_json["seq_name"];
  const std::string output_path = config_json["output_path"];
  const double camera_size = config_json["camera_size"]; 
  const std::string seq_path = data_path + seq_name + '/'; 
  constexpr double th_rpe_lba = 8, th_angle_lba = 2.0;
  constexpr double th_rpe_gba = 4, th_angle_gba = 1.5;

  // 2.Initialize Program
  TimerArray timer;
  // ViewerThread viewer;
  // viewer.start();

  Map map;
  BASolver ba_solver;
  Point3dProcessor p3d_processor(th_rpe_lba, th_angle_lba, th_rpe_gba, th_angle_lba);

  // 3.Read & PreProcess Data
  PreProcess(seq_path , map);  

  // 4.Initialize Map
  timer.tot.resume();
  FramePair init_frame_pair;
  FindInitFramePair(map, init_frame_pair);
  InitializeMap(map, init_frame_pair);
  map.cameras_[init_frame_pair.id1].log();
  map.cameras_[init_frame_pair.id2].log();
  ba_solver.GBA(map);
  timer.tot.stop();

  int num_image_reg = 2, num_image_reg_pre = 2;
  for (int iter = 0; iter < map.frames_.size(); iter++) {
    // viewer.update_map(map);

    timer.tot.resume();
    // 5.Register Frame
    timer.reg.resume();
    printf("-----------------------------------------------\n");
    int frame_id = map.MaxPoint3dFrameId();
    if (frame_id == -1)break;
    if (!RegisterImage(frame_id, map)){
      printf("Fail to Register\n");
      map.frames_[frame_id].registered_fail = true;
      continue;
      // break;
    }
    map.current_frame_id_ = frame_id;
    printf("Iter %d %d %s\n", iter, frame_id, map.frames_[frame_id].name.c_str());
    timer.reg.stop();
 

    // 7.Expand & Optimize Map
    TIMING(timer.tri, p3d_processor.TriangulateFramePoint(map, frame_id, th_angle_lba));
    TIMING(timer.fil, p3d_processor.FilterPointsFrame(map, frame_id, th_rpe_lba, th_angle_lba));
    TIMING(timer.merge, p3d_processor.MergeTracks(map, frame_id, th_rpe_lba));
    if (p3d_processor.CheckFrameMeasurement(map, frame_id)) continue;
    TIMING(timer.lba, ba_solver.LBA(frame_id, map));
    TIMING(timer.fil, p3d_processor.FilterPointsFrame(map, frame_id, th_rpe_lba, th_angle_lba));

    if (num_image_reg++ > 1.2 * num_image_reg_pre) {
      TIMING(timer.che, p3d_processor.CheckTrackDepth(map));
      p3d_processor.CheckFramesMeasurement(map, th_rpe_lba, th_angle_lba);
      TIMING(timer.gba, ba_solver.KGBA(map, std::vector<int>(0), true));
      TIMING(timer.fil, p3d_processor.FilterPoints3d(map, th_rpe_gba, th_angle_gba));
      num_image_reg_pre = num_image_reg;
    }

    UpdateCovisiblity(map, frame_id);
    timer.tot.stop();
  }

  ba_solver.GBA(map);

  for (auto& timer_ptr : timer.timer_vec) {
    timer_ptr->print();
  } 
  WriteColMapDataBinary(output_path , map); 
  // viewer.stop();
  return 0;
}
