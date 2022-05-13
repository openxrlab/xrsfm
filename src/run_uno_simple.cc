
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
#include "mapper/incremental_mapper.h"
 

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
    frame.uint_descs_.resize(0,0);
    frame.track_ids_.assign(num_points, -1);
    for (const auto& kpt : frame.keypoints_) {
      const auto& pt = kpt.pt;
      Eigen::Vector2d ept(pt.x, pt.y), eptn;
      ImageToNormalized(camera, ept, eptn);
      frame.points.emplace_back(ept);
      frame.points_normalized.emplace_back(eptn);
    }
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
  Map map;
  PreProcess(seq_path , map);  

  // 3. Map Reconstruction 
  IncrementalMapper imapper; 
  imapper.options.th_rpe_gba = 4.0;
  imapper.options.th_angle_gba = 1.5;
  imapper.Reconstruct(map);
  std::cout << "Reconstruction Done!" << std::endl;

  WriteColMapDataBinary(output_path , map); 
  
  return 0;
}
