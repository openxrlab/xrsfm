
#include <fstream>

#include "3rdparty/json/json.hpp"
#include "base/map.h"
#include "base/camera.h"
#include "geometry/pnp.h"
#include "geometry/essential.h"
#include "geometry/track_processor.h"
#include "geometry/error_corrector.h"
#include "geometry/map_initializer.h"
#include "optimization/ba_solver.h"
#include "utility/io_ecim.hpp"
#include "utility/io_feature.hpp"
#include "utility/timer.h"
#include "utility/view.h"
#include "utility/viewer.h"
#include "mapper/incremental_mapper.h"
 
void PreProcess(const std::string dir_path,const std::string images_path, Map& map) {
  std::vector<Frame> frames;
  std::vector<FramePair> frame_pairs; 
  std::vector<std::string> image_names;
  ReadFeatures(dir_path + "ftr.bin", frames, true); 
  ReadFramePairs(dir_path + "fp.bin", frame_pairs); 
  LoadImageNames(images_path, image_names);

  // set cameras & image name
  std::vector<Camera> cameras;
  Camera seq =  Camera(0, 1450,1450, 960, 720, 0.0); 
  cameras.emplace_back(seq);
  for (auto& frame : frames) {
    frame.camera_id = 0;
    frame.name = image_names.at(frame.id);
  }
 
  // set points for reconstruction
  for (auto& frame : frames) {
    const int num_points = frame.keypoints_.size();
    frame.points.clear();
    frame.points_normalized.clear();
    frame.track_ids_.assign(num_points, -1);
    for (const auto& kpt : frame.keypoints_) {
      const auto& pt = kpt.pt;
      Eigen::Vector2d ept(pt.x, pt.y), eptn;
      ImageToNormalized(cameras[0], ept, eptn);
      frame.points.emplace_back(ept);
      frame.points_normalized.emplace_back(eptn);
    }
  }

  map.frames_ = frames;
  map.cameras_ = cameras;
  map.frame_pairs_ = frame_pairs; 
  map.RemoveRedundancyPoints();
  map.Init();
}

int main(int argc, char* argv[]) {
  google::InitGoogleLogging(argv[0]);
  // 1.Read Config
  std::string config_path = "./config_seq.json";
  if (argc == 2) {
    config_path = argv[1];
  }
  auto config_json = LoadJSON(config_path);

  const std::string bin_path = config_json["bin_path"]; 
  const std::string images_path = config_json["images_path"]; 
  const std::string output_path = config_json["output_path"];
  const int init_id1 = config_json["init_id1"];
  const int init_id2 = config_json["init_id2"];
 
  // 2. Map PreProcess 
  Map map;
  PreProcess(bin_path ,images_path, map);
  std::cout << "PreProcess Done!" << std::endl;

  // 3. Map Reconstruction 
  IncrementalMapper imapper;
  imapper.options.init_id1 = init_id1;
  imapper.options.init_id2 = init_id2;
  imapper.options.correct_pose = true;
  imapper.options.stop_when_register_fail = true;
  imapper.Reconstruct(map);
  std::cout << "Reconstruction Done!" << std::endl;
   
  // 4. Output Trajectory
  // std::vector<double> timestamp_vec;
  // LoadTimeStamp(seq_path + "times.txt", timestamp_vec);
  // UpdateFrameTimeStamp(map.frames_, timestamp_vec);
  // WriteTrajectory(map, output_path + seq_name + "_test.tum");

  WriteColMapDataBinary(output_path , map); 

  return 0;
}