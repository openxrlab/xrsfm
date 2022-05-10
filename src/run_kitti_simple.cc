
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

int camera_param_id = -1;

void PreProcess(const std::string dir_path, Map& map) {
  std::vector<Frame> frames;
  std::vector<Camera> cameras;
  std::vector<FramePair> frame_pairs;
  ReadFeatures(dir_path + "ftr.bin", frames, true);
  ReadFramePairs(dir_path + "fp.bin", frame_pairs);

  // set cameras
  Camera seq;
  if (camera_param_id == 0) {
    seq = Camera(0, 718.856, 718.856, 607.1928, 185.27157, 0.0);  // 00-02
  } else if (camera_param_id == 1) {
    seq = Camera(0, 721.5377, 721.5377, 609.5593, 172.854, 0.0);  // 03
  } else if (camera_param_id == 2) {
    seq = Camera(0, 707.0912, 707.0912, 601.8873, 183.1104, 0.0);  // 04-12
  }
  cameras.emplace_back(seq);
  for (auto& frame : frames) {
    frame.camera_id = 0;
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
  std::string config_path = "./config_kitti.json";
  if (argc == 2) {
    config_path = argv[1];
  }
  auto config_json = LoadJSON(config_path);

  const std::string data_path = config_json["data_path"];
  const std::string seq_name = config_json["seq_name"];
  const std::string output_path = config_json["output_path"];
  const int init_id1 = config_json["init_id1"];
  const int init_id2 = config_json["init_id2"];
  const double camera_size = config_json["camera_size"];
  const bool debug = (config_json["debug"] == 1);
  const std::string seq_path = data_path + seq_name + '/';
  const std::string image_dir = seq_path + "/image_0/"; 

  std::map<std::string, int> name2camera_id = {{"00", 0}, {"01", 0}, {"02", 0}, {"03", 1}, {"04", 2}, {"05", 2},
                                               {"06", 2}, {"07", 2}, {"08", 2}, {"09", 2}, {"10", 2}};
  CHECK(name2camera_id.count(seq_name) != 0) << "NO suitable camera param.";
  camera_param_id = name2camera_id[seq_name];

  // 2. Map PreProcess 
  Map map;
  PreProcess(seq_path + "/open/nv50", map);
  std::cout << "PreProcess Done!" << std::endl;

  // 3. Map Reconstruction 
  IncrementalMapper imapper;
  imapper.options.init_id1 = init_id1;
  imapper.options.init_id2 = init_id2;
  imapper.Reconstruct(map);
  std::cout << "Reconstruction Done!" << std::endl;
   
  // 4. Output Trajectory
  std::vector<double> timestamp_vec;
  LoadTimeStamp(seq_path + "times.txt", timestamp_vec);
  UpdateFrameTimeStamp(map.frames_, timestamp_vec);
  WriteTrajectory(map, output_path + seq_name + "_test.tum");
  
  return 0;
}
