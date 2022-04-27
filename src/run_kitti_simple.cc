
#include <fstream>

#include "3rdparty/json/json.hpp"
#include "geometry/essential.h"
#include "geometry/pnp.h"
#include "geometry/track_processor.h"
#include "base/camera.h"
#include "geometry/error_corrector.h"
#include "base/map.h"
#include "geometry/map_initializer.h"
#include "optimization/ba_solver.h"
#include "utility/io_ecim.hpp"
#include "utility/io_feature.hpp"
#include "utility/timer.h"
#include "utility/view.h"
#include "utility/viewer.h"

int camera_param_id = -1;

void PreProcess(const std::string dir_path, Map& map) {
  std::vector<Frame> frames;
  std::vector<Camera> cameras;
  std::vector<FramePair> frame_pairs;
  ReadFeatures(dir_path + "ftr.bin", frames, true);
  ReadFramePairs(dir_path + "fp.bin", frame_pairs);

  // set map camera
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
    const int num_points = frame.keypoints_.size();
    frame.camera_id = 0;
    frame.points.clear();
    frame.points_normalized.clear();
    for (const auto& kpt : frame.keypoints_) {
      const auto& pt = kpt.pt;
      Eigen::Vector2d ept(pt.x, pt.y), eptn;
      ImageToNormalized(cameras[0], ept, eptn);
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
  constexpr double th_rpe_lba = 8, th_angle_lba = 2.0;
  constexpr double th_rpe_gba = 8, th_angle_gba = 2.0;

  std::map<std::string, int> name2camera_id = {{"00", 0}, {"01", 0}, {"02", 0}, {"03", 1}, {"04", 2}, {"05", 2},
                                               {"06", 2}, {"07", 2}, {"08", 2}, {"09", 2}, {"10", 2}};
  CHECK(name2camera_id.count(seq_name) != 0) << "NO camera param.";
  camera_param_id = name2camera_id[seq_name];

  // 2.Initialize Program
  TimerArray timer;
  // ViewerThread viewer;
  // viewer.start();

  Map map;
  BASolver ba_solver;
  Point3dProcessor p3d_processor(th_rpe_lba, th_angle_lba, th_rpe_gba, th_angle_lba);
  ErrorCorrector error_corrector(&ba_solver, &p3d_processor);
  error_corrector.only_correct_with_sim3_ = false;
  error_corrector.error_detector.Init(image_dir, debug);

  // 3.Read & PreProcess Data
  PreProcess(seq_path + "/open/nv50", map);
  std::vector<double> timestamp_vec;
  LoadTimeStamp(seq_path + "times.txt", timestamp_vec);
  UpdateFrameTimeStamp(map.frames_, timestamp_vec);
  // SaveMap(output_path + seq_name + "/", map);

  // 4.Initialize Map
  timer.tot.resume();
  FramePair init_frame_pair = FindPair(map.frame_pairs_, init_id1, init_id2);
  // FramePair init_frame_pair;
  // FindInitFramePair(map, init_frame_pair);
  InitializeMap(map, init_frame_pair);
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
    if (frame_id == -1) break;
    if (!RegisterImage(frame_id, map)) break;
    map.current_frame_id_ = frame_id;
    printf("Iter %d %d %s\n", iter, frame_id, map.frames_[frame_id].name.c_str());
    timer.reg.stop();

    // 6.Check & Correct Frame Pose
    error_corrector.CheckAndCorrectPose(map, frame_id, iter);

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
  // ba_solver.lba1.print();
  // ba_solver.lba2.print();
  // ba_solver.lba3.print();

  // int iter = 0;
  // SaveSfMState(output_path + seq_name + "/end.bin", iter, map);
  // WriteColMapDataBinary(output_path + seq_name + '/', map);
  WriteTrajectory(map, output_path + seq_name + "_test.tum");
  // viewer.stop();
  return 0;
}
