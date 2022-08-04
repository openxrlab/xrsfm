#include "incremental_mapper.h"

namespace xrsfm{
IncrementalMapper::IncrementalMapper(){
  
}
    
void IncrementalMapper::Reconstruct(Map &map){
  p3d_processor = Point3dProcessor(options.th_rpe_lba, options.th_angle_lba, options.th_rpe_gba, options.th_angle_lba);
  error_corrector.ba_solver_ = &ba_solver;
  error_corrector.p3d_processor_ = &p3d_processor;
  error_corrector.only_correct_with_sim3_ = false;
  // ViewerThread viewer;
  // viewer.start();
  
  timer.tot.resume();
  // 1. Map Initialization
  FramePair init_frame_pair;
  if(options.init_id1==-1|| options.init_id2==-1){
    FindInitFramePair(map, init_frame_pair);
  }else{
    init_frame_pair = FindPair(map.frame_pairs_, options.init_id1, options.init_id2); 
  } 
  std::cout << "Found Init Frame Pair Done!" << std::endl;
  InitializeMap(map, init_frame_pair); 
  ba_solver.GBA(map);

  // 2. Map Iterative Extension
  int num_image_reg = 2, num_image_reg_pre = 2;
  for (int iter = 0; iter < map.frames_.size(); iter++) {
    printf("-----------------------------------------------\n");
    // 1) Frame Pose Estimation
    timer.reg.resume();
    const int frame_id = map.MaxPoint3dFrameId();
    if (frame_id == -1) break;
    printf("Iter %d %d %s\n", iter, frame_id, map.frames_[frame_id].name.c_str());
    if (!RegisterImage(frame_id, map)){ 
      if(options.stop_when_register_fail)break;
      map.frames_[frame_id].registered_fail = true;
      continue;
    }
    map.current_frame_id_ = frame_id;
    timer.reg.stop();

    // 2) Check & Correct Frame Pose
   if(options.correct_pose)error_corrector.CheckAndCorrectPose(map, frame_id, iter);

    // 3) Map Point Estimation
    TIMING(timer.tri, p3d_processor.TriangulateFramePoint(map, frame_id, options.th_angle_lba));
    TIMING(timer.fil, p3d_processor.FilterPointsFrame(map, frame_id, options.th_rpe_lba, options.th_angle_lba));
    TIMING(timer.merge, p3d_processor.MergeTracks(map, frame_id, options.th_rpe_lba));
    if (p3d_processor.CheckFrameMeasurement(map, frame_id)) continue;

    // 4) Local Optimization 
    TIMING(timer.lba, ba_solver.LBA(frame_id, map));
    TIMING(timer.fil, p3d_processor.FilterPointsFrame(map, frame_id, options.th_rpe_lba, options.th_angle_lba));

    // 5) Global Optimization
    if (num_image_reg++ > 1.2 * num_image_reg_pre) {
      TIMING(timer.che, p3d_processor.CheckTrackDepth(map));
      p3d_processor.CheckFramesMeasurement(map, options.th_rpe_lba, options.th_angle_lba);
      TIMING(timer.gba, ba_solver.KGBA(map, std::vector<int>(0), true));
      // TIMING(timer.gba, ba_solver.GBA(map));
      TIMING(timer.fil, p3d_processor.FilterPoints3d(map, options.th_rpe_gba, options.th_angle_gba));
      num_image_reg_pre = num_image_reg;
    }

    UpdateCovisiblity(map, frame_id);
    // viewer.update_map(map);
  }
  timer.tot.stop();

  // ba_solver.GBA(map);

  for (auto& timer_ptr : timer.timer_vec) {
    timer_ptr->print();
  } 
  // sleep(1000);
  // viewer.stop();
}
}
 