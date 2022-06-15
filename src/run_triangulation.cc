

#include <cctype>
#include <regex>
#include <unordered_set>

#include "feature/feature_processing.h"
#include "base/camera.h"
#include "base/map.h"
#include "geometry/track_processor.h"
#include "optimization/ba_solver.h"
#include "utility/timer.h"
#include "utility/io_ecim.hpp"
#include "utility/viewer.h"

void PreProcess(const std::string dir_path,const std::string images_path, Map& map) {
  std::vector<Frame> frames;
  std::vector<FramePair> frame_pairs; 

  std::map<int,Frame> frames_pt,frames_pose;
  ReadImagesBinaryForTriangulation("/data/cowtransfer/matching/feat_superpoint_1024.bin",frames_pt);
  ReadImagesBinary("/data/2022-05-18T16-28-58/refine/images.bin", frames_pose);
  for(auto&[id,frame]:frames_pt){
    frame.registered = true;
    frame.Tcw = frames_pose[id].Tcw;
    std::cout<<id<<" "<<frame.id<<std::endl;
    frames.push_back(frame);
  }
  ReadFramePairBinaryForTriangulation("/data/cowtransfer/matching/match_nn_cross_07.bin",frame_pairs);
  
  // set cameras & image name
  std::vector<Camera> cameras;
  Camera seq =  Camera(0, 1450,1450, 960, 720, 0.0); 
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
  // map.RemoveRedundancyPoints();
  map.Init();
}

int main(int argc, const char* argv[]) {
  std::string bin_path;
  std::string images_path;
  std::string output_path = "/data/2022-05-18T16-28-58/retri/"; 


  Map map;
  PreProcess(bin_path ,images_path, map);

  BASolver ba_solver;
  Point3dProcessor p3d_processor;
  // TriangulateImage
  for(auto&frame:map.frames_){
    p3d_processor.TriangulateFramePoint(map, frame.id,2.0);
  }

  // Complete Tracks
  const int num_track = map.tracks_.size();
  for(int track_id = 0;track_id<num_track;++track_id){
    if(map.tracks_[track_id].outlier)continue;
    p3d_processor.ContinueTrack(map,track_id,8.0);
  }
  
  // Merge Tracks
  for(int track_id = 0;track_id<num_track;++track_id){
    if(map.tracks_[track_id].outlier)continue;
    p3d_processor.MergeTrack(map,track_id,8.0);
  }

  // GBA
  ba_solver.GBA(map,true,true);

  WriteColMapDataBinary(output_path,map);

  ViewerThread viewer;
  viewer.start();
  viewer.update_map(map);
  sleep(1000);
  viewer.stop();
  
  
  return 0;
}