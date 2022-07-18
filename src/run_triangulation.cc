

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

void PreProcess(const std::string pose_path,const std::string feature_path,const std::string frame_pair_path, Map& map) {
  std::vector<Frame> frames;
  std::vector<FramePair> frame_pairs; 

  std::map<int,Frame> frames_pose,frames_pt;
  ReadImagesBinary(pose_path, frames_pose);
  ReadImagesBinaryForTriangulation(feature_path,frames_pt);
  std::cout<<frames_pose.size()<<" "<<frames_pt.size()<<std::endl;
  
  int max_id = -1;
  for(auto&[id,frame]:frames_pt){
    max_id = std::max(max_id,id);
  }
  frames.resize(max_id+1);
  for(int id = 0;id<frames.size();++id){
    auto&frame = frames[id];
    if(frames_pose.count(id)==0){
      frame.id = id;
      frame.registered = false;
    }else{
      frame = frames_pt[id];
      frame.Tcw = frames_pose[id].Tcw;
      frame.registered = true;
    }
  }
 
  ReadFramePairBinaryForTriangulation(frame_pair_path,frame_pairs);
 
  // set cameras & image name
  std::vector<Camera> cameras;
  Camera seq =  Camera(0, 1000,1000, 640, 360, 0.0); 
  // Camera seq =  Camera(0, 1450,1450, 960, 720, 0.0); 
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
  std::string config_path = "config_tri.json";
  auto config_json = LoadJSON(config_path);
  std::string pose_path = config_json["pose_path"];
  std::string feature_path = config_json["feature_path"];
  std::string frame_pair_path = config_json["frame_pair_path"];
  std::string output_path = config_json["output_path"]; 

  Map map;
  PreProcess(pose_path,feature_path ,frame_pair_path, map);
  std::cout<<"ok\n";

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