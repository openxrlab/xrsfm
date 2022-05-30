

#include <cctype>
#include <regex>
#include <unordered_set>

#include "feature/feature_processing.h"
#include "base/camera.h"
#include "base/map.h"
#include "utility/io_ecim.hpp"
#include "utility/timer.h"
#include "geometry/track_processor.h"
#include "optimization/ba_solver.h"

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

int main(int argc, const char* argv[]) {
  std::string bin_path;
  std::string images_path;
  std::string data_path;

  Map map;
  PreProcess(bin_path ,images_path, map);
  
  std::map<int,Frame> frame_map;
  ReadImagesBinary(data_path + "images.bin", frame_map);
  
  for(auto&[id,frame]:frame_map){
    map.frames_[id].registered = true;
    map.frames_[id].Tcw = frame.Tcw;
  }

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
  ba_solver.GBA(map);

  return 0;
}