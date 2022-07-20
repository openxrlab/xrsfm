
#include <unistd.h> 
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
#include "utility/view.h"
#include "utility/viewer.h"
#include "geometry/epipolar_geometry.hpp"

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

  // set cameras & image name
  std::vector<Camera> cameras;
  Camera seq(0, 1000,1000, 640, 360, 0.0); 
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

  ReadFramePairBinaryForTriangulation(frame_pair_path,frame_pairs);
  std::vector<FramePair> filtered_frame_pairs;
  const int num_fp = frame_pairs.size();
  int count = 0;
  for(auto&frame_pair:frame_pairs){
    const int num_matches = frame_pair.matches.size();
    if(num_matches<30)continue;
    count++;
    if(count%100==0)
    std::cout<<1.0*count/num_fp<<std::endl;
    // std::cout<<frame_pair.id1<<" "<<frame_pair.id2<<std::endl;
    auto&frame1 = frames[frame_pair.id1];
    auto&frame2 = frames[frame_pair.id2];
    std::vector<Eigen::Vector2d> points1, points2;
    for (const auto &match : frame_pair.matches) {
      points1.push_back(frame1.points[match.id1]);
      points2.push_back(frame2.points[match.id2]);
    }
    SolveFundamnetalCOLMAP(points1, points2, frame_pair);
    // std::cout<<frame_pair.inlier_num<<" "<<num_matches<<std::endl;
    if(frame_pair.inlier_num<30)continue;
    std::vector<Match> new_matches;
    for(int i = 0;i<num_matches;++i){
      if(frame_pair.inlier_mask[i])
        new_matches.push_back(frame_pair.matches[i]);
    }
    frame_pair.matches = new_matches;
    frame_pair.inlier_mask.assign(frame_pair.matches.size(),true);
    filtered_frame_pairs.push_back(frame_pair);
  }
  frame_pairs = filtered_frame_pairs;

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

  // for(auto&fp:map.frame_pairs_){
  //   std::string img_dir = "/data/2022-07-06T14-39-35/images/";
  //   auto&f1 = map.frames_[fp.id1];
  //   auto&f2 = map.frames_[fp.id2];
  //   cv::Mat img1 = cv::imread(img_dir+f1.name);
  //   cv::Mat img2 = cv::imread(img_dir+f2.name);
  //   std::cout<<f1.name<<" "<<f2.name<<" "<<fp.matches.size()<<std::endl; 
  //   DrawFeatureMatches(img1,img2, f1.points,f2.points,fp.matches);
  //   cv::waitKey();
  // }

  BASolver ba_solver;
  Point3dProcessor p3d_processor;
  // TriangulateImage
  for(auto&frame:map.frames_){
    if(frame.registered){
      p3d_processor.TriangulateFramePoint(map, frame.id,8.0);
    }else{
      std::cout<<frame.id<<" not registered\n";
    }
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


  // Remove Frames
  for (auto& frame : map.frames_) {
    int num_mea = 0;
    for (int i = 0; i < frame.track_ids_.size(); ++i) {
      if (frame.track_ids_[i] == -1) continue; 
      num_mea++;
    }
    if(num_mea==0){
      frame.registered = false;
    }
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