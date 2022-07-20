
#include <fstream>

#include "3rdparty/json/json.hpp"
#include "base/map.h"
#include "base/camera.h"
#include "geometry/pnp.h"
#include "geometry/essential.h"
#include "geometry/track_processor.h"
#include "geometry/error_corrector.h"
#include "geometry/map_initializer.h"
#include "geometry/umeyama.h" 
#include "optimization/ba_solver.h"
#include "utility/io_ecim.hpp"
#include "utility/io_feature.hpp"
#include "utility/timer.h"
#include "utility/view.h"
#include "utility/viewer.h" 

using namespace std;

Eigen::Matrix4d readT(std::ifstream &file){
  Eigen::Matrix4d T;
  std::string line;
  for(int k = 0;k<4;++k){ 
    std::getline(file, line);
    std::stringstream ss(line);
    std::cout<<line<<std::endl;
    ss>>T(k,0)>>T(k,1)>>T(k,2)>>T(k,3);
  }
  return T;
}

vector<string> split (string s, string delimiter) {
    size_t pos_start = 0, pos_end, delim_len = delimiter.length();
    string token;
    vector<string> res;

    while ((pos_end = s.find (delimiter, pos_start)) != string::npos) {
        token = s.substr (pos_start, pos_end - pos_start);
        pos_start = pos_end + delim_len;
        res.push_back (token);
    }

    res.push_back (s.substr (pos_start));
    return res;
}


int main(int argc, char* argv[]) { 


  std::vector<std::string> name_vec;
  std::vector<Pose> slam_vec,loc_vec;
  // std::ifstream file("/data/Rokid/query/hloc.txt");
  // std::ifstream file("/data/results/hloc.txt");
  // std::ifstream file("/data/Rokid/for_cal_Ts_m/hloc.txt");
  std::ifstream file("/data/results/hloc.txt");
  CHECK(file.is_open());
  std::string line;
  while (std::getline(file, line)) {
    if (line[0] == '#') continue;
    std::stringstream ss(line);
    std::string name;
    Pose pose;
    ss >> name ;
    ss >> pose.q.w()>> pose.q.x() >> pose.q.y() >> pose.q.z() ;
    ss >> pose.t.x() >> pose.t.y() >> pose.t.z();
    // std::cout<<pose.q.coeffs().transpose()<<"|"<<pose.center().transpose()<<std::endl;
    name_vec.push_back(name);
    loc_vec.emplace_back(pose); 
    Eigen::Matrix4d T;
    auto strlist = split(name,"_");
    for(int i = 0;i<16;++i){
      T(i) = stod(strlist[i]);
    }  
    slam_vec.emplace_back(Pose(T.transpose()));
    // std::cout<<T<<"\n"<<strlist.size()<<std::endl;
  }
  file.close();


  std::vector<Eigen::Vector3d> gt_positions,in_positions;
  for(int i= 0;i<loc_vec.size();++i){
    gt_positions.push_back(slam_vec[i].center());
    in_positions.push_back(loc_vec[i].center());
  }
  benchmark::SRT srt = benchmark::umeyama(gt_positions, in_positions, false);
  std::cout<<"scale:"<<srt.scale<<std::endl;
  // srt.
  // for (size_t i = 0; i < sfm_vec.size(); ++i) {
  //   Pose &pose = sfm_vec[i];
  //   auto t = pose.center();
  //   // Eigen::Vector3d new_p = srt.q.inverse() * (srt.scale * gt_positions[i] - srt.t);
  //   Eigen::Vector3d new_p = (srt.q*pose.center()+srt.t)/ srt.scale;
  //   pose.q = pose.q*srt.q.inverse();
  //   pose.t = -(pose.q*new_p);
  // }
  std::set<int> bad_id = {0,9,90,130};
  for (size_t i = 0; i < slam_vec.size(); ++i) {
    Pose &pose = slam_vec[i];
    auto t = pose.center();
    Eigen::Vector3d new_p = srt.q.inverse() * (srt.scale * gt_positions[i] - srt.t);
    pose.q = pose.q*srt.q;
    pose.t = -(pose.q*new_p);
    double dist = (pose.center()-loc_vec[i].center()).norm();
    if(dist>1.0){
    std::cout<<i<<" "<<dist<<" "<<name_vec[i]<<std::endl;
    bad_id.insert(i);
    }
  }
  std::vector<Pose> view_vec = loc_vec;
  for(auto&pose:slam_vec)view_vec.push_back(pose);

  {
    // std::vector<Eigen::Vector3d> gt_positions,in_positions;
    gt_positions.clear();
    in_positions.clear();
    for(int i= 0;i<loc_vec.size();++i){
      if(bad_id.count(i)!=0)continue;
      gt_positions.push_back(slam_vec[i].center());
      in_positions.push_back(loc_vec[i].center());
    }
    std::cout<<gt_positions.size()<<" "<<in_positions.size()<<std::endl;
    srt = benchmark::umeyama(gt_positions, in_positions, false);
    std::cout<<"scale:"<<srt.scale<<std::endl;
    for (size_t i = 0; i < slam_vec.size(); ++i) {
      if(bad_id.count(i)!=0)continue;
      auto &pose = slam_vec[i];
      auto t = pose.center();
      Eigen::Vector3d new_p = srt.q.inverse() * (srt.scale * t - srt.t);
      pose.q = pose.q*srt.q;
      pose.t = -(pose.q*new_p);
    }
    view_vec.clear();
    for(int i= 0;i<loc_vec.size();++i){
      if(bad_id.count(i)!=0)continue;
      auto&pose = loc_vec[i];
      view_vec.push_back(pose);
    }
    for(int i= 0;i<loc_vec.size();++i){
      if(bad_id.count(i)!=0)continue;
      auto&pose = slam_vec[i]; 
      view_vec.push_back(pose);
    }
  }


  
  Map map;
  ReadColMapDataBinary("/data1/rokid/results/test/",map);
  // map.cameras_[0].camera_params[0] = 1668.559330;
  // map.cameras_[0].camera_params[2] = 960.000000;
  // map.cameras_[0].camera_params[3] = 540.000000;
  // map.cameras_[0].distort_params[0] = 0.025766;
  // // ReadColMapDataBinary("/data/Rokid/dense/sparse/",map);
  // // ReadColMapDataBinary("/data1/rokid/results/resize/",map);
  // // for(auto&[id,frame]:map.frame_map_){
  // //   frame.Tcw.t /=srt.scale;
  // // }
  // // for(auto&[id,track]:map.track_map_){
  // //   track.point3d_ /=srt.scale;
  // // }
  // WriteColMapDataBinary2("/data1/rokid/results/test/",map);
  
  ViewerThread viewer;
  viewer.camera_size_ = 0.5;
  viewer.update_map_colmap(map);
  viewer.update_cameras(view_vec);
  viewer.start();
  sleep(1000);
  viewer.stop(); 
  return 0;
}

// int main(int argc, char* argv[]) { 
//   Map map;
//   // ReadColMapDataBinary("/data/Rokid/dense/sparse/",map);
//   ReadColMapDataBinary("/data1/rokid/results/sfm/",map);

//   std::vector<Pose> pose_vec;
//   // std::ifstream file("/data/Rokid/query/hloc.txt");
//   std::ifstream file("/home/yzc/Downloads/for_cal_Ts_m/20220708111654.log");
//   CHECK(file.is_open());
//   std::string line;
//   while (std::getline(file, line)) {
//     if (line[0] == '#') continue;
//     std::getline(file, line);
//     std::cout<<line<<std::endl;
//     Eigen::Matrix4d T = readT(file);
//     std::getline(file, line);
//     T = readT(file);
//     std::getline(file, line);
//     readT(file);
//     std::getline(file, line);
//     std::cout<<"-"<<T<<std::endl;
//     pose_vec.push_back(Pose(T));
//   }
//   file.close();
  
//   ViewerThread viewer;
//   viewer.update_map_colmap(map);
//   viewer.update_cameras(pose_vec);
//   viewer.start();
//   sleep(1000);
//   viewer.stop(); 
//   return 0;
// }