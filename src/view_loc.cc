
#include <fstream>
 
#include "base/map.h"
#include "base/camera.h" 
#include "geometry/umeyama.h"  
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
  
  // std::set<int> bad_id = {0,9,90,130};
  for (size_t i = 0; i < slam_vec.size(); ++i) {
    Pose &pose = slam_vec[i];
    auto t = pose.center();
    Eigen::Vector3d new_p = srt.q.inverse() * (srt.scale * gt_positions[i] - srt.t);
    pose.q = pose.q*srt.q;
    pose.t = -(pose.q*new_p);
    double dist = (pose.center()-loc_vec[i].center()).norm();
    // if(dist>1.0){
    // std::cout<<i<<" "<<dist<<" "<<name_vec[i]<<std::endl;
    // bad_id.insert(i);
    // }
  }
  std::vector<Pose> view_vec = loc_vec;
  for(auto&pose:slam_vec)view_vec.push_back(pose);

 
  
  Map map;
  ReadColMapDataBinary("/data1/rokid/results/test/",map); 
  
  ViewerThread viewer;
  viewer.camera_size_ = 0.5;
  viewer.update_map_colmap(map);
  viewer.update_cameras(view_vec);
  viewer.start();
  sleep(1000);
  viewer.stop(); 
  return 0;
}
