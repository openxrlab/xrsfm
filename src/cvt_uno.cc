
#include <fstream>

#include "3rdparty/json/json.hpp"
#include "geometry/essential.h"
#include "geometry/pnp.h"
#include "geometry/track_processor.h"
#include "base/camera.h"
#include "base/map.h"
#include "geometry/map_initializer.h"
#include "optimization/ba_solver.h"
#include "utility/io_ecim.hpp"
#include "utility/timer.h"
#include "utility/viewer.h"

bool debug = false;
constexpr double th_rpe_lba = 8, th_angle_lba = 2.0;
constexpr double th_rpe_gba = 4, th_angle_gba = 1.5;

std::string dir_path;
std::string image_dir;
std::set<int> bad_essential_frame_ids;

void PreProcess(const std::string dir_path, Map &map) {
  std::string dir_path1 = dir_path+ "/model/";
  std::vector<Camera> cameras_refine;
  std::vector<std::tuple<std::string, int, int>> image_infos;
  ReadFrames(dir_path1 + "frame_colmap.bin", map.frames_);
  ReadFramePairs(dir_path1 + "fp_colmap.bin", map.frame_pairs_);
  ReadCamerasColmap(dir_path1 + "camera_colmap.bin", map.cameras_);
  ReadExtraColmap(dir_path1 + "frame_extra_colmap.bin", image_infos);
  ReadCameraColmapTXT(dir_path1 + "cameras.txt", cameras_refine);


  std::vector<std::string> image_names; 
  LoadImageNames(dir_path+"/images/", image_names);
  std::sort(image_infos.begin(),image_infos.end(),[](auto&a,auto&b){
    return std::get<0>(a) < std::get<0>(b);
  });

  std::cout<<std::get<0>(image_infos[0])<<" "<<image_names.at(0)<<std::endl;
  std::cout<<image_infos.size()<<" "<<image_names.size()<<" "<<map.frames_.size()<<std::endl;
  for(auto&frame:map.frames_){
    std::cout<<frame.points.size()<<std::endl;
  }
  

}

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  std::string config_path = "./config_uno.json";
  if (argc == 2) {
    config_path = argv[1];
  }

  nlohmann::json config_json;
  std::ifstream ifs(config_path);
  ifs >> config_json;
  ifs.close();

  const std::string data_path = config_json["data_path"];
  const std::string seq_name = config_json["seq_name"];
  const std::string output_path = config_json["output_path"];
  const double camera_size = config_json["camera_size"];
  image_dir = data_path + seq_name + "/images/";

  Map map;
  // PreProcess(data_path + seq_name , map);

  ReadColMapDataBinary("/data/ECIM/SfM/ECIM_1DSFM/Alamo/open/",map);

  return 0;
}
