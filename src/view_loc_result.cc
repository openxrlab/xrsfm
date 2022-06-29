
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



int main(int argc, char* argv[]) { 
  Map map;
  ReadColMapDataBinary("/data/Rokid/dense/sparse/",map);

  std::vector<Pose> pose_vec;
  std::ifstream file("/data/Rokid/query/hloc.txt");
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
    pose_vec.emplace_back(pose);
  }
  file.close();
  
  ViewerThread viewer;
  viewer.update_map_colmap(map);
  viewer.update_cameras(pose_vec);
  viewer.start();
  sleep(1000);
  viewer.stop(); 
  return 0;
}
