
#include <fstream>
#include <unistd.h>

#include "base/map.h"
#include "base/camera.h"
#include "geometry/track_processor.h"
#include "optimization/ba_solver.h"
#include "optimization/cost_factor_ceres.h"
#include "utility/io_ecim.hpp"
#include "utility/timer.h"
#include "tag/tag_extract.hpp"
#include "utility/viewer.h"


using namespace xrsfm;
 
int main(int argc, char *argv[]) {
    google::InitGoogleLogging(argv[0]);
    std::string map_dir = "/data1/Rokid/f1/test/sfm/";
    std::string image_dir = "/data1/Rokid/f1/images1/";
    Map map;
    ReadColMapDataBinary(map_dir,map);
    std::vector<std::string> image_names;
    for(auto&[id,frame]:map.frame_map_){
        image_names.emplace_back(frame.name);
    }
    auto tag_info_vec = tag_extract(image_dir,image_names);
    for(auto&[name,tag_info]:tag_info_vec){
        std::cout<<name<<" "<<tag_info.size()<<"\n";
        for(auto&[tag_id,pts]:tag_info){
            std::cout<<tag_id;
            for (int i = 0; i < 4; ++i) {
                std::cout<<" "<<pts[i].x()<<" "<<pts[i].y(); 
            } 
            std::cout<<"\n";
        }
    }

    return 0;
}
