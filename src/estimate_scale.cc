
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
    std::string images_path,map_path,output_path;
    if (argc == 4){
        images_path = argv[1];
        map_path = argv[2];
        output_path = argv[3];
    }else{
        exit(-1);
    }

    // double tag_length = std::stod(argv[4]);
    constexpr double tag_length = 0.113;
    tag_refine(images_path,map_path,tag_length,output_path); 
    return 0;
}
