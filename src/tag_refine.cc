
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
    constexpr double tag_length = 0.113; 
    std::string image_dir = argv[1];
    std::string map_dir = argv[2];
    std::string output_path = argv[3];
    tag_refine(image_dir,map_dir,output_path); 
    return 0;
}
