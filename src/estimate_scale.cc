
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
    std::string images_dir = argv[1];
    std::string map_dir = argv[2];
    std::string output_path = argv[3];
    double tag_length = std::stod(argv[4]);
    tag_refine(images_dir,map_dir,output_path); 
    return 0;
}
