
#ifndef INCREMENTAL_MAPPER_H_
#define INCREMENTAL_MAPPER_H_

 
#include "base/map.h" 
#include "geometry/pnp.h"
#include "geometry/track_processor.h" 
#include "geometry/error_corrector.h"
#include "geometry/map_initializer.h"
#include "optimization/ba_solver.h" 
#include "utility/timer.h" 

struct IncrementalMapperOptions {
    bool correct_pose = false;
    bool stop_when_register_fail = false;
    int init_id1 = -1;
    int init_id2 = -1;
    double th_rpe_lba = 8;
    double th_angle_lba = 2.0;
    double th_rpe_gba = 8;
    double th_angle_gba = 2.0;
};

class IncrementalMapper{
public:
    IncrementalMapper();
    void Reconstruct(Map &map);
    TimerArray timer;
    BASolver ba_solver;
    Point3dProcessor p3d_processor;
    ErrorCorrector error_corrector;
    IncrementalMapperOptions options;
};



#endif  // COLMAP_SRC_CONTROLLERS_INCREMENTAL_MAPPER_H_