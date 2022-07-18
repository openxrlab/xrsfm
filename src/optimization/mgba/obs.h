#ifndef _MAP_OBS_H_
#define _MAP_OBS_H_
 
#include "type_mg.h"
#include <iostream>

namespace mgba {

struct OBS {
  OBS() {
    camera_id = -1;
    point_id = -1;
    mea.setZero();
    weight = 1.0;
  }
  OBS(int cid, int pid, vector2 _mea, double _w = 1.0) {
    camera_id = cid;
    point_id = pid;
    mea = _mea;
    weight = _w;
  }

  int camera_id;
  int point_id;
  vector2 mea;
  double weight;
};

class BAGraph {
 public:
  enum SolverType { Dense, Sparse, Iterative };
  int max_iter = 50;
  int num_frame_optim = 10000; 
  SolverType solve_type = SolverType::Dense;

  std::vector<mgba::OBS> obs_vec;
  std::vector<mgba::vector<7>> cam_vec;
  std::vector<mgba::vector<3>> point_vec;
  std::vector<bool> const_cam_flags;
  std::vector<bool> const_pt_flags;
  bool use_local = false;
  
  inline default_float LogMSE() { 
    int count = 0;
    default_float mse = 0; 
    default_float rpe = 0; 
    for (const auto &obs : obs_vec) {
      const auto &cam = cam_vec[obs.camera_id];
      const auto &pw = point_vec[obs.point_id];
      const mgba::const_map<mgba::quaternion> qcw(cam.data());
      const mgba::const_map<mgba::vector3> t(cam.data() + 4);
      const mgba::vector3 pc =  qcw * (pw + t) ;
      const mgba::vector2 est =  pc.hnormalized();
      const mgba::vector2 residual =est - obs.mea;
      // mse += 3752.228571*3752.228571*residual.squaredNorm();
      // count += 2;
      mse += obs.weight*obs.weight*residual.squaredNorm(); 
      rpe += obs.weight*residual.norm(); 
      count++;
    }
    // printf("num_camera: %zu, num_point: %zu num_obs: %d, mse: %lf cost: %lf\n", cam_vec.size(), point_vec.size(), count,sqrt(0.5*mse / count),0.5*mse);
    printf("num_camera: %zu, num_point: %zu num_obs: %d, mse: %lf cost: %lf\n", cam_vec.size(), point_vec.size(), count,mse / count,0.5*mse);
    return mse / count;
  }

};

void RunMGBA(mgba::BAGraph &ba_graph);

void RunCeres(mgba::BAGraph &ba_graph);

}  // namespace mgba

#endif
