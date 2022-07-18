//
// Created by yzc on 2019/11/19.
//

#include <string>
#include <vector>

#include "obs.h"
#include "type_mg.h"
#include "optim/ba_block.h"
#include "optim/lm_minimizer.h"
#include "optim/local_parameterization.h" 
#include "optim/ceres_cost_function.h" 

using namespace ceres;

void AddParamblock(Problem &problem, Solver::Options &solver_options, mgba::BAGraph &ba_graph) {
  constexpr int kLandmarkGroup = 0, kOtherGroup = 1;
  solver_options.linear_solver_ordering = std::make_shared<ceres::ParameterBlockOrdering>();

  for (auto &cam : ba_graph.cam_vec) { 
    // problem.AddParameterBlock(cam.data(), 4, new ceres::QuaternionParameterization);
    problem.AddParameterBlock(cam.data(), 4, new EigenQuatParam);
    problem.AddParameterBlock(cam.data() + 4, 3);

    solver_options.linear_solver_ordering->AddElementToGroup(cam.data(), kOtherGroup);
    solver_options.linear_solver_ordering->AddElementToGroup(cam.data() + 4, kOtherGroup);
  }
 
  for (auto &pt : ba_graph.point_vec) {
    problem.AddParameterBlock(pt.data(), 3);
    solver_options.linear_solver_ordering->AddElementToGroup(pt.data(), kLandmarkGroup);
  }  

  for(int i = 0,num = ba_graph.const_cam_flags.size();i<num;++i){
    if(ba_graph.const_cam_flags[i]){
      problem.SetParameterBlockConstant(ba_graph.cam_vec[i].data());
      problem.SetParameterBlockConstant(ba_graph.cam_vec[i].data() + 4);
    }
  }
}

void AddResidualblock(Problem &problem, mgba::BAGraph &ba_graph) {
  auto &obs_vec = ba_graph.obs_vec;
  auto &cam_vec = ba_graph.cam_vec;
  auto &point_vec = ba_graph.point_vec;

  ceres::LossFunction *loss_function = nullptr;
  // if(ba_graph.use_local)loss_function = new ceres::SoftLOneLoss(1.0);
  ceres::CostFunction *cost_function = nullptr;
  for (auto &obs : obs_vec) {
    auto &cam = cam_vec[obs.camera_id];
    double *qvec_ptr = cam.data(), *tvec_ptr = cam.data() + 4;
    double *point3d_ptr = point_vec[obs.point_id].data();

    // cost_function = ProjectionCostAuto1::Create(obs.mea,obs.weight);
    // cost_function = new ProjectionCost(obs.mea);  // 4722326.796170
    // cost_function = new UnitVectorCost(obs.mea);  // 4712223.400709 
    // if(ba_graph.use_local)
    cost_function = new UnitVectorCostW(obs.mea,obs.weight);   
    // cost_function = new UnitVectorCost(obs.mea);  
    
    problem.AddResidualBlock(cost_function, loss_function, qvec_ptr, tvec_ptr, point3d_ptr);
  
  }
}

void SolveProblem(Problem &problem, Solver::Options &solver_options, mgba::BAGraph &ba_graph) {
  solver_options.minimizer_progress_to_stdout = true;
  solver_options.num_threads = 1;
  solver_options.max_num_iterations = ba_graph.max_iter;
  solver_options.max_lm_diagonal = solver_options.min_lm_diagonal = 1;
  solver_options.jacobi_scaling = false;
  // solver_options.preconditioner_type = ceres::SCHUR_JACOBI;
  std::map<int, ceres::LinearSolverType> type_map = {
      {0, ceres::DENSE_SCHUR}, {1, ceres::SPARSE_SCHUR}, {2, ceres::ITERATIVE_SCHUR}};
  solver_options.preconditioner_type = ceres::IDENTITY;
  solver_options.linear_solver_type = type_map.at(ba_graph.solve_type);
  solver_options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;

  Solver::Summary summary;
  ceres::Solve(solver_options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";
}

void mgba::RunCeres(mgba::BAGraph &ba_graph) {
  ba_graph.LogMSE();
  Problem problem;
  Solver::Options solver_options;
  AddParamblock(problem, solver_options, ba_graph);
  AddResidualblock(problem, ba_graph);
  SolveProblem(problem, solver_options, ba_graph);
  ba_graph.LogMSE();
}
