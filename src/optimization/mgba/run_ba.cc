//
// Created by SENSETIME\yezhichao1 on 2020/4/19.
//

#include <string>
#include <vector>

#include "obs.h"
#include "type_mg.h"
#include "optim/ba_block.h"
#include "optim/lm_minimizer.h"
#include "optim/local_parameterization.h" 

using m_float = mgba::default_float;
 
void AddParamblock(mgba::BAGraph &ba_graph, std::vector<mgba::ParamBlock<m_float>> &param_blocks) {
  for (auto &cam : ba_graph.cam_vec) {
    param_blocks.emplace_back(mgba::ParamBlock<m_float>(cam.data(), 4, new mgba::EQuatParam));
    param_blocks.emplace_back(mgba::ParamBlock<m_float>(cam.data() + 4, 3));
  }
  for (auto &pt : ba_graph.point_vec) {
    param_blocks.emplace_back(mgba::ParamBlock<m_float>(pt.data(), 3));
  }
}

void AddResidualblock(mgba::BAGraph &ba_graph, std::vector<mgba::ParamBlock<m_float>> &param_blocks,
                      std::vector<mgba::ResidualBlock<m_float>> &residual_blocks) {
  residual_blocks.resize(ba_graph.obs_vec.size());
  int count = 0;
  const int num_cam = ba_graph.cam_vec.size();
  for (auto &obs : ba_graph.obs_vec) {

    auto &rb = residual_blocks[count++];
    rb.factor = std::make_unique<mgba::ProjectionFactor<m_float>>(obs.mea);
    rb.param_blocks = {
        param_blocks[obs.camera_id * 2].param_ptr,
        param_blocks[obs.camera_id * 2 + 1].param_ptr,
        param_blocks[num_cam * 2 + obs.point_id].param_ptr,
    };
    rb.param_blocks_candidate = {
        param_blocks[obs.camera_id * 2].param_new.data(),
        param_blocks[obs.camera_id * 2 + 1].param_new.data(),
        param_blocks[num_cam * 2 + obs.point_id].param_new.data(),
    };
  }
}

void SolveProblem(mgba::BAGraph &ba_graph, std::vector<mgba::ParamBlock<m_float>> &param_blocks,
                  std::vector<mgba::ResidualBlock<m_float>> &residual_blocks) {
  auto &obs_vec = ba_graph.obs_vec;
  auto &cam_vec = ba_graph.cam_vec;
  auto &point_vec = ba_graph.point_vec;

  std::vector<std::pair<int, int>> edge_vec;
  edge_vec.resize(obs_vec.size());
  for (int i = 0; i < obs_vec.size(); ++i) {
    edge_vec[i] = {obs_vec[i].camera_id, obs_vec[i].point_id};
  }

  mgba::LMMinimizer lm;
  lm.init(ba_graph.max_iter);
  lm.solver.edge_vec = edge_vec;
  lm.solver.param_blocks = std::move(param_blocks);
  lm.solver.residual_blocks = std::move(residual_blocks);
  lm.solver.const_cam_flags = ba_graph.const_cam_flags;
  lm.solver.const_pt_flags = ba_graph.const_pt_flags;
  lm.solver.init(edge_vec.size(), cam_vec.size(), point_vec.size());
  lm.solve_problem();
}

void mgba::RunMGBA(mgba::BAGraph &ba_graph) {
  ba_graph.LogMSE();
  std::vector<mgba::ParamBlock<m_float>> param_blocks;
  std::vector<mgba::ResidualBlock<m_float>> residual_blocks;
  AddParamblock(ba_graph, param_blocks);                     
  AddResidualblock(ba_graph, param_blocks, residual_blocks); 
  SolveProblem(ba_graph, param_blocks, residual_blocks);
  ba_graph.LogMSE();
}
