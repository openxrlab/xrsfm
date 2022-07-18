//
// Created by SENSETIME\yezhichao1 on 2020/7/17.
//

#ifndef WMAP_LM_MINIMIZER_H
#define WMAP_LM_MINIMIZER_H

#include "linear_solver.h"
namespace mgba {
class LMMinimizer {
 public:
  LMMinimizer() = default;

  void init(int _max_iter = 20);
  
  void solve_problem();

  linear_solver_refine solver;

 private:
  void StepRejected(double step_quality);

  void StepAccepted(double step_quality);

  bool ParameterToleranceReached(double step_norm, double x_norm);

  bool FunctionToleranceReached(double x_cost, double candidate_cost);

  int max_iter = 100;

  double radius;
  double decrease_factor;
  double parameter_tolerance;
  double function_tolerance;
};
}  // namespace mgba
#endif  // WMAP_LM_MINIMIZER_H
