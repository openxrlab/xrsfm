//
// Created by SENSETIME\yezhichao1 on 2020/7/17.
//

#include "lm_minimizer.h"

#include <Eigen/Eigen>
#include <iostream>
#include <memory>
#include <vector>

#include "../timer.h"

using namespace mgba;

Timer total_timer("Time Total %.6lfs\n");

void LMMinimizer::init(int _max_iter) {
  radius = 1e4;
  decrease_factor = 2;
  function_tolerance = 1e-6;
  parameter_tolerance = 1e-8;
  max_iter = _max_iter;

  radius = 1e6;
  function_tolerance = 1e-4;
  parameter_tolerance = 1e-5;
}

void LMMinimizer::solve_problem() {
  total_timer.start();
  int iter_num = 0;
  int step_invalid_number = 0;
  while (iter_num++ < max_iter) {
    if (step_invalid_number >= 5) break;
    solver.solve_linear_problem(radius);  // compute trust region step
    solver.compute_model_cost_change();   // compute model cost
    // printf("Iter %d model_cost_change_: %e\n", iter_num,solver.model_cost_change_);
    // printf("diagnal: %e\n", sqrt(1.0 / radius));
    // printf("model_cost_change_: %e\n", solver.model_cost_change_);
    // printf("relative_decrease_: %e\n", solver.relative_decrease_);

    bool step_valid = solver.model_cost_change_ > 0.0;
    if (!step_valid) {
      step_invalid_number++;
      StepRejected(0);
      continue;
    }
    solver.compute_relative_decrease();
    step_valid = solver.relative_decrease_ > 1e-3;
    if (!step_valid) {
      step_invalid_number++;
      StepRejected(0);
      continue;
    }
    double step_norm, x_norm;
    solver.compute_step_norm_x_norm(step_norm, x_norm);
    if (ParameterToleranceReached(step_norm, x_norm)) {
      break;
    }
    if (FunctionToleranceReached(solver.current_cost_, solver.candidate_cost_)) {
      break;
    }
    solver.param_update();
    double step_quality = solver.relative_decrease_;
    StepAccepted(step_quality);
  }
  // total_timer.stop_and_log();
}

void LMMinimizer::StepRejected(double step_quality) {
  radius = radius / decrease_factor;
  radius = std::min(1e16, radius);
  radius = std::max(1e-32, radius);
  decrease_factor *= 2.0;
}

void LMMinimizer::StepAccepted(double step_quality) {
  radius = radius / std::max(1.0 / 3.0, 1.0 - std::pow(2.0 * step_quality - 1.0, 3));
  radius = std::min(1e16, radius);
  radius = std::max(1e-32, radius);
  decrease_factor = 2.0;
}

bool LMMinimizer::ParameterToleranceReached(double step_norm, double x_norm) {
  const double step_size_tolerance = parameter_tolerance * (x_norm + parameter_tolerance);
  //    std::cout<<step_norm<<"||"<<step_size_tolerance <<std::endl;
  if (step_norm > step_size_tolerance) {
    return false;
  }
  //    printf("Parameter tolerance reached. \n");
  return true;
}

bool LMMinimizer::FunctionToleranceReached(double x_cost, double candidate_cost) {
  double cost_change = x_cost - candidate_cost;
  const double absolute_function_tolerance = function_tolerance * x_cost;

  if (std::abs(cost_change) > absolute_function_tolerance) {
    return false;
  }
  //    printf("Function tolerance reached. %e\n",cost_change/x_cost);
  return true;
}
