/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/*
 * spiral_reference_line_smoother.h
 */

#ifndef MODULES_PLANNING_PLANNER_OPEN_SPACE_DISTANCE_APPROACH_PROBLEM_H_
#define MODULES_PLANNING_PLANNER_OPEN_SPACE_DISTANCE_APPROACH_PROBLEM_H_

#include <vector>

#include "Eigen/Dense"

#include "modules/planning/planner/open_space/distance_approach_ipopt_interface.h"
#include "modules/planning/proto/planning.pb.h"

namespace apollo {
namespace planning {

class DistanceApproachProblem {
 public:
  explicit DistanceApproachProblem(
      Eigen::MatrixXd x0, Eigen::MatrixXd xF, std::size_t horizon, float ts,
      float wheelbase_length, Eigen::MatrixXd ego, Eigen::MatrixXd xWS,
      Eigen::MatrixXd uWS, Eigen::MatrixXd timeWS, Eigen::MatrixXd XYbounds,
      int nOb, Eigen::MatrixXd vOb, Eigen::MatrixXd AOb, Eigen::MatrixXd bOb);

  virtual ~DistanceApproachProblem() = default;

  bool Solve(std::vector<double>* x1_result, std::vector<double>* x2_result,
             std::vector<double>* x3_result, std::vector<double>* x4_result,
             std::vector<double>* u1_result, std::vector<double>* u2_result,
             std::vector<double>* t_result);

 private:
  // start point
  Eigen::MatrixXd x0_;

  // end point
  Eigen::MatrixXd xF_;

  // time horizon
  std::size_t horizon_;

  // time interval
  float ts_;

  // wheelbase_length
  float wheelbase_length_;

  // ego car dimension
  Eigen::MatrixXd ego_;

  // xWs from warm start problem
  Eigen::MatrixXd xWS_;

  // uWs from warm start problem
  Eigen::MatrixXd uWS_;

  // timeWS from warm start problem
  Eigen::MatrixXd timeWS_;

  // XY bounds
  Eigen::MatrixXd XYbounds_;

  // number of obstacles
  int nOb_;

  // vOb
  Eigen::MatrixXd vOb_;

  // AOb
  Eigen::MatrixXd AOb_;

  // bOb
  Eigen::MatrixXd bOb_;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_PLANNER_OPEN_SPACE_DISTANCE_APPROACH_PROBLEM_H_
