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
 * @file
 */

#pragma once

#include <algorithm>
#include <iomanip>
#include <utility>
#include <vector>

#include "Eigen/Dense"
#include "IpIpoptApplication.hpp"
#include "IpSolveStatistics.hpp"

#include "modules/common/time/time.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/open_space/distance_approach_ipopt_interface.h"
#include "modules/planning/proto/planning.pb.h"

namespace apollo {
namespace planning {

class DistanceApproachProblem {
 public:
  explicit DistanceApproachProblem(
      Eigen::MatrixXd x0, Eigen::MatrixXd xF, Eigen::MatrixXd last_time_u,
      std::size_t horizon, float ts, Eigen::MatrixXd ego, Eigen::MatrixXd xWS,
      Eigen::MatrixXd uWS, Eigen::MatrixXd XYbounds, std::size_t obstacles_num,
      Eigen::MatrixXd obstacles_edges_num, Eigen::MatrixXd obstacles_A,
      Eigen::MatrixXd obstacles_b,
      const PlannerOpenSpaceConfig& planner_open_space_config);

  virtual ~DistanceApproachProblem() = default;

  bool Solve(Eigen::MatrixXd* state_result, Eigen::MatrixXd* control_result,
             Eigen::MatrixXd* time_result);

 private:
  // start point
  Eigen::MatrixXd x0_;

  // end point
  Eigen::MatrixXd xF_;

  // the last control command in last planning cycle
  Eigen::MatrixXd last_time_u_;

  // time horizon
  std::size_t horizon_;

  // time interval
  float ts_;

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
  std::size_t obstacles_num_;

  // obstacles_edges_num
  Eigen::MatrixXd obstacles_edges_num_;

  // obstacles_A
  Eigen::MatrixXd obstacles_A_;

  // obstacles_b
  Eigen::MatrixXd obstacles_b_;

  PlannerOpenSpaceConfig planner_open_space_config_;
};

}  // namespace planning
}  // namespace apollo
