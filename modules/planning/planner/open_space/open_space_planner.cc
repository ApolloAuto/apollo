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

#include "modules/planning/planner/open_space/open_space_planner.h"

#include <fstream>
#include <utility>

#include "modules/common/log.h"
#include "modules/common/util/string_tokenizer.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::common::math::Box2d;
using apollo::common::math::Vec2d;

Status OpenSpacePlanner::Init(const PlanningConfig&) {
  AINFO << "In OpenSpacePlanner::Init()";
  return Status::OK();
}

apollo::common::Status OpenSpacePlanner::Plan(
    const common::TrajectoryPoint& planning_init_point, FrameOpenSpace* frame) {
  // Problem setup

  // TODO(JinYun) : cleaning up : load control configs from VehicleParam at
  // initialization

  // horizon
  std::size_t horizon = 80;

  // nominal sampling time
  float ts = 0.1;

  // load vehicle configuration
  front_to_center_ = vehicle_param_.front_edge_to_center();
  back_to_center_ = vehicle_param_.back_edge_to_center();
  left_to_center_ = vehicle_param_.left_edge_to_center();
  right_to_center_ = vehicle_param_.right_edge_to_center();
  Eigen::MatrixXd ego(4, 1);
  ego << front_to_center_, back_to_center_, left_to_center_, right_to_center_;

  // initial state
  init_state_ = frame->vehicle_state();
  init_x_ = init_state_.x();
  init_y_ = init_state_.y();
  init_phi_ = init_state_.heading();
  init_v_ = init_state_.linear_velocity();
  Eigen::MatrixXd x0(4, 1);
  x0 << init_x_, init_y_, init_phi_, init_v_;
  // std::vector<double> x0({-12, 11, 0, 0});

  // final state
  // TODO(QiL): Step 2 ： Take final state from decision / or decision level
  // when enabled.
  Eigen::MatrixXd xF(4, 1);
  xF << 0, 1.2, M_PI / 2, 0;
  // std::vector<double> xf({0, 1.2, M_PI / 2, 0});

  // vertices using V-represetntation (clock wise)
  std::size_t obstacles_num = frame->obstacles_num();
  Eigen::MatrixXd obstacles_vertices_num = frame->obstacles_vertices_num();
  std::vector<std::vector<Vec2d>> obstacles_vertices_vec =
      frame->obstacles_vertices_vec();
  Eigen::MatrixXd obstacles_A = frame->obstacles_A();
  Eigen::MatrixXd obstacles_b = frame->obstacles_b();
  // std::vector<std::vector<std::vector<double>>> obstacles_vertices_vec = {
  //     {{-20, 5}, {-1.3, 5}, {-1.3, -5}, {-20, -5}, {-20, 5}},
  //     {{1.3, 5}, {20, 5}, {20, -5}, {1.3, -5}, {1.3, 5}},
  //     {{-20, 15}, {20, 15}, {20, 11}, {-20, 11}, {-20, 15}}};

  // Eigen::MatrixXd ob1(4, 1), ob2(4, 1), ob3(4, 1);
  // [x_upper, y_upper, -x_lower, -y_lower]
  // ob1 << -1.3, 5, 20, 5;
  // ob2 << 20, 5, -1.3, 5;
  // ob3 << 20, 15, 20, -11;

  // TODO(QiL): Step 5 : Add absolute constraints (States and Controls) from
  // perception/map
  // [x_lower, x_upper, - y_lower, y_upper]
  Eigen::MatrixXd XYbounds(4, 1);
  XYbounds << -15, 15, 1, 10;

  // TODO(QiL): Step 6 ： Fromulate warmstart matrix

  // warm start variables
  Eigen::MatrixXd xWS = Eigen::MatrixXd::Zero(4, horizon + 1);
  Eigen::MatrixXd uWS = Eigen::MatrixXd::Zero(2, horizon);
  Eigen::MatrixXd timeWS = Eigen::MatrixXd::Zero(1, horizon + 1);

  warm_start_.reset(new WarmStartProblem(horizon, ts, x0, xF, XYbounds));

  Eigen::MatrixXd state_result;
  Eigen::MatrixXd control_result;
  Eigen::MatrixXd time_result;

  bool ret_status =
      warm_start_->Solve(&state_result, &control_result, &time_result);

  if (ret_status) {
    ADEBUG << "Warm start problem solved successfully!";
  } else {
    return Status(ErrorCode::PLANNING_ERROR,
                  "Warm start problem failed to solve");
  }

  // TODO(QiL): Step 8 : Formulate distance approach problem
  // solution from distance approach
  Eigen::MatrixXd xp1 = Eigen::MatrixXd::Zero(4, horizon + 1);
  Eigen::MatrixXd up1 = Eigen::MatrixXd::Zero(2, horizon);
  Eigen::MatrixXd scaleTime1 = Eigen::MatrixXd::Zero(1, horizon + 1);

  // TODO(QiL) : update the I/O to make the warm start problem and distance
  // approach problem connect
  distance_approach_.reset(new DistanceApproachProblem(
      x0, xF, horizon, ts, ego, xWS, uWS, timeWS, XYbounds, obstacles_num,
      obstacles_vertices_num, obstacles_A, obstacles_b));

  // result for distance approach problem
  Eigen::MatrixXd state_result_ds;
  Eigen::MatrixXd control_result_ds;
  Eigen::MatrixXd time_result_ds;

  bool status = distance_approach_->Solve(&state_result_ds, &control_result_ds,
                                          &time_result_ds);

  if (status) {
    ADEBUG << "Distance approach problem solved successfully!";
  } else {
    return Status(ErrorCode::PLANNING_ERROR,
                  "Distance approach problem failed to solve");
  }

  // TODO(QiL): Step 9 : Publish trajectoryPoint in planning trajectory, i.e.
  // Fullfil frame.
  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
