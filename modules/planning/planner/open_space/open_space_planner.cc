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

/**
 * @file
 **/

#include "modules/planning/planner/open_space/open_space_planner.h"

#include <fstream>
#include <utility>

#include "cybertron/common/log.h"
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
  // nominal sampling time
  ts_ = planner_open_space_config_.delta_t();

  // load vehicle configuration
  double front_to_center = vehicle_param_.front_edge_to_center();
  double back_to_center = vehicle_param_.back_edge_to_center();
  double left_to_center = vehicle_param_.left_edge_to_center();
  double right_to_center = vehicle_param_.right_edge_to_center();
  ego_.resize(4, 1);
  ego_ << front_to_center, right_to_center, back_to_center, left_to_center;
  // load xy boundary into the Plan() from configuration(before ROI is done)
  double x_max = planner_open_space_config_.warm_start_config().max_x();
  double y_max = planner_open_space_config_.warm_start_config().max_y();
  double x_min = planner_open_space_config_.warm_start_config().min_x();
  double y_min = planner_open_space_config_.warm_start_config().min_y();
  XYbounds_.resize(4, 1);
  XYbounds_ << x_min, x_max, y_min, y_max;

  // initialize warm start class pointer
  warm_start_.reset(new HybridAStar());

  return Status::OK();
}

apollo::common::Status OpenSpacePlanner::Plan(
    const common::TrajectoryPoint& planning_init_point, FrameOpenSpace* frame) {
  // Problem setup

  // initial state
  init_state_ = frame->vehicle_state();
  init_x_ = init_state_.x();
  init_y_ = init_state_.y();
  init_phi_ = init_state_.heading();
  init_v_ = init_state_.linear_velocity();
  init_steer_ = 0;
  init_a_ = 0;
  Eigen::MatrixXd x0(4, 1);
  x0 << init_x_, init_y_, init_phi_, init_v_;
  Eigen::MatrixXd last_time_u(2, 1);
  last_time_u << init_steer_, init_a_;
  // std::vector<double> x0({-12, 11, 0, 0});

  // final state
  // TODO(QiL): Step 2 ： Take final state from decision / or decision level
  // when enabled.
  Eigen::MatrixXd xF(4, 1);
  xF << 0, 1.2, M_PI / 2, 0;
  // std::vector<double> xf({0, 1.2, M_PI / 2, 0});

  // vertices using V-represetntation (clock wise)
  std::size_t obstacles_num = frame->obstacles_num();
  Eigen::MatrixXd obstacles_edges_num = frame->obstacles_edges_num();
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

  // Warm Start (initial velocity is assumed to be 0 for now)

  Result result;
  ThreadSafeIndexedObstacles* obstalce_list = frame->GetObstacleList();

  if (warm_start_->Plan(x0(0, 0), x0(1, 0), x0(2, 0), xF(0, 0), xF(1, 0),
                        xF(2, 0), obstalce_list, &result)) {
    ADEBUG << "Warm start problem solved successfully!";
  } else {
    return Status(ErrorCode::PLANNING_ERROR,
                  "Warm start problem failed to solve");
  }
  // load Warm Start result(horizon is the "N", not the size of step points)
  horizon_ = result.x.size() - 1;
  Eigen::MatrixXd xWS = Eigen::MatrixXd::Zero(4, horizon_ + 1);
  Eigen::MatrixXd uWS = Eigen::MatrixXd::Zero(2, horizon_);
  Eigen::VectorXd x = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      result.x.data(), horizon_ + 1);
  Eigen::VectorXd y = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      result.y.data(), horizon_ + 1);
  Eigen::VectorXd phi = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      result.phi.data(), horizon_ + 1);
  Eigen::VectorXd v = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      result.v.data(), horizon_ + 1);
  Eigen::VectorXd steer = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
      result.steer.data(), horizon_);
  Eigen::VectorXd a =
      Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(result.a.data(), horizon_);
  xWS.row(0) = x;
  xWS.row(1) = y;
  xWS.row(2) = phi;
  xWS.row(3) = v;
  uWS.row(0) = steer;
  uWS.row(1) = a;

  // TODO(QiL): Step 8 : Formulate distance approach problem
  // solution from distance approach
  distance_approach_.reset(new DistanceApproachProblem(
      x0, xF, last_time_u, horizon_, ts_, ego_, xWS, uWS, XYbounds_,
      obstacles_num, obstacles_edges_num, obstacles_A, obstacles_b));

  ADEBUG << "Distance approach configs set"
         << distance_approach_config_.ShortDebugString();
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
