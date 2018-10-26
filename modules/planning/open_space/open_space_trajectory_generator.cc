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

#include "modules/planning/open_space/open_space_trajectory_generator.h"

#include <fstream>
#include <utility>

#include "cyber/common/log.h"
#include "cyber/task/task.h"
#include "modules/common/util/string_tokenizer.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::common::math::Box2d;
using apollo::common::math::Vec2d;

Status OpenSpaceTrajectoryGenerator::Init(const PlanningConfig&) {
  AINFO << "In OpenSpaceTrajectoryGenerator::Init()";

  // TODO(QiL): integrate open_space planner into task config when refactor done
  CHECK(common::util::GetProtoFromFile(FLAGS_planner_open_space_config_filename,
                                       &planner_open_space_config_))
      << "Failed to load open space config file "
      << FLAGS_planner_open_space_config_filename;

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
  warm_start_.reset(new HybridAStar(planner_open_space_config_));

  // initialize distance approach class pointer
  distance_approach_.reset(
      new DistanceApproachProblem(planner_open_space_config_));

  return Status::OK();
}

apollo::common::Status OpenSpaceTrajectoryGenerator::Plan(
    const common::TrajectoryPoint& planning_init_point, Frame* frame) {
  // initial state
  init_state_ = frame->vehicle_state();
  init_x_ = init_state_.x();
  init_y_ = init_state_.y();
  init_phi_ = init_state_.heading();
  init_v_ = init_state_.linear_velocity();
  // rotate and scale the state according to the origin point defined in
  // frame
  double rotate_angle = frame->origin_heading();
  const Vec2d& translate_origin = frame->origin_point();
  init_x_ = init_x_ - translate_origin.x();
  init_y_ = init_y_ - translate_origin.y();
  double tmp_x = init_x_;
  init_x_ =
      init_x_ * std::cos(-rotate_angle) - init_y_ * std::sin(-rotate_angle);
  init_y_ = tmp_x * std::sin(-rotate_angle) + init_y_ * std::cos(-rotate_angle);

  // TODO(Jinyun) how to initial input not decided yet
  init_steer_ = 0;
  init_a_ = 0;
  Eigen::MatrixXd x0(4, 1);
  x0 << init_x_, init_y_, init_phi_, init_v_;
  Eigen::MatrixXd last_time_u(2, 1);
  last_time_u << init_steer_, init_a_;

  // final state
  const std::vector<double>& end_pose = frame->open_space_end_pose();
  Eigen::MatrixXd xF(4, 1);
  xF << end_pose[0], end_pose[1], end_pose[2], end_pose[3];

  // vertices using V-represetntation (counter clock wise)
  std::size_t obstacles_num = frame->obstacles_num();
  Eigen::MatrixXd obstacles_edges_num = frame->obstacles_edges_num();
  Eigen::MatrixXd obstacles_A = frame->obstacles_A();
  Eigen::MatrixXd obstacles_b = frame->obstacles_b();

  // Warm Start (initial velocity is assumed to be 0 for now)
  Result result;
  ThreadSafeIndexedObstacles* obstalce_list =
      frame->openspace_warmstart_obstacles();

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

  ADEBUG << "Distance approach configs set"
         << distance_approach_config_.ShortDebugString();
  // result for distance approach problem
  Eigen::MatrixXd state_result_ds;
  Eigen::MatrixXd control_result_ds;
  Eigen::MatrixXd time_result_ds;

  bool status = distance_approach_->Solve(
      x0, xF, last_time_u, horizon_, ts_, ego_, xWS, uWS, XYbounds_,
      obstacles_num, obstacles_edges_num, obstacles_A, obstacles_b,
      &state_result_ds, &control_result_ds, &time_result_ds);

  if (status) {
    ADEBUG << "Distance approach problem solved successfully!";
  } else {
    return Status(ErrorCode::PLANNING_ERROR,
                  "Distance approach problem failed to solve");
  }

  // rescale the states to the world frame
  for (std::size_t i = 0; i < horizon_ + 1; i++) {
    double tmp_x = state_result_ds(0, i);
    state_result_ds(0, i) = state_result_ds(0, i) * std::cos(rotate_angle) -
                            state_result_ds(1, i) * std::sin(rotate_angle);
    state_result_ds(1, i) = tmp_x * std::sin(rotate_angle) +
                            state_result_ds(1, i) * std::cos(rotate_angle);
    state_result_ds(0, i) += translate_origin.x();
    state_result_ds(1, i) += translate_origin.y();
    state_result_ds(2, i) += rotate_angle;
  }

  // TODO(Jiaxuan): Step 9 : trajectory Partition and  Publish trajectoryPoint
  // in planning trajectory, Result saved in current trajectory.
  // ADCTrajectory current_trajectory_,
  return Status::OK();
}

apollo::common::Status OpenSpaceTrajectoryGenerator::UpdateTrajectory(
    ADCTrajectory* current_trajectory) {
  current_trajectory->CopyFrom(current_trajectory_);
  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
