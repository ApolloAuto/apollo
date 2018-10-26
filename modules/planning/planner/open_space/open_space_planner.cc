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

Status OpenSpacePlanner::Init(const PlanningConfig&) {
  AINFO << "In OpenSpacePlanner::Init()";

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

  if (FLAGS_enable_open_space_planner_thread) {
    task_future_ =
        cyber::Async(&OpenSpacePlanner::GenerateTrajectoryThread, this);
  }

  return Status::OK();
}

apollo::common::Status OpenSpacePlanner::Plan(
    const common::TrajectoryPoint& planning_init_point, Frame* frame) {
  // 1. Check if exist trajectory. If yes, do collision check.
  // 1a. If collision check failed, publish stop trajectory.
  // 1b. If collision check passed, publish trajectory.
  {
    std::lock_guard<std::mutex> lock(open_space_mutex_);

    BuildPredictedEnvironment(frame->obstacles());
    if (IsCollisionFreeTrajectory(current_trajectory_)) {
      frame->mutable_trajectory()->CopyFrom(current_trajectory_);
      return Status::OK();
    } else {
      // If collision happens, return wrong planning status and estop trajectory
      // would be sent in std planning
      return Status(ErrorCode::PLANNING_ERROR, "Collision Check failed");
    }
  }
}

apollo::common::Status OpenSpacePlanner::GenerateTrajectoryThread(
    const common::TrajectoryPoint& planning_init_point, Frame* frame) {
  while (!is_stop_) {
    {
      std::lock_guard<std::mutex> lock(open_space_mutex_);
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
      init_y_ =
          tmp_x * std::sin(-rotate_angle) + init_y_ * std::cos(-rotate_angle);

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
      obstacles_num_ = frame->obstacles_num();
      obstacles_edges_num_ = frame->obstacles_edges_num();
      obstacles_A_ = frame->obstacles_A();
      obstacles_b_ = frame->obstacles_b();

      // Warm Start (initial velocity is assumed to be 0 for now)
      Result result;
      ThreadSafeIndexedObstacles* obstalce_list =
          frame->openspace_warmstart_obstacles();
    }

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
    Eigen::VectorXd a = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(
        result.a.data(), horizon_);
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
        obstacles_num_, obstacles_edges_num_, obstacles_A_, obstacles_b_,
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
    // ADCTrajectory current_trajectory_, need to add mutex lock here;
  }
}

void OpenSpacePlanner::Stop() {
  is_stop_ = true;
  if (FLAGS_enable_open_space_planner_thread) {
    task_future_.get();
  }
}

bool OpenSpacePlanner::IsCollisionFreeTrajectory(
    const ADCTrajectory& adc_trajectory) {
  CHECK_LE(adc_trajectory.trajectory_point().size(),
           predicted_bounding_rectangles_.size());
  const auto& vehicle_config =
      common::VehicleConfigHelper::Instance()->GetConfig();
  double ego_length = vehicle_config.vehicle_param().length();
  double ego_width = vehicle_config.vehicle_param().width();

  for (std::size_t i = 0; i < adc_trajectory.size(); ++i) {
    const auto& trajectory_point = adc_trajectory.at(i);
    double ego_theta = trajectory_point.path_point().theta();
    Box2d ego_box(
        {trajectory_point.path_point().x(), trajectory_point.path_point().y()},
        ego_theta, ego_length, ego_width);
    double shift_distance =
        ego_length / 2.0 - vehicle_config.vehicle_param().back_edge_to_center();
    Vec2d shift_vec{shift_distance * std::cos(ego_theta),
                    shift_distance * std::sin(ego_theta)};
    ego_box.Shift(shift_vec);

    for (const auto& obstacle_box : predicted_bounding_rectangles_[i]) {
      if (ego_box.HasOverlap(obstacle_box)) {
        return true;
      }
    }
  }
  return false;
}

void OpenSpacePlanner::BuildPredictedEnvironment(
    const std::vector<const Obstacle*>& obstacles) {
  CHECK(predicted_bounding_rectangles_.empty());
  double relative_time = 0.0;
  while (relative_time < FLAGS_trajectory_time_length) {
    std::vector<Box2d> predicted_env;
    for (const Obstacle* obstacle : obstacles) {
      TrajectoryPoint point = obstacle->GetPointAtTime(relative_time);
      Box2d box = obstacle->GetBoundingBox(point);
      predicted_env.push_back(std::move(box));
    }
    predicted_bounding_rectangles_.push_back(std::move(predicted_env));
    relative_time += FLAGS_trajectory_time_resolution;
  }
}

}  // namespace planning
}  // namespace apollo
