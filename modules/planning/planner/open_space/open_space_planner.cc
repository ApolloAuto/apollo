/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

Status OpenSpacePlanner::Init(const PlanningConfig& planning_confgs) {
  AINFO << "In OpenSpacePlanner::Init()";

  // TODO(QiL): integrate open_space planner into task config when refactor done
  CHECK(common::util::GetProtoFromFile(FLAGS_planner_open_space_config_filename,
                                       &planner_open_space_config_))
      << "Failed to load open space config file "
      << FLAGS_planner_open_space_config_filename;

  current_trajectory_index_ = 0;
  // initialize open space trajectory generator
  open_space_trajectory_generator_.reset(new OpenSpaceTrajectoryGenerator());

  open_space_trajectory_generator_->Init(planner_open_space_config_);

  // initialize open space region of interest generator
  if (FLAGS_enable_open_space_roi_and_info) {
    open_space_roi_generator_.reset(new OpenSpaceROI());
  }

  if (FLAGS_enable_open_space_planner_thread) {
    task_future_ =
        cyber::Async(&OpenSpacePlanner::GenerateTrajectoryThread, this);
  }

  return Status::OK();
}

apollo::common::Status OpenSpacePlanner::Plan(
    const common::TrajectoryPoint& planning_init_point, Frame* frame) {
  // 1. Build Predicition Environments.
  predicted_bounding_rectangles_.clear();
  if (FLAGS_enable_open_space_planner_thread) {
    std::lock_guard<std::mutex> lock(open_space_mutex_);
    BuildPredictedEnvironment(frame->obstacles());

    // 2. Update Vehicle information and obstacles information from frame.

    vehicle_state_ = frame->vehicle_state();
    if (!open_space_roi_generator_->GenerateRegionOfInterest(frame)) {
      return Status(ErrorCode::PLANNING_ERROR,
                    "Generate Open Space ROI failed");
    }
    rotate_angle_ = open_space_roi_generator_->origin_heading();
    translate_origin_ = open_space_roi_generator_->origin_point();
    end_pose_ = open_space_roi_generator_->open_space_end_pose();
    obstacles_num_ = open_space_roi_generator_->obstacles_num();
    obstacles_edges_num_ = open_space_roi_generator_->obstacles_edges_num();
    obstacles_A_ = open_space_roi_generator_->obstacles_A();
    obstacles_b_ = open_space_roi_generator_->obstacles_b();
    obstalce_list_ = open_space_roi_generator_->openspace_warmstart_obstacles();
    XYbounds_ = open_space_roi_generator_->ROI_xy_boundary();
    // 3. Check if trajectory updated, if so, update internal
    // trajectory_partition_;
    if (trajectory_updated_) {
      trajectory_partition_.Clear();
      gear_positions_.clear();
      open_space_trajectory_generator_->UpdateTrajectory(&trajectory_partition_,
                                                         &gear_positions_);
      AINFO << "Trajectory caculation updated, new results : "
            << trajectory_partition_.ShortDebugString();
    }
    // TODO(Jiaxuan): Choose the current_trajectory in trajectory_partition_
    // If the vehicle each the end point of current trajectory and stop
    // Then move to the next Trajectory
    current_trajectory_ =
        trajectory_partition_.trajectory(current_trajectory_index_);

    TrajectoryPoint end_point = current_trajectory_.trajectory_point(
        current_trajectory_.trajectory_point_size() - 1);

    if (vehicle_state_.linear_velocity() <= 1e-3 &&
        std::sqrt((vehicle_state_.x() - end_point.path_point().x()) *
                      (vehicle_state_.x() - end_point.path_point().x()) +
                  (vehicle_state_.y() - end_point.path_point().y()) *
                      (vehicle_state_.y() - end_point.path_point().y())) <
            planner_open_space_config_.max_position_error_to_end_point() &&
        std::abs(vehicle_state_.heading() - end_point.path_point().theta()) <
            planner_open_space_config_.max_theta_error_to_end_point() &&
        (current_trajectory_index_ <
         trajectory_partition_.trajectory_size() - 1)) {
      current_trajectory_index_ += 1;
    }

    // 4. Collision check for updated trajectory, if pass, update frame, else,
    // return error status

    if (IsCollisionFreeTrajectory(current_trajectory_)) {
      // Convet current trajectory to publishable_trajectory
      publishable_trajectory_.Clear();
      publishable_trajectory_.MergeFrom(current_trajectory_);
      frame->mutable_trajectory()->CopyFrom(publishable_trajectory_);
      return Status::OK();
    } else {
      // If collision happens, return wrong planning status and estop
      // trajectory would be sent in std planning
      return Status(ErrorCode::PLANNING_ERROR, "Collision Check failed");
    }
  } else {
    // Single thread logic

    vehicle_state_ = frame->vehicle_state();
    if (!open_space_roi_generator_->GenerateRegionOfInterest(frame)) {
      return Status(ErrorCode::PLANNING_ERROR,
                    "Generate Open Space ROI failed");
    }
    rotate_angle_ = open_space_roi_generator_->origin_heading();
    translate_origin_ = open_space_roi_generator_->origin_point();
    end_pose_ = open_space_roi_generator_->open_space_end_pose();
    obstacles_num_ = open_space_roi_generator_->obstacles_num();
    obstacles_edges_num_ = open_space_roi_generator_->obstacles_edges_num();
    obstacles_A_ = open_space_roi_generator_->obstacles_A();
    obstacles_b_ = open_space_roi_generator_->obstacles_b();
    obstalce_list_ = open_space_roi_generator_->openspace_warmstart_obstacles();
    XYbounds_ = open_space_roi_generator_->ROI_xy_boundary();

    // 2. Generate Trajectory;
    Status status = open_space_trajectory_generator_->Plan(
        vehicle_state_, XYbounds_, rotate_angle_, translate_origin_, end_pose_,
        obstacles_num_, obstacles_edges_num_, obstacles_A_, obstacles_b_,
        obstalce_list_);

    // 3. If status is OK, update vehicle trajectory;
    if (status == Status::OK()) {
      trajectory_partition_.Clear();
      gear_positions_.clear();
      open_space_trajectory_generator_->UpdateTrajectory(&trajectory_partition_,
                                                         &gear_positions_);
      AINFO << "Trajectory caculation updated, new results : "
            << trajectory_partition_.ShortDebugString();
    } else {
      return Status(ErrorCode::PLANNING_ERROR,
                    "Planning failed to generate open space trajectory");
    }

    current_trajectory_ =
        trajectory_partition_.trajectory(current_trajectory_index_);

    TrajectoryPoint end_point = current_trajectory_.trajectory_point(
        current_trajectory_.trajectory_point_size() - 1);

    if (vehicle_state_.linear_velocity() <= 1e-3 &&
        std::sqrt((vehicle_state_.x() - end_point.path_point().x()) *
                      (vehicle_state_.x() - end_point.path_point().x()) +
                  (vehicle_state_.y() - end_point.path_point().y()) *
                      (vehicle_state_.y() - end_point.path_point().y())) <
            planner_open_space_config_.max_position_error_to_end_point() &&
        std::abs(vehicle_state_.heading() - end_point.path_point().theta()) <
            planner_open_space_config_.max_theta_error_to_end_point() &&
        (current_trajectory_index_ <
         trajectory_partition_.trajectory_size() - 1)) {
      current_trajectory_index_ += 1;
    }

    // 4. Collision check for updated trajectory, if pass, update frame, else,
    // return error status

    if (IsCollisionFreeTrajectory(current_trajectory_)) {
      publishable_trajectory_.Clear();
      publishable_trajectory_.MergeFrom(current_trajectory_);
      frame->mutable_trajectory()->CopyFrom(publishable_trajectory_);
      return Status::OK();
    } else {
      // If collision happens, return wrong planning status and estop
      // trajectory would be sent in std planning
      return Status(ErrorCode::PLANNING_ERROR, "Collision Check failed");
    }
  }
}

void OpenSpacePlanner::GenerateTrajectoryThread() {
  while (!is_stop_) {
    {
      std::lock_guard<std::mutex> lock(open_space_mutex_);
      trajectory_updated_ = false;
      if (open_space_trajectory_generator_->Plan(
              vehicle_state_, XYbounds_, rotate_angle_, translate_origin_,
              end_pose_, obstacles_num_, obstacles_edges_num_, obstacles_A_,
              obstacles_b_, obstalce_list_) == Status::OK()) {
        trajectory_updated_ = true;
      }
    }
  }
}

void OpenSpacePlanner::Stop() {
  is_stop_ = true;
  if (FLAGS_enable_open_space_planner_thread) {
    task_future_.get();
  }
}

bool OpenSpacePlanner::IsCollisionFreeTrajectory(
    const apollo::common::Trajectory& trajectory) {
  CHECK_LE(trajectory.trajectory_point().size(),
           predicted_bounding_rectangles_.size());
  const auto& vehicle_config =
      common::VehicleConfigHelper::Instance()->GetConfig();
  double ego_length = vehicle_config.vehicle_param().length();
  double ego_width = vehicle_config.vehicle_param().width();

  for (int i = 0; i < trajectory.trajectory_point().size(); ++i) {
    const auto& trajectory_point = trajectory.trajectory_point(i);
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
        return false;
      }
    }
  }
  return true;
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
