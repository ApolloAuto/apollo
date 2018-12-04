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

  // initialize open space trajectory generator
  open_space_trajectory_generator_.reset(new OpenSpaceTrajectoryGenerator());

  open_space_trajectory_generator_->Init(planner_open_space_config_);

  if (FLAGS_enable_open_space_planner_thread) {
    task_future_ =
        cyber::Async(&OpenSpacePlanner::GenerateTrajectoryThread, this);
  }
  return Status::OK();
}

apollo::common::Status OpenSpacePlanner::Plan(
    const common::TrajectoryPoint& planning_init_point, Frame* frame) {
  if (FLAGS_enable_open_space_planner_thread) {
    ADEBUG << "Open space plan in multi-threads mode";

    // Update Vehicle information and obstacles information from frame.
    open_space_roi_generator_.reset(new OpenSpaceROI());
    if (!open_space_roi_generator_->GenerateRegionOfInterest(frame)) {
      return Status(ErrorCode::PLANNING_ERROR,
                    "Generate Open Space ROI failed");
    }

    {
      std::lock_guard<std::mutex> lock(open_space_mutex_);
      thread_data_.planning_init_point = planning_init_point;
      thread_data_.vehicle_state = frame->vehicle_state();
      thread_data_.rotate_angle = open_space_roi_generator_->origin_heading();
      thread_data_.translate_origin = open_space_roi_generator_->origin_point();
      thread_data_.end_pose = open_space_roi_generator_->open_space_end_pose();
      thread_data_.obstacles_num = open_space_roi_generator_->obstacles_num();
      thread_data_.obstacles_edges_num =
          open_space_roi_generator_->obstacles_edges_num();
      thread_data_.obstacles_A = open_space_roi_generator_->obstacles_A();
      thread_data_.obstacles_b = open_space_roi_generator_->obstacles_b();
      thread_data_.XYbounds = open_space_roi_generator_->ROI_xy_boundary();
      thread_data_.warmstart_obstacles =
          open_space_roi_generator_->openspace_warmstart_obstacles();
    }

    // Check destination
    if (CheckDestination(thread_data_.planning_init_point,
                         thread_data_.end_pose)) {
      GenerateStopTrajectory(thread_data_.planning_init_point,
                             &trajectory_to_end_);
      LoadTrajectoryToFrame(frame);
      ADEBUG << "Init point reach destination, stop trajectory is "
                "sent";
      return Status::OK();
    }

    // Check if trajectory updated
    if (trajectory_updated_) {
      std::lock_guard<std::mutex> lock(open_space_mutex_);
      open_space_trajectory_generator_->UpdateTrajectory(&trajectory_to_end_);
      open_space_trajectory_generator_->UpdateDebugInfo(&open_space_debug_);
      LoadTrajectoryToFrame(frame);
      trajectory_updated_.store(false);
      return Status::OK();
    }

    return Status(ErrorCode::PLANNING_ERROR,
                  "Waiting for planning thread in OpenSpacePlanner");

  } else {
    // Single thread logic
    open_space_roi_generator_.reset(new OpenSpaceROI());
    if (!open_space_roi_generator_->GenerateRegionOfInterest(frame)) {
      return Status(ErrorCode::PLANNING_ERROR,
                    "Generate Open Space ROI failed");
    }

    planning_init_point_ = planning_init_point;
    vehicle_state_ = frame->vehicle_state();
    rotate_angle_ = open_space_roi_generator_->origin_heading();
    translate_origin_ = open_space_roi_generator_->origin_point();
    end_pose_ = open_space_roi_generator_->open_space_end_pose();
    obstacles_num_ = open_space_roi_generator_->obstacles_num();
    obstacles_edges_num_ = open_space_roi_generator_->obstacles_edges_num();
    obstacles_A_ = open_space_roi_generator_->obstacles_A();
    obstacles_b_ = open_space_roi_generator_->obstacles_b();
    XYbounds_ = open_space_roi_generator_->ROI_xy_boundary();
    warmstart_obstacles_ =
        open_space_roi_generator_->openspace_warmstart_obstacles();

    // Check destination
    if (CheckDestination(planning_init_point_, end_pose_)) {
      GenerateStopTrajectory(planning_init_point, &trajectory_to_end_);
      LoadTrajectoryToFrame(frame);
      ADEBUG << "Init point reach destination, stop trajectory is "
                "sent";
      return Status::OK();
    }

    // Generate Trajectory;
    Status status = open_space_trajectory_generator_->Plan(
        planning_init_point_, vehicle_state_, XYbounds_, rotate_angle_,
        translate_origin_, end_pose_, obstacles_num_, obstacles_edges_num_,
        obstacles_A_, obstacles_b_, warmstart_obstacles_);

    // If status is OK, update vehicle trajectory;
    if (status == Status::OK()) {
      open_space_trajectory_generator_->UpdateTrajectory(&trajectory_to_end_);
      open_space_trajectory_generator_->UpdateDebugInfo(&open_space_debug_);
      LoadTrajectoryToFrame(frame);
      return status;
    } else {
      return status;
    }
  }
}

void OpenSpacePlanner::GenerateTrajectoryThread() {
  while (!is_stop_) {
    {
      ADEBUG << "Open space plan in multi-threads mode : start to generate new "
                "trajectories";
      std::lock_guard<std::mutex> lock(open_space_mutex_);
      if (open_space_trajectory_generator_->Plan(
              thread_data_.planning_init_point, thread_data_.vehicle_state,
              thread_data_.XYbounds, thread_data_.rotate_angle,
              thread_data_.translate_origin, thread_data_.end_pose,
              thread_data_.obstacles_num, thread_data_.obstacles_edges_num,
              thread_data_.obstacles_A, thread_data_.obstacles_b,
              thread_data_.warmstart_obstacles) == Status::OK()) {
        trajectory_updated_.store(true);
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

bool OpenSpacePlanner::CheckDestination(
    const common::TrajectoryPoint& planning_init_point,
    const std::vector<double>& end_pose) {
  CHECK_EQ(end_pose.size(), 4);
  constexpr double kepsilon = 1e-4;
  const apollo::common::PathPoint path_point = planning_init_point.path_point();
  double distance =
      (path_point.x() - end_pose[0]) * (path_point.x() - end_pose[0]) +
      (path_point.y() - end_pose[1]) * (path_point.y() - end_pose[1]);
  if (distance < kepsilon) {
    return true;
  }
  return false;
}

void OpenSpacePlanner::GenerateStopTrajectory(
    const common::TrajectoryPoint& planning_init_point,
    common::Trajectory* trajectory_to_end) {
  constexpr int stop_trajectory_length = 10;
  constexpr double relative_stop_time = 0.1;
  apollo::common::Trajectory trajectory_to_stop;
  double stop_x = planning_init_point.path_point().x();
  double stop_y = planning_init_point.path_point().y();
  double stop_theta = planning_init_point.path_point().theta();
  double relative_time = 0;
  for (int i = 0; i < stop_trajectory_length; i++) {
    apollo::common::TrajectoryPoint* point =
        trajectory_to_stop.add_trajectory_point();
    point->mutable_path_point()->set_x(stop_x);
    point->mutable_path_point()->set_x(stop_y);
    point->mutable_path_point()->set_x(stop_theta);
    point->mutable_path_point()->set_s(0.0);
    point->mutable_path_point()->set_kappa(0.0);
    point->set_relative_time(relative_time);
    point->set_v(0.0);
    point->set_a(0.0);
    relative_time += relative_stop_time;
  }
  trajectory_to_end->mutable_trajectory_point()->CopyFrom(
      *(trajectory_to_stop.mutable_trajectory_point()));
}

void OpenSpacePlanner::LoadTrajectoryToFrame(Frame* frame) {
  publishable_trajectory_.Clear();
  publishable_trajectory_.mutable_trajectory_point()->CopyFrom(
      *(trajectory_to_end_.mutable_trajectory_point()));
  frame->mutable_trajectory()->CopyFrom(publishable_trajectory_);
  frame->mutable_open_space_debug()->CopyFrom(open_space_debug_);
}

}  // namespace planning
}  // namespace apollo
