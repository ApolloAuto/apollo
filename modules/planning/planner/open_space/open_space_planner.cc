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
    const std::vector<common::TrajectoryPoint>& stitching_trajectory,
    Frame* frame) {
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
      thread_data_.stitching_trajectory = stitching_trajectory;
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

    /*
        // Check destination
        apollo::common::Status destination_status =
            CheckDestination(thread_data_.stitching_trajectory.back(),
                             thread_data_.vehicle_state, thread_data_.end_pose);
        if (destination_status ==
            Status(ErrorCode::OK, "init_point reach end_pose")) {
          double x = thread_data_.stitching_trajectory.back().path_point().x();
          double y = thread_data_.stitching_trajectory.back().path_point().y();
          double theta =
              thread_data_.stitching_trajectory.back().path_point().theta();
          GenerateStopTrajectory(x, y, theta, &trajectory_to_end_);
          LoadTrajectoryToFrame(frame);
          ADEBUG << "Init point reach destination, stop trajectory is "
                    "sent";
          return Status::OK();
        } else if (destination_status ==
                   Status(ErrorCode::OK, "vehicle reach end_pose")) {
          double x = thread_data_.vehicle_state.x();
          double y = thread_data_.vehicle_state.y();
          double theta = thread_data_.vehicle_state.heading();
          GenerateStopTrajectory(x, y, theta, &trajectory_to_end_);
          LoadTrajectoryToFrame(frame);
          ADEBUG << "vehicle reach destination, stop trajectory is "
                    "sent";
          return Status::OK();
        }
    */
    // Check if trajectory updated
    if (trajectory_updated_) {
      std::lock_guard<std::mutex> lock(open_space_mutex_);
      open_space_trajectory_generator_->UpdateTrajectory(&trajectory_to_end_);
      open_space_trajectory_generator_->UpdateDebugInfo(&open_space_debug_);
      open_space_trajectory_generator_->GetStitchingTrajectory(
          &stitching_trajectory_);
      LoadTrajectoryToFrame(frame);
      trajectory_updated_.store(false);
      return Status::OK();
    }

    return Status(ErrorCode::OK,
                  "Waiting for planning thread in OpenSpacePlanner");

  } else {
    // Single thread logic
    open_space_roi_generator_.reset(new OpenSpaceROI());
    if (!open_space_roi_generator_->GenerateRegionOfInterest(frame)) {
      return Status(ErrorCode::PLANNING_ERROR,
                    "Generate Open Space ROI failed");
    }

    stitching_trajectory_ = stitching_trajectory;
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

    /*
        // Check destination
        apollo::common::Status destination_status = CheckDestination(
            stitching_trajectory_.back(), vehicle_state_, end_pose_);
        if (destination_status ==
            Status(ErrorCode::OK, "init_point reach end_pose")) {
          double x = stitching_trajectory_.back().path_point().x();
          double y = stitching_trajectory_.back().path_point().y();
          double theta = stitching_trajectory_.back().path_point().theta();
          GenerateStopTrajectory(x, y, theta, &trajectory_to_end_);
          LoadTrajectoryToFrame(frame);
          ADEBUG << "Init point reaches destination, stop trajectory is "
                    "sent";
          return Status::OK();
        } else if (destination_status ==
                   Status(ErrorCode::OK, "vehicle reach end_pose")) {
          double x = vehicle_state_.x();
          double y = vehicle_state_.y();
          double theta = vehicle_state_.heading();
          GenerateStopTrajectory(x, y, theta, &trajectory_to_end_);
          LoadTrajectoryToFrame(frame);
          ADEBUG << "vehicle reaches destination, stop trajectory is "
                    "sent";
          return destination_status;
        }

    */

    // Generate Trajectory;
    Status status = open_space_trajectory_generator_->Plan(
        stitching_trajectory_, vehicle_state_, XYbounds_, rotate_angle_,
        translate_origin_, end_pose_, obstacles_num_, obstacles_edges_num_,
        obstacles_A_, obstacles_b_, warmstart_obstacles_);

    // If status is OK, update vehicle trajectory;
    if (status == Status::OK()) {
      open_space_trajectory_generator_->UpdateTrajectory(&trajectory_to_end_);
      open_space_trajectory_generator_->UpdateDebugInfo(&open_space_debug_);
      open_space_trajectory_generator_->GetStitchingTrajectory(
          &stitching_trajectory_);
      LoadTrajectoryToFrame(frame);
      return status;
    } else {
      return status;
    }
  }
}

void OpenSpacePlanner::GenerateTrajectoryThread() {
  while (!is_stop_) {
    if (!is_destination_) {
      ADEBUG << "Open space plan in multi-threads mode : start to generate new "
                "trajectories";
      OpenSpaceThreadData thread_data;
      {
        std::lock_guard<std::mutex> lock(open_space_mutex_);
        thread_data = thread_data_;
      }
      if (!trajectory_updated_) {
        if (open_space_trajectory_generator_->Plan(
                thread_data.stitching_trajectory, thread_data.vehicle_state,
                thread_data.XYbounds, thread_data.rotate_angle,
                thread_data.translate_origin, thread_data.end_pose,
                thread_data.obstacles_num, thread_data.obstacles_edges_num,
                thread_data.obstacles_A, thread_data.obstacles_b,
                thread_data.warmstart_obstacles) == Status::OK()) {
          trajectory_updated_.store(true);
        }
      }
    } else {
      ADEBUG << "Vehicle reach end thread not doing planning";
    }
  }
}

void OpenSpacePlanner::Stop() {
  is_stop_ = true;
  if (FLAGS_enable_open_space_planner_thread) {
    task_future_.get();
  }
}

void OpenSpacePlanner::GenerateStopTrajectory(
    const double& stop_x, const double& stop_y, const double& stop_theta,
    common::Trajectory* trajectory_to_end) {
  constexpr int stop_trajectory_length = 10;
  constexpr double relative_stop_time = 0.1;
  apollo::common::Trajectory trajectory_to_stop;
  double relative_time = 0;
  for (int i = 0; i < stop_trajectory_length; i++) {
    apollo::common::TrajectoryPoint* point =
        trajectory_to_stop.add_trajectory_point();
    point->mutable_path_point()->set_x(stop_x);
    point->mutable_path_point()->set_y(stop_y);
    point->mutable_path_point()->set_theta(stop_theta);
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
  trajectory_to_end_pb_.Clear();
  trajectory_to_end_pb_.mutable_trajectory_point()->CopyFrom(
      *(trajectory_to_end_.mutable_trajectory_point()));
  frame->mutable_trajectory()->CopyFrom(trajectory_to_end_pb_);
  frame->mutable_open_space_debug()->CopyFrom(open_space_debug_);
  *(frame->mutable_last_stitching_trajectory()) = stitching_trajectory_;
}

}  // namespace planning
}  // namespace apollo
