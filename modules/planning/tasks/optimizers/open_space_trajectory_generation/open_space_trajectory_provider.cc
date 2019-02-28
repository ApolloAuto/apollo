/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include <string>
#include <vector>

#include "modules/planning/tasks/optimizers/open_space_trajectory_generation/open_space_trajectory_provider.h"

#include "cyber/task/task.h"
#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/trajectory/publishable_trajectory.h"
#include "modules/planning/common/trajectory_stitcher.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::common::time::Clock;

OpenSpaceTrajectoryProvider::OpenSpaceTrajectoryProvider(
    const TaskConfig& config)
    : TrajectoryOptimizer(config) {
  open_space_trajectory_optimizer_.reset(new OpenSpaceTrajectoryOptimizer(
      config.open_space_trajectory_provider_config()
          .open_space_trajectory_optimizer_config()));
}

Status OpenSpaceTrajectoryProvider::Process(
    DiscretizedTrajectory* const trajectory_data) {
  // Start thread when getting in Process() for the first time
  if (FLAGS_enable_open_space_planner_thread && !thread_init_flag_) {
    task_future_ = cyber::Async(
        &OpenSpaceTrajectoryProvider::GenerateTrajectoryThread, this);
    thread_init_flag_ = true;
  }
  // Get stitching trajectory from last frame
  const double start_timestamp = Clock::NowInSeconds();
  const common::VehicleState vehicle_state = frame_->vehicle_state();
  const double planning_cycle_time = FLAGS_open_space_planning_period;
  std::string replan_reason;
  auto previous_frame = FrameHistory::Instance()->Latest();
  const auto& previous_planning =
      previous_frame->current_frame_planned_trajectory();
  PublishableTrajectory last_frame_publishable_trajectory(previous_planning);
  auto stitching_trajectory = TrajectoryStitcher::ComputeStitchingTrajectory(
      vehicle_state, start_timestamp, planning_cycle_time,
      &last_frame_publishable_trajectory, &replan_reason);

  // Get open_space_info from current frame
  const auto& open_space_info = frame_->open_space_info();

  if (FLAGS_enable_open_space_planner_thread) {
    ADEBUG << "Open space plan in multi-threads mode";

    {
      std::lock_guard<std::mutex> lock(open_space_mutex_);
      thread_data_.stitching_trajectory = stitching_trajectory;
      thread_data_.end_pose = open_space_info.open_space_end_pose();
      thread_data_.rotate_angle = open_space_info.origin_heading();
      thread_data_.translate_origin = open_space_info.origin_point();
      thread_data_.obstacles_edges_num = open_space_info.obstacles_edges_num();
      thread_data_.obstacles_A = open_space_info.obstacles_A();
      thread_data_.obstacles_b = open_space_info.obstacles_b();
      thread_data_.obstacles_vertices_vec =
          open_space_info.obstacles_vertices_vec();
      thread_data_.XYbounds = open_space_info.ROI_xy_boundary();
    }

    // check vehicle state
    if (IsVehicleNearDestination(
            vehicle_state, open_space_info.open_space_end_pose(),
            open_space_info.origin_heading(), open_space_info.origin_point())) {
      return Status(ErrorCode::OK, "Vehicle is near to destination");
    }

    // Check if trajectory updated
    if (trajectory_updated_) {
      std::lock_guard<std::mutex> lock(open_space_mutex_);
      open_space_trajectory_optimizer_->GetOptimizedTrajectory(trajectory_data);
      open_space_trajectory_optimizer_->GetStitchingTrajectory(
          frame_->mutable_open_space_info()
              ->mutable_stitching_trajectory_data());
      trajectory_updated_.store(false);
      return Status::OK();
    }

    if (trajectory_error_) {
      ++optimizer_thread_counter;
      trajectory_error_.store(false);
      if (optimizer_thread_counter > 5) {
        return Status(ErrorCode::PLANNING_ERROR,
                      "open_space_optimizer failed too many times");
      }
    }

    return Status(ErrorCode::OK,
                  "Waiting for open_space_trajectory_optimizer in "
                  "open_space_trajectory_provider");

  } else {
    const auto& end_pose = open_space_info.open_space_end_pose();
    const auto& rotate_angle = open_space_info.origin_heading();
    const auto& translate_origin = open_space_info.origin_point();
    const auto& obstacles_edges_num = open_space_info.obstacles_edges_num();
    const auto& obstacles_A = open_space_info.obstacles_A();
    const auto& obstacles_b = open_space_info.obstacles_b();
    const auto& obstacles_vertices_vec =
        open_space_info.obstacles_vertices_vec();
    const auto& XYbounds = open_space_info.ROI_xy_boundary();

    // check vehicle state
    if (IsVehicleNearDestination(vehicle_state, end_pose, rotate_angle,
                                 translate_origin)) {
      return Status(ErrorCode::OK, "Vehicle is near to destination");
    }

    // Generate Trajectory;
    Status status = open_space_trajectory_optimizer_->Plan(
        stitching_trajectory, end_pose, XYbounds, rotate_angle,
        translate_origin, obstacles_edges_num, obstacles_A, obstacles_b,
        obstacles_vertices_vec);

    // If status is OK, update vehicle trajectory;
    if (status == Status::OK()) {
      open_space_trajectory_optimizer_->GetOptimizedTrajectory(trajectory_data);
      open_space_trajectory_optimizer_->GetStitchingTrajectory(
          frame_->mutable_open_space_info()
              ->mutable_stitching_trajectory_data());
      return status;
    } else {
      return status;
    }
  }
  return Status::OK();
}

void OpenSpaceTrajectoryProvider::GenerateTrajectoryThread() {
  while (!is_stop_) {
    OpenSpaceTrajectoryThreadData thread_data;
    {
      std::lock_guard<std::mutex> lock(open_space_mutex_);
      thread_data = thread_data_;
    }
    if (!trajectory_updated_) {
      Status status = open_space_trajectory_optimizer_->Plan(
          thread_data.stitching_trajectory, thread_data.end_pose,
          thread_data.XYbounds, thread_data.rotate_angle,
          thread_data.translate_origin, thread_data.obstacles_edges_num,
          thread_data.obstacles_A, thread_data.obstacles_b,
          thread_data_.obstacles_vertices_vec);
      if (status == Status::OK()) {
        trajectory_updated_.store(true);
      } else {
        ADEBUG_EVERY(200) << "open_space_trajectory_optimizer not returning "
                             "OK() with status: "
                          << status.ToString();
        if (status.ok()) {
          trajectory_skipped_.store(true);
        } else {
          trajectory_error_.store(true);
        }
      }
    }
  }
}

bool OpenSpaceTrajectoryProvider::IsVehicleNearDestination(
    const common::VehicleState& vehicle_state,
    const std::vector<double>& end_pose, double rotate_angle,
    const Vec2d& translate_origin) {
  CHECK_EQ(end_pose.size(), 4);
  Vec2d end_pose_to_world_frame = Vec2d(end_pose[0], end_pose[1]);

  end_pose_to_world_frame.SelfRotate(rotate_angle);
  end_pose_to_world_frame += translate_origin;

  double distance_to_vehicle =
      std::sqrt((vehicle_state.x() - end_pose_to_world_frame.x()) *
                    (vehicle_state.x() - end_pose_to_world_frame.x()) +
                (vehicle_state.y() - end_pose_to_world_frame.y()) *
                    (vehicle_state.y() - end_pose_to_world_frame.y()));

  if (distance_to_vehicle < config_.open_space_trajectory_provider_config()
                                .open_space_trajectory_optimizer_config()
                                .is_near_destination_threshold()) {
    ADEBUG << "vehicle reach end_pose";
    return true;
  }
  return false;
}

}  // namespace planning
}  // namespace apollo
