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

#include "modules/planning/tasks/optimizers/open_space_trajectory_generation/open_space_trajectory_provider.h"

#include "cyber/task/task.h"
#include "modules/common/vehicle_state/proto/vehicle_state.pb.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/trajectory/publishable_trajectory.h"
#include "modules/planning/common/trajectory_stitcher.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::common::math::Vec2d;
using apollo::common::time::Clock;

OpenSpaceTrajectoryProvider::OpenSpaceTrajectoryProvider(
    const TaskConfig& config)
    : TrajectoryOptimizer(config) {
  open_space_trajectory_optimizer_.reset(new OpenSpaceTrajectoryOptimizer(
      config.open_space_trajectory_provider_config()
          .open_space_trajectory_optimizer_config()));
}

OpenSpaceTrajectoryProvider::~OpenSpaceTrajectoryProvider() {
  if (FLAGS_enable_open_space_planner_thread) {
    Stop();
  }
}

void OpenSpaceTrajectoryProvider::Stop() {
  if (FLAGS_enable_open_space_planner_thread) {
    is_generation_thread_stop_.store(true);
    if (thread_init_flag_) {
      task_future_.get();
    }
    trajectory_updated_.store(false);
    trajectory_error_.store(false);
    trajectory_skipped_.store(false);
    optimizer_thread_counter = 0;
  }
}

void OpenSpaceTrajectoryProvider::Restart() {
  if (FLAGS_enable_open_space_planner_thread) {
    is_generation_thread_stop_.store(true);
    if (thread_init_flag_) {
      task_future_.get();
    }
    is_generation_thread_stop_.store(false);
    thread_init_flag_ = false;
    trajectory_updated_.store(false);
    trajectory_error_.store(false);
    trajectory_skipped_.store(false);
    optimizer_thread_counter = 0;
  }
}

Status OpenSpaceTrajectoryProvider::Process() {
  ADEBUG << "trajectory provider";
  auto trajectory_data =
      frame_->mutable_open_space_info()->mutable_stitched_trajectory_result();

  // generate stop trajectory at park_and_go check_stage
  if (PlanningContext::Instance()
          ->mutable_planning_status()
          ->mutable_park_and_go()
          ->in_check_stage()) {
    ADEBUG << "ParkAndGo Stage Check.";
    GenerateStopTrajectory(trajectory_data);
    return Status::OK();
  }
  // Start thread when getting in Process() for the first time
  if (FLAGS_enable_open_space_planner_thread && !thread_init_flag_) {
    task_future_ = cyber::Async(
        &OpenSpaceTrajectoryProvider::GenerateTrajectoryThread, this);
    thread_init_flag_ = true;
  }
  // Get stitching trajectory from last frame
  const common::VehicleState vehicle_state = frame_->vehicle_state();
  auto* previous_frame = FrameHistory::Instance()->Latest();
  // Use complete raw trajectory from last frame for stitching purpose
  std::vector<TrajectoryPoint> stitching_trajectory;
  if (!IsVehicleStopDueToFallBack(
          previous_frame->open_space_info().fallback_flag(), vehicle_state)) {
    const auto& previous_planning =
        previous_frame->open_space_info().stitched_trajectory_result();
    const auto& previous_planning_header =
        previous_frame->current_frame_planned_trajectory()
            .header()
            .timestamp_sec();
    const double planning_cycle_time = FLAGS_open_space_planning_period;
    PublishableTrajectory last_frame_complete_trajectory(
        previous_planning_header, previous_planning);
    std::string replan_reason;
    const double start_timestamp = Clock::NowInSeconds();
    stitching_trajectory = TrajectoryStitcher::ComputeStitchingTrajectory(
        vehicle_state, start_timestamp, planning_cycle_time,
        FLAGS_open_space_trajectory_stitching_preserved_length, false,
        &last_frame_complete_trajectory, &replan_reason);
  } else {
    ADEBUG << "Replan due to fallback stop";
    const double planning_cycle_time =
        1.0 / static_cast<double>(FLAGS_planning_loop_rate);
    stitching_trajectory = TrajectoryStitcher::ComputeReinitStitchingTrajectory(
        planning_cycle_time, vehicle_state);
    auto* open_space_status = PlanningContext::Instance()
                                  ->mutable_planning_status()
                                  ->mutable_open_space();
    open_space_status->set_position_init(false);
  }
  // Get open_space_info from current frame
  const auto& open_space_info = frame_->open_space_info();

  if (FLAGS_enable_open_space_planner_thread) {
    ADEBUG << "Open space plan in multi-threads mode";

    if (is_generation_thread_stop_) {
      GenerateStopTrajectory(trajectory_data);
      return Status(ErrorCode::OK, "Parking finished");
    }

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
      data_ready_.store(true);
    }

    // Check vehicle state
    if (IsVehicleNearDestination(
            vehicle_state, open_space_info.open_space_end_pose(),
            open_space_info.origin_heading(), open_space_info.origin_point())) {
      GenerateStopTrajectory(trajectory_data);
      is_generation_thread_stop_.store(true);
      return Status(ErrorCode::OK, "Vehicle is near to destination");
    }

    // Check if trajectory updated
    if (trajectory_updated_) {
      std::lock_guard<std::mutex> lock(open_space_mutex_);
      LoadResult(trajectory_data);
      if (FLAGS_enable_record_debug) {
        // call merge debug ptr, open_space_trajectory_optimizer_
        auto* ptr_debug = frame_->mutable_open_space_info()->mutable_debug();
        open_space_trajectory_optimizer_->UpdateDebugInfo(
            ptr_debug->mutable_planning_data()->mutable_open_space());

        // sync debug instance
        frame_->mutable_open_space_info()->sync_debug_instance();
      }
      data_ready_.store(false);
      trajectory_updated_.store(false);
      return Status::OK();
    }

    if (trajectory_error_) {
      ++optimizer_thread_counter;
      std::lock_guard<std::mutex> lock(open_space_mutex_);
      trajectory_error_.store(false);
      // TODO(Jinyun) Use other fallback mechanism when last iteration smoothing
      // result has out of bound pathpoint which is not allowed for next
      // iteration hybrid astar algorithm which requires start position to be
      // strictly in bound
      if (optimizer_thread_counter > 1000) {
        return Status(ErrorCode::PLANNING_ERROR,
                      "open_space_optimizer failed too many times");
      }
    }

    if (previous_frame->open_space_info().open_space_provider_success()) {
      ReuseLastFrameResult(previous_frame, trajectory_data);
      if (FLAGS_enable_record_debug) {
        // copy previous debug to current frame
        ReuseLastFrameDebug(previous_frame);
      }
      // reuse last frame debug when use last frame traj
      return Status(ErrorCode::OK,
                    "Waiting for open_space_trajectory_optimizer in "
                    "open_space_trajectory_provider");
    } else {
      GenerateStopTrajectory(trajectory_data);
      return Status(ErrorCode::OK, "Stop due to computation not finished");
    }
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

    // Check vehicle state
    if (IsVehicleNearDestination(vehicle_state, end_pose, rotate_angle,
                                 translate_origin)) {
      GenerateStopTrajectory(trajectory_data);
      return Status(ErrorCode::OK, "Vehicle is near to destination");
    }

    // Generate Trajectory;
    Status status = open_space_trajectory_optimizer_->Plan(
        stitching_trajectory, end_pose, XYbounds, rotate_angle,
        translate_origin, obstacles_edges_num, obstacles_A, obstacles_b,
        obstacles_vertices_vec);

    // If status is OK, update vehicle trajectory;
    if (status == Status::OK()) {
      LoadResult(trajectory_data);
      return status;
    } else {
      return status;
    }
  }
  return Status(ErrorCode::PLANNING_ERROR);
}

void OpenSpaceTrajectoryProvider::GenerateTrajectoryThread() {
  while (!is_generation_thread_stop_) {
    if (!trajectory_updated_ && data_ready_) {
      OpenSpaceTrajectoryThreadData thread_data;
      {
        std::lock_guard<std::mutex> lock(open_space_mutex_);
        thread_data = thread_data_;
      }
      Status status = open_space_trajectory_optimizer_->Plan(
          thread_data.stitching_trajectory, thread_data.end_pose,
          thread_data.XYbounds, thread_data.rotate_angle,
          thread_data.translate_origin, thread_data.obstacles_edges_num,
          thread_data.obstacles_A, thread_data.obstacles_b,
          thread_data.obstacles_vertices_vec);
      if (status == Status::OK()) {
        std::lock_guard<std::mutex> lock(open_space_mutex_);
        trajectory_updated_.store(true);
      } else {
        if (status.ok()) {
          std::lock_guard<std::mutex> lock(open_space_mutex_);
          trajectory_skipped_.store(true);
        } else {
          std::lock_guard<std::mutex> lock(open_space_mutex_);
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

  double end_theta_to_world_frame = end_pose[2];
  end_theta_to_world_frame += rotate_angle;

  double distance_to_vehicle =
      std::sqrt((vehicle_state.x() - end_pose_to_world_frame.x()) *
                    (vehicle_state.x() - end_pose_to_world_frame.x()) +
                (vehicle_state.y() - end_pose_to_world_frame.y()) *
                    (vehicle_state.y() - end_pose_to_world_frame.y()));

  double theta_to_vehicle = std::abs(common::math::AngleDiff(
      vehicle_state.heading(), end_theta_to_world_frame));
  ADEBUG << "theta_to_vehicle" << theta_to_vehicle << "end_theta_to_world_frame"
         << end_theta_to_world_frame << "rotate_angle" << rotate_angle;
  ADEBUG << "is_near_destination_threshold"
         << config_.open_space_trajectory_provider_config()
                .open_space_trajectory_optimizer_config()
                .planner_open_space_config()
                .is_near_destination_threshold();  // which config file
  ADEBUG << "is_near_destination_theta_threshold"
         << config_.open_space_trajectory_provider_config()
                .open_space_trajectory_optimizer_config()
                .planner_open_space_config()
                .is_near_destination_theta_threshold();
  if (distance_to_vehicle < config_.open_space_trajectory_provider_config()
                                .open_space_trajectory_optimizer_config()
                                .planner_open_space_config()
                                .is_near_destination_threshold() &&
      theta_to_vehicle < config_.open_space_trajectory_provider_config()
                             .open_space_trajectory_optimizer_config()
                             .planner_open_space_config()
                             .is_near_destination_theta_threshold()) {
    ADEBUG << "vehicle reach end_pose";
    frame_->mutable_open_space_info()->set_destination_reached(true);
    return true;
  }
  return false;
}

bool OpenSpaceTrajectoryProvider::IsVehicleStopDueToFallBack(
    const bool is_on_fallback, const common::VehicleState& vehicle_state) {
  if (!is_on_fallback) {
    return false;
  }
  static constexpr double kEpsilon = 1.0e-1;
  const double adc_speed = vehicle_state.linear_velocity();
  const double adc_acceleration = vehicle_state.linear_acceleration();
  if (std::abs(adc_speed) < kEpsilon && std::abs(adc_acceleration) < kEpsilon) {
    ADEBUG << "ADC stops due to fallback trajectory";
    return true;
  }
  return false;
}

void OpenSpaceTrajectoryProvider::GenerateStopTrajectory(
    DiscretizedTrajectory* const trajectory_data) {
  double relative_time = 0.0;
  // TODO(Jinyun) Move to conf
  static constexpr int stop_trajectory_length = 10;
  static constexpr double relative_stop_time = 0.1;
  static constexpr double vEpsilon = 0.00001;
  double standstill_acceleration =
      frame_->vehicle_state().linear_velocity() >= -vEpsilon
          ? -FLAGS_open_space_standstill_acceleration
          : FLAGS_open_space_standstill_acceleration;
  trajectory_data->clear();
  for (size_t i = 0; i < stop_trajectory_length; i++) {
    TrajectoryPoint point;
    point.mutable_path_point()->set_x(frame_->vehicle_state().x());
    point.mutable_path_point()->set_y(frame_->vehicle_state().y());
    point.mutable_path_point()->set_theta(frame_->vehicle_state().heading());
    point.mutable_path_point()->set_s(0.0);
    point.mutable_path_point()->set_kappa(0.0);
    point.set_relative_time(relative_time);
    point.set_v(0.0);
    point.set_a(standstill_acceleration);
    trajectory_data->emplace_back(point);
    relative_time += relative_stop_time;
  }
}

void OpenSpaceTrajectoryProvider::LoadResult(
    DiscretizedTrajectory* const trajectory_data) {
  // Load unstitched two trajectories into frame for debug
  trajectory_data->clear();
  auto optimizer_trajectory_ptr =
      frame_->mutable_open_space_info()->mutable_optimizer_trajectory_data();
  auto stitching_trajectory_ptr =
      frame_->mutable_open_space_info()->mutable_stitching_trajectory_data();
  open_space_trajectory_optimizer_->GetOptimizedTrajectory(
      optimizer_trajectory_ptr);
  open_space_trajectory_optimizer_->GetStitchingTrajectory(
      stitching_trajectory_ptr);
  // Stitch two trajectories and load back to trajectory_data from frame
  size_t optimizer_trajectory_size = optimizer_trajectory_ptr->size();
  double stitching_point_relative_time =
      stitching_trajectory_ptr->back().relative_time();
  double stitching_point_relative_s =
      stitching_trajectory_ptr->back().path_point().s();
  for (size_t i = 0; i < optimizer_trajectory_size; ++i) {
    optimizer_trajectory_ptr->at(i).set_relative_time(
        optimizer_trajectory_ptr->at(i).relative_time() +
        stitching_point_relative_time);
    optimizer_trajectory_ptr->at(i).mutable_path_point()->set_s(
        optimizer_trajectory_ptr->at(i).path_point().s() +
        stitching_point_relative_s);
  }
  *(trajectory_data) = *(optimizer_trajectory_ptr);

  // Last point in stitching trajectory is already in optimized trajectory, so
  // it is deleted
  frame_->mutable_open_space_info()
      ->mutable_stitching_trajectory_data()
      ->pop_back();
  trajectory_data->PrependTrajectoryPoints(
      frame_->open_space_info().stitching_trajectory_data());
  frame_->mutable_open_space_info()->set_open_space_provider_success(true);
}

void OpenSpaceTrajectoryProvider::ReuseLastFrameResult(
    const Frame* last_frame, DiscretizedTrajectory* const trajectory_data) {
  *(trajectory_data) =
      last_frame->open_space_info().stitched_trajectory_result();
  frame_->mutable_open_space_info()->set_open_space_provider_success(true);
}

void OpenSpaceTrajectoryProvider::ReuseLastFrameDebug(const Frame* last_frame) {
  // reuse last frame's instance
  auto* ptr_debug = frame_->mutable_open_space_info()->mutable_debug_instance();
  ptr_debug->mutable_planning_data()->mutable_open_space()->MergeFrom(
      last_frame->open_space_info()
          .debug_instance()
          .planning_data()
          .open_space());
}

}  // namespace planning
}  // namespace apollo
