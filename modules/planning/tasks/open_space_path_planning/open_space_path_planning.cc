/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/tasks/open_space_path_planning/open_space_path_planning.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;
using common::math::Vec2d;

bool OpenSpacePathPlanning::Init(
    const std::string &config_dir,
    const std::string &name,
    const std::shared_ptr<DependencyInjector> &injector) {
  if (!Task::Init(config_dir, name, injector)) {
    return false;
  }
  if (!Task::LoadConfig<OpenSpacePathPlanningConfig>(&config_)) {
    return false;
  }
  path_finder_.reset(new HybridAStar(config_.warm_start_config()));
  AINFO << config_.DebugString();
  return true;
}

OpenSpacePathPlanning::~OpenSpacePathPlanning() {
  if (config_.enable_path_planning_thread()) {
    Stop();
  }
}

void OpenSpacePathPlanning::Stop() {
  if (config_.enable_path_planning_thread()) {
    if (thread_init_flag_) {
      task_future_.get();
    }
    data_ready_.store(false);
    path_update_.store(false);
  }
}

Status OpenSpacePathPlanning::Process() {
  AINFO << "data_ready_: " << (data_ready_ ? "true" : "false");
  auto* previous_frame = injector_->frame_history()->Latest();
  if (!frame_->open_space_info().replan_flag()) {
    AINFO << "Not replan";
    *(frame_->mutable_open_space_info()->
        mutable_path_planning_trajectory_result()) =
        previous_frame->open_space_info().path_planning_trajectory_result();
    return Status::OK();
  }

  if (!previous_frame->open_space_info().
      path_planning_trajectory_result().empty()) {
    AINFO << "Previous frame has path planning result";
    *(frame_->mutable_open_space_info()->
        mutable_path_planning_trajectory_result()) =
        previous_frame->open_space_info().path_planning_trajectory_result();
    return Status::OK();
  }
  AINFO << "Path planning";

  if (path_update_) {
    AINFO << "Path planning updated";
    LoadResult(frame_->mutable_open_space_info()->
        mutable_path_planning_trajectory_result());
    path_update_.store(false);
    return Status::OK();
  }
  PathPlanning();
  return Status::OK();
}

void OpenSpacePathPlanning::PathPlanning() {
  if (config_.enable_path_planning_thread() && !thread_init_flag_) {
    task_future_ = cyber::Async(
        &OpenSpacePathPlanning::GeneratePathThread, this);
    thread_init_flag_.store(true);
  }

  const auto& open_space_info = frame_->open_space_info();
  double start_x = injector_->vehicle_state()->x();
  double start_y = injector_->vehicle_state()->y();
  double start_theta = injector_->vehicle_state()->heading();
  OpenSpaceTrajectoryOptimizerUtil::PathPointNormalizing(
      open_space_info.origin_heading(),
      open_space_info.origin_point(),
      &start_x, &start_y, &start_theta);
  if (config_.enable_path_planning_thread()) {
    if (!data_ready_) {
      std::lock_guard<std::mutex> lock(data_mutex_);
      thread_data_.start_pose = {start_x, start_y, start_theta};
      thread_data_.end_pose = open_space_info.open_space_end_pose();
      thread_data_.rotate_angle = open_space_info.origin_heading();
      thread_data_.translate_origin = open_space_info.origin_point();
      thread_data_.obstacles_edges_num = open_space_info.obstacles_edges_num();
      thread_data_.obstacles_A = open_space_info.obstacles_A();
      thread_data_.obstacles_b = open_space_info.obstacles_b();
      thread_data_.obstacles_vertices_vec =
          open_space_info.obstacles_vertices_vec();
      thread_data_.soft_boundary_vertices_vec =
          open_space_info.soft_boundary_vertices_vec();
      thread_data_.XYbounds = open_space_info.ROI_xy_boundary();
      thread_data_.reeds_sheep_last_straight =
          ((config_.enable_vertical_parking_last_trajectory_straight() &&
          open_space_info.parking_type() == ParkingType::VERTICAL_PARKING) || 
          (config_.enable_parallel_parking_last_trajectory_straight() &&
          open_space_info.parking_type() == ParkingType::PARALLEL_PARKING)) ?
              true : false;
      data_ready_.store(true);
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
    const auto& soft_boundary_vertices_vec =
        open_space_info.soft_boundary_vertices_vec();
    const auto& XYbounds = open_space_info.ROI_xy_boundary();
    const bool reeds_sheep_last_straight =
        ((config_.enable_vertical_parking_last_trajectory_straight() &&
          open_space_info.parking_type() == ParkingType::VERTICAL_PARKING) || 
          (config_.enable_parallel_parking_last_trajectory_straight() &&
          open_space_info.parking_type() == ParkingType::PARALLEL_PARKING)) ?
              true : false;
    if (path_finder_->Plan(start_x, start_y, start_theta,
                           end_pose[0], end_pose[1],
                           end_pose[2], XYbounds,
                           obstacles_vertices_vec,
                           &result_,
                           soft_boundary_vertices_vec,
                           reeds_sheep_last_straight)) {
      path_update_.store(true);
      AINFO << "State warm start problem solved successfully!";
    } else {
      AERROR << "State warm start problem failed to solve";
    }
  }
}

void OpenSpacePathPlanning::GeneratePathThread() {
  while (!data_ready_) {
    usleep(10);
  }
  OpenSpacePathPlanningThreadData thread_data;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    thread_data = thread_data_;
  }

  AINFO << "start pose: " << thread_data.start_pose[0]
        << " " << thread_data.start_pose[1]
        << " " << thread_data.start_pose[2];
  bool ret = path_finder_->Plan(
      thread_data.start_pose[0], thread_data.start_pose[1],
      thread_data.start_pose[2], thread_data.end_pose[0],
      thread_data.end_pose[1], thread_data.end_pose[2],
      thread_data.XYbounds, thread_data.obstacles_vertices_vec,
      &result_,thread_data_.soft_boundary_vertices_vec,
      thread_data.reeds_sheep_last_straight);

  if (ret) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    path_update_.store(true);
    AINFO << "path find success!";
  } else {
    AERROR << "path find failed";
  }
  thread_init_flag_.store(false);
  data_ready_.store(false);
  path_finder_.reset(new HybridAStar(config_.warm_start_config()));
}

void OpenSpacePathPlanning::LoadResult(
    DiscretizedTrajectory* const trajectory_data) {
  trajectory_data->clear();
  result_.x.back() = frame_->open_space_info().open_space_end_pose()[0];
  result_.y.back() = frame_->open_space_info().open_space_end_pose()[1];
  result_.phi.back() = frame_->open_space_info().open_space_end_pose()[2];
  for (size_t i = 0; i < result_.x.size(); i++) {
    OpenSpaceTrajectoryOptimizerUtil::PathPointDeNormalizing(
        frame_->open_space_info().origin_heading(),
        frame_->open_space_info().origin_point(),
        &result_.x[i],
        &result_.y[i],
        &result_.phi[i]);
    common::TrajectoryPoint point;
    point.mutable_path_point()->set_x(result_.x[i]);
    point.mutable_path_point()->set_y(result_.y[i]);
    point.mutable_path_point()->set_theta(result_.phi[i]);
    point.set_v(result_.v[i]);
    point.set_steer(result_.steer[i]);
    point.set_a(result_.a[i]);
    point.set_relative_time(i * 0.5);
    trajectory_data->emplace_back(point);
  }
}

}  // namespace planning
}  // namespace apollo
