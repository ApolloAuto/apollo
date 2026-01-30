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

#include "modules/planning/tasks/open_space_trajectory_optimizer_park/open_space_trajectory_optimizer_park.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;

bool OpenSpaceTrajectoryOptimizerPark::Init(
    const std::string &config_dir, const std::string &name,
    const std::shared_ptr<DependencyInjector> &injector) {
  if (!Task::Init(config_dir, name, injector)) {
    return false;
  }
  if (!Task::LoadConfig<OpenSpaceTrajectoryOptimizerParkConfig>(&config_)) {
    return false;
  }
  trajectory_optimizer_.reset(new Optimizer(config_));
  return true;
}

OpenSpaceTrajectoryOptimizerPark::~OpenSpaceTrajectoryOptimizerPark() {
  if (config_.enable_trajectory_optimize_thread()) {
    Stop();
  }
}

void OpenSpaceTrajectoryOptimizerPark::Stop() {
  if (config_.enable_trajectory_optimize_thread()) {
    if (thread_init_flag_) {
      task_future_.get();
    }
  }
}

Status OpenSpaceTrajectoryOptimizerPark::Process() {
  auto* previous_frame = injector_->frame_history()->Latest();
  if (!frame_->open_space_info().replan_flag()) {
    AINFO << "Not replan";
    *(frame_->mutable_open_space_info()->mutable_optimizer_trajectory_data()) =
        previous_frame->open_space_info().optimizer_trajectory_data();
    frame_->mutable_open_space_info()->set_open_space_provider_success(
      previous_frame->open_space_info().open_space_provider_success());
    if (FLAGS_enable_record_debug) {
      // copy previous debug to current frame
      ReuseLastFrameDebug(previous_frame);
    }
    return Status::OK();
  }

  if (replan_flag_) {
    replan_flag_ = false;
    frame_->mutable_open_space_info()->
        mutable_path_planning_trajectory_result()->clear();
  }

  if (frame_->open_space_info().path_planning_trajectory_result().empty()) {
    AINFO << "No initial trajectory solution";
    return Status::OK();
  }

  Optimize();
  if (trajectory_update_) {
    // call merge debug ptr, open_space_trajectory_optimizer_
    auto* ptr_debug = frame_->mutable_open_space_info()->mutable_debug();
    trajectory_optimizer_->UpdateDebugInfo(
        ptr_debug->mutable_planning_data()->mutable_open_space());
    // sync debug instance
    frame_->mutable_open_space_info()->sync_debug_instance();

    LoadResult(frame_->mutable_open_space_info()->
        mutable_optimizer_trajectory_data());
    thread_init_flag_ = false;
    trajectory_update_ = false;
  }

  return Status::OK();
}

void OpenSpaceTrajectoryOptimizerPark::Optimize() {
  if (config_.enable_trajectory_optimize_thread()) {
    if (thread_init_flag_) {
      AINFO << "trajectory is optimizing, please wait!";
      return;
    }

    task_future_ = cyber::Async(
        &OpenSpaceTrajectoryOptimizerPark::GenerateTrajectoryThread, this);
    thread_init_flag_ = true;
  } else {
    GenerateTrajectoryThread();
    LoadResult(frame_->mutable_open_space_info()->
        mutable_optimizer_trajectory_data());
  }
}

void OpenSpaceTrajectoryOptimizerPark::GenerateTrajectoryThread() {
  Status ret = trajectory_optimizer_->Plan(frame_->open_space_info());

  if (ret == Status::OK()) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    trajectory_update_ = true;
    AINFO << "tarjectory optimize successfully!";
  } else {
    thread_init_flag_ = false;
    replan_flag_ = true;
    AERROR << "tarjectory optimize failed";
  }
}

void OpenSpaceTrajectoryOptimizerPark::LoadResult(
    DiscretizedTrajectory* const trajectory_data) {
  // Load unstitched two trajectories into frame for debug
  trajectory_data->clear();

  trajectory_optimizer_->GetOptimizedTrajectory(
      *trajectory_data);

  frame_->mutable_open_space_info()->set_open_space_provider_success(true);
}

void OpenSpaceTrajectoryOptimizerPark::ReuseLastFrameDebug(
    const Frame* last_frame) {
  // reuse last frame's instance
  auto* ptr_debug = frame_->mutable_open_space_info()->mutable_debug_instance();
  ptr_debug->mutable_planning_data()->mutable_open_space()->MergeFrom(
      last_frame->open_space_info()
          .debug_instance()
          .planning_data()
          .open_space());
  ptr_debug->mutable_planning_data()->mutable_open_space()->clear_obstacles();
  // load obstacles
  for (const auto& obstacle_vertices :
      frame_->open_space_info().obstacles_vertices_vec()) {
    auto* obstacle_ptr = ptr_debug->mutable_planning_data()->
        mutable_open_space()->add_obstacles();
    for (const auto& vertex : obstacle_vertices) {
      obstacle_ptr->add_vertices_x_coords(vertex.x());
      obstacle_ptr->add_vertices_y_coords(vertex.y());
    }
  }
}

}  // namespace planning
}  // namespace apollo
