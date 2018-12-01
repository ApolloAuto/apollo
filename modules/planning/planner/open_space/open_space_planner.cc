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
  planning_init_point_ = planning_init_point;

  if (FLAGS_enable_open_space_planner_thread) {
    ADEBUG << "Open space plan in multi-threads mode";

    // 2. Update Vehicle information and obstacles information from frame.
    vehicle_state_ = frame->vehicle_state();
    open_space_roi_generator_.reset(new OpenSpaceROI());
    if (!open_space_roi_generator_->GenerateRegionOfInterest(frame)) {
      return Status(ErrorCode::PLANNING_ERROR,
                    "Generate Open Space ROI failed");
    }

    {
      std::lock_guard<std::mutex> lock(open_space_mutex_);
      thread_data_.rotate_angle = open_space_roi_generator_->origin_heading();
      thread_data_.translate_origin = open_space_roi_generator_->origin_point();
      thread_data_.end_pose = open_space_roi_generator_->open_space_end_pose();
      thread_data_.obstacles_num = open_space_roi_generator_->obstacles_num();
      thread_data_.obstacles_edges_num =
          open_space_roi_generator_->obstacles_edges_num();
      thread_data_.obstacles_A = open_space_roi_generator_->obstacles_A();
      thread_data_.obstacles_b = open_space_roi_generator_->obstacles_b();
      thread_data_.XYbounds = open_space_roi_generator_->ROI_xy_boundary();
    }
    // 3. Check if trajectory updated, if so, update internal
    // trajectory_partition_;
    if (trajectory_updated_) {
      AINFO << "Trajectories have been updated!";
      std::lock_guard<std::mutex> lock(open_space_mutex_);
      trajectory_to_end_.Clear();
      open_space_debug_.Clear();
      open_space_trajectory_generator_->UpdateTrajectory(&trajectory_to_end_);
      open_space_trajectory_generator_->UpdateDebugInfo(&open_space_debug_);
    }

    publishable_trajectory_.Clear();
    publishable_trajectory_.mutable_trajectory_point()->CopyFrom(
        *(trajectory_to_end_.mutable_trajectory_point()));
    frame->mutable_trajectory()->CopyFrom(publishable_trajectory_);
    frame->mutable_open_space_debug()->CopyFrom(open_space_debug_);

    return Status::OK();

  } else {
    // Single thread logic
    vehicle_state_ = frame->vehicle_state();
    open_space_roi_generator_.reset(new OpenSpaceROI());
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
    XYbounds_ = open_space_roi_generator_->ROI_xy_boundary();

    // 2. Generate Trajectory;
    Status status = open_space_trajectory_generator_->Plan(
        planning_init_point_, vehicle_state_, XYbounds_, rotate_angle_,
        translate_origin_, end_pose_, obstacles_num_, obstacles_edges_num_,
        obstacles_A_, obstacles_b_, obstalce_list_);
    // 3. If status is OK, update vehicle trajectory;
    if (status == Status::OK()) {
      trajectory_to_end_.Clear();
      open_space_debug_.Clear();
      open_space_trajectory_generator_->UpdateTrajectory(&trajectory_to_end_);
      open_space_trajectory_generator_->UpdateDebugInfo(&open_space_debug_);
      publishable_trajectory_.Clear();
      publishable_trajectory_.mutable_trajectory_point()->CopyFrom(
          *(trajectory_to_end_.mutable_trajectory_point()));
      frame->mutable_trajectory()->CopyFrom(publishable_trajectory_);
      frame->mutable_open_space_debug()->CopyFrom(open_space_debug_);
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
      {
        std::lock_guard<std::mutex> lock(open_space_mutex_);
        trajectory_updated_ = false;
      }
      if (open_space_trajectory_generator_->Plan(
              thread_data_.vehicle_state, thread_data_.XYbounds,
              thread_data_.rotate_angle, thread_data_.translate_origin,
              thread_data_.end_pose, thread_data_.obstacles_num,
              thread_data_.obstacles_edges_num, thread_data_.obstacles_A,
              thread_data_.obstacles_b,
              open_space_roi_generator_->openspace_warmstart_obstacles()) ==
          Status::OK()) {
        {
          std::lock_guard<std::mutex> lock(open_space_mutex_);
          trajectory_updated_ = true;
        }
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

}  // namespace planning
}  // namespace apollo
