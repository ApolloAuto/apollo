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
    open_space_roi_generator_.reset(
        new OpenSpaceROI(planner_open_space_config_));
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
      thread_data_.obstacles_edges_num =
          open_space_roi_generator_->obstacles_edges_num();
      thread_data_.obstacles_A = open_space_roi_generator_->obstacles_A();
      thread_data_.obstacles_b = open_space_roi_generator_->obstacles_b();
      thread_data_.obstacles_vertices_vec =
          open_space_roi_generator_->obstacles_vertices_vec();
      thread_data_.XYbounds = open_space_roi_generator_->ROI_xy_boundary();
    }

    // Check if trajectory updated
    if (trajectory_updated_) {
      std::lock_guard<std::mutex> lock(open_space_mutex_);
      if (open_space_trajectory_generator_->end_pose() ==
          thread_data_.end_pose) {
        open_space_trajectory_generator_->UpdateTrajectory(&trajectory_to_end_);
        open_space_trajectory_generator_->UpdateDebugInfo(&open_space_debug_);
        open_space_trajectory_generator_->GetStitchingTrajectory(
            &stitching_trajectory_);
        LoadTrajectoryToFrame(frame);
        trajectory_updated_.store(false);
        return Status(ErrorCode::OK, "Planning trajectory updated");
      } else {
        AINFO << "End pose not same between threads, old pose : "
              << open_space_trajectory_generator_->end_pose()[0] << ", "
              << open_space_trajectory_generator_->end_pose()[1] << ", "
              << open_space_trajectory_generator_->end_pose()[2] << ", "
              << open_space_trajectory_generator_->end_pose()[3]
              << ", new pose : " << thread_data_.end_pose[0] << ", "
              << thread_data_.end_pose[1] << ", " << thread_data_.end_pose[2]
              << ", " << thread_data_.end_pose[3]
              << ", disgard this trajectory generation.";
        trajectory_updated_.store(false);
        return Status(ErrorCode::OK,
                      "Disgard this frame due to end_pose not match");
      }
    }

    return Status(ErrorCode::OK,
                  "Waiting for planning thread in OpenSpacePlanner");

  } else {
    // Single thread logic
    open_space_roi_generator_.reset(
        new OpenSpaceROI(planner_open_space_config_));
    if (!open_space_roi_generator_->GenerateRegionOfInterest(frame)) {
      return Status(ErrorCode::PLANNING_ERROR,
                    "Generate Open Space ROI failed");
    }

    stitching_trajectory_ = stitching_trajectory;
    vehicle_state_ = frame->vehicle_state();
    rotate_angle_ = open_space_roi_generator_->origin_heading();
    translate_origin_ = open_space_roi_generator_->origin_point();
    end_pose_ = open_space_roi_generator_->open_space_end_pose();
    obstacles_edges_num_ = open_space_roi_generator_->obstacles_edges_num();
    obstacles_A_ = open_space_roi_generator_->obstacles_A();
    obstacles_b_ = open_space_roi_generator_->obstacles_b();
    obstacles_vertices_vec_ =
        open_space_roi_generator_->obstacles_vertices_vec();
    XYbounds_ = open_space_roi_generator_->ROI_xy_boundary();

    // Generate Trajectory;
    Status status = open_space_trajectory_generator_->Plan(
        stitching_trajectory_, vehicle_state_, XYbounds_, rotate_angle_,
        translate_origin_, end_pose_, obstacles_edges_num_, obstacles_A_,
        obstacles_b_, obstacles_vertices_vec_);

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
    OpenSpaceThreadData thread_data;
    {
      std::lock_guard<std::mutex> lock(open_space_mutex_);
      thread_data = thread_data_;
    }
    if (!trajectory_updated_) {
      Status status = open_space_trajectory_generator_->Plan(
          thread_data.stitching_trajectory, thread_data.vehicle_state,
          thread_data.XYbounds, thread_data.rotate_angle,
          thread_data.translate_origin, thread_data.end_pose,
          thread_data.obstacles_edges_num, thread_data.obstacles_A,
          thread_data.obstacles_b, thread_data_.obstacles_vertices_vec);
      if (status == Status::OK()) {
        trajectory_updated_.store(true);
      } else {
        AERROR
            << "Multi-thread trajectory generator failed with return satus : "
            << status.ToString();
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
