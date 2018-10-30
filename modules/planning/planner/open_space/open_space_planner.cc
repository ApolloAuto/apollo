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

  // initialize open space trajectory generator;
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
  // 1. Build Predicition Environments.
  predicted_bounding_rectangles_.clear();
  {
    std::lock_guard<std::mutex> lock(open_space_mutex_);
    BuildPredictedEnvironment(frame->obstacles());

    // 2. Update Vehicle information and obstacles information from frame.
    // TODO(QiL, Jinyun): Refactor this to be more compact
    vehicle_state_ = frame->vehicle_state();
    rotate_angle_ = frame->origin_heading();
    translate_origin_ = frame->origin_point();
    end_pose_ = frame->open_space_end_pose();
    obstacles_num_ = frame->obstacles_num();
    obstacles_edges_num_ = frame->obstacles_edges_num();
    obstacles_A_ = frame->obstacles_A();
    obstacles_b_ = frame->obstacles_b();
    obstalce_list_ = frame->openspace_warmstart_obstacles();
    XYbounds_ = frame->ROI_xy_boundary();
    // 3. Check if trajectory updated, if so, update internal
    // current_trajectory_;
    if (trajectory_updated_) {
      open_space_trajectory_generator_->UpdateTrajectory(&current_trajectory_);
      AINFO << "Trajectory caculation updated, new results : "
            << current_trajectory_.ShortDebugString();
    }

    // 4. Collision check for updated trajectory, if pass, update frame, else,
    // return error status
    if (IsCollisionFreeTrajectory(current_trajectory_)) {
      frame->mutable_trajectory()->CopyFrom(current_trajectory_);
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
    const ADCTrajectory& adc_trajectory) {
  CHECK_LE(adc_trajectory.trajectory_point().size(),
           predicted_bounding_rectangles_.size());
  const auto& vehicle_config =
      common::VehicleConfigHelper::Instance()->GetConfig();
  double ego_length = vehicle_config.vehicle_param().length();
  double ego_width = vehicle_config.vehicle_param().width();

  for (int i = 0; i < adc_trajectory.trajectory_point().size(); ++i) {
    const auto& trajectory_point = adc_trajectory.trajectory_point(i);
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
