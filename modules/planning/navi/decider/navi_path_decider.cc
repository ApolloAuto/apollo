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
 * @brief This file provides the implementation of the class "NaviPathDecider".
 */

#include "modules/planning/navi/decider/navi_path_decider.h"

#include <vector>

#include "glog/logging.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::Status;

NaviPathDecider::NaviPathDecider() : Task("NaviPathDecider") {
  // TODO(all): Add your other initialization.
}

bool NaviPathDecider::Init(const PlanningConfig& config) {
  config_ = config.navi_planner_config().navi_path_decider_config();
  is_init_ = true;
  return true;
}

Status NaviPathDecider::Execute(Frame* frame,
                                ReferenceLineInfo* const reference_line_info) {
  Task::Execute(frame, reference_line_info);
  auto ret =
      Process(reference_line_info->reference_line(),
              frame->PlanningStartPoint(), reference_line_info->path_decision(),
              reference_line_info->mutable_path_data());
  RecordDebugInfo(reference_line_info->path_data());
  if (ret != Status::OK()) {
    reference_line_info->SetDrivable(false);
    AERROR << "Reference Line " << reference_line_info->Lanes().Id()
           << " is not drivable after " << Name();
  }
  return ret;
}

apollo::common::Status NaviPathDecider::Process(
    const ReferenceLine& reference_line,
    const common::TrajectoryPoint& init_point,
    PathDecision* const path_decision, PathData* const path_data) {
  CHECK_NOTNULL(path_decision);
  CHECK_NOTNULL(path_data);

  // get path plane lenth
  size_t path_len = static_cast<size_t>(
      std::ceil(init_point.v() * config_.min_look_forward_time()));
  size_t min_path_len = config_.min_path_length();
  path_len = path_len > min_path_len ? path_len : min_path_len;

  // get LocalPath
  // LocalPath local_path = GetLocalPath(reference_line);
  LocalPath local_path = GetLocalPath(reference_line);
  int range = local_path.GetRange();
  if (local_path.GetRange() < static_cast<int>(path_len)) {
    AERROR << "Planning path strength is too short.";
    return Status(apollo::common::ErrorCode::PLANNING_ERROR,
                  "NaviPathDecider GetLocalPath");
  }

  // According to the position of the car and the reference line, the path
  // trajectory taken by the reference line is shifted on the y-axis.
  double init_local_path_y = 0.0;
  if (!local_path.GetInitY(&init_local_path_y) || init_local_path_y > 3) {
    AERROR << "Get init y from local path failed.";
    return Status(apollo::common::ErrorCode::PLANNING_ERROR,
                  "NaviPathDecider GetInitY");
  }

  if (reference_line_info_->IsChangeLanePath()) {
    // get shift value and shift
  } else {
    double start_y = SmoothInitY(init_local_path_y);
    local_path.Shift(start_y - init_local_path_y);
  }

  if (FLAGS_enable_nudge_decision) {
    // do nudge process
  }

  // calculate the value of the path trajectory later
  reference_line_info_->AddCost(0.0);

  DiscretizedPath discretized_path(local_path.GetPathPoints());
  path_data->SetReferenceLine(&reference_line);
  if (!path_data->SetDiscretizedPath(discretized_path)) {
    AERROR << "Set path data failed.";
    return Status(apollo::common::ErrorCode::PLANNING_ERROR,
                  "NaviPathDecider SetDiscretizedPath");
  }

  return Status::OK();
}

void NaviPathDecider::RecordDebugInfo(const PathData& path_data) {
  const auto& path_points = path_data.discretized_path().path_points();
  auto* ptr_optimized_path = reference_line_info_->mutable_debug()
                                 ->mutable_planning_data()
                                 ->add_path();
  ptr_optimized_path->set_name(Name());
  ptr_optimized_path->mutable_path_point()->CopyFrom(
      {path_points.begin(), path_points.end()});
}

LocalPath NaviPathDecider::GetLocalPath(const ReferenceLine& reference_line) {
  if (reference_line.reference_points().size() < 10) {
    AERROR
        << "Reference line points is not enough to generate path trajectory.";
    return LocalPath();
  }

  // find the projection reference point of the car on the reference line
  auto project_point = reference_line.GetReferencePoint(0, 0);
  double reference_line_len = reference_line.Length();
  auto& lane_way_points = project_point.lane_waypoints();
  if (lane_way_points.empty()) {
    AERROR << "Failed to get lane way points from reference line.";
    return LocalPath();
  }

  // get points form reference_line
  double project_point_s = lane_way_points[0].s;
  size_t reference_point_num =
      static_cast<size_t>(std::ceil(reference_line_len - project_point_s)) + 1;
  std::vector<apollo::common::PathPoint> path_points;
  for (size_t i = 0; i <= reference_point_num; ++i) {
    double s = i + project_point_s;
    auto point = reference_line.GetReferencePoint(s);
    path_points.emplace_back(point.ToPathPoint(s));
  }

  return LocalPath(path_points);
}

double NaviPathDecider::SmoothInitY(const double init_y) {
  const double max_init_y = config_.max_smooth_init_y();
  double smooth_init_y = init_y;
  if (init_y > max_init_y) {
    smooth_init_y = max_init_y;
  } else if (init_y < -max_init_y) {
    smooth_init_y = -max_init_y;
  }
  return smooth_init_y;
}

}  // namespace planning
}  // namespace apollo
