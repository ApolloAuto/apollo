/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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
 * @file path_reference_decider.cc
 */
#include "modules/planning/tasks/deciders/path_reference_decider/path_reference_decider.h"

#include <string>
#include <utility>
#include <vector>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/box2d.h"
#include "modules/common/math/line_segment2d.h"
#include "modules/planning/common/path_boundary.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/tasks/task.h"

namespace apollo {
namespace planning {

using common::SLPoint;
using common::Status;
using common::TrajectoryPoint;
using common::math::Box2d;
using common::math::LineSegment2d;
using common::math::Vec2d;

PathReferenceDecider::PathReferenceDecider(const TaskConfig &config)
    : Task(config) {}

Status PathReferenceDecider::Execute(Frame *frame,
                                     ReferenceLineInfo *reference_line_info) {
  Task::Execute(frame, reference_line_info);
  return Status::OK();
}

Status PathReferenceDecider::Process(
    Frame *frame, const ReferenceLineInfo *reference_line_info) {
  // get path bounds info from reference line info
  const std::vector<PathBoundary> &path_boundaries =
      reference_line_info_->GetCandidatePathBoundaries();
  ADEBUG << "There are " << path_boundaries.size() << " path boundaries.";

  // get learning model output (trajectory) from frame
  const std::vector<common::TrajectoryPoint> &path_reference =
      frame->learning_data_adc_future_trajectory_points();
  ADEBUG << "There are " << path_reference.size() << " path points.";

  // check if trajectory points are within path bounds
  // if yes: use learning model output
  // otherwise: use previous path planning method
  if (IsValidPathReference(path_reference, path_boundaries)) {
    // mark learning trajectory as path reference
    frame->set_learning_trajectory_valid(true);
  }

  return Status::OK();
}

bool PathReferenceDecider::IsValidPathReference(
    const std::vector<TrajectoryPoint> &path_reference,
    const std::vector<PathBoundary> &path_bounds) {
  // choose only regular path_bound
  const PathBoundary *regular_path_bound;
  for (const auto path_bound : path_bounds) {
    if (path_bound.label() == "regular") {
      regular_path_bound = &path_bound;
      break;
    }
  }
  ADEBUG << regular_path_bound->label();
  // loop over output trajectory points
  // check if path reference point is valid or not
  // 1. line segment formed by two adjacent boundary point
  std::vector<std::vector<LineSegment2d>> segmented_path_bounds;
  PathBoundToLineSegments(regular_path_bound, segmented_path_bounds);
  // has intersection with
  std::vector<Box2d> vehicle_boxes;
  for (const auto path_reference_point : path_reference) {
    // add ADC box to current path_reference_point
    const auto &path_point = path_reference_point.path_point();
    Box2d vehicle_box =
        common::VehicleConfigHelper::Instance()->GetBoundingBox(path_point);
    vehicle_boxes.emplace_back(vehicle_box);
  }

  // check intersection of linesegments and adc boxes
  // left & right path bounds
  for (auto segmented_path_bound : segmented_path_bounds) {
    // line segment for each bound
    for (auto line_segment : segmented_path_bound) {
      // check if all vehicle boxes along learning model outputhas overlap with
      // ADC box
      // TODO(Shu): early stop when vehicle box is far away.
      for (auto vehicle_box : vehicle_boxes) {
        if (vehicle_box.HasOverlap(line_segment)) {
          return false;
        }
      }
    }
  }

  return true;
}

void PathReferenceDecider::PathBoundToLineSegments(
    const PathBoundary *path_bound,
    std::vector<std::vector<LineSegment2d>> &path_bound_segments) {
  const double start_s = path_bound->start_s();
  const double delta_s = path_bound->delta_s();
  const size_t path_bound_size = path_bound->boundary().size();
  std::vector<LineSegment2d> cur_left_bound_segments;
  std::vector<LineSegment2d> cur_right_bound_segments;
  const std::vector<std::pair<double, double>> &path_bound_points =
      path_bound->boundary();
  // setup the init path bound point
  SLPoint prev_left_sl_point;
  SLPoint prev_right_sl_point;
  prev_left_sl_point.set_l(std::get<0>(path_bound_points[0]));
  prev_left_sl_point.set_s(start_s);
  prev_right_sl_point.set_l(std::get<1>(path_bound_points[0]));
  prev_right_sl_point.set_s(start_s);
  // slpoint to cartesion point
  Vec2d prev_left_cartesian_point;
  Vec2d prev_right_cartesian_point;
  reference_line_info_->reference_line().SLToXY(prev_left_sl_point,
                                                &prev_left_cartesian_point);
  reference_line_info_->reference_line().SLToXY(prev_right_sl_point,
                                                &prev_right_cartesian_point);
  for (size_t i = 1; i < path_bound_size; ++i) {
    // s = start_s + i * delta_s
    double current_s = start_s + i * delta_s;
    SLPoint left_sl_point;
    Vec2d left_cartesian_point;
    SLPoint right_sl_point;
    Vec2d right_cartesian_point;

    left_sl_point.set_l(std::get<0>(path_bound_points[i]));
    left_sl_point.set_s(current_s);
    right_sl_point.set_l(std::get<1>(path_bound_points[i]));
    right_sl_point.set_s(current_s);

    reference_line_info_->reference_line().SLToXY(left_sl_point,
                                                  &left_cartesian_point);
    reference_line_info_->reference_line().SLToXY(right_sl_point,
                                                  &right_cartesian_point);

    // together with previous point form line segment
    LineSegment2d cur_left_segment(prev_left_cartesian_point,
                                   left_cartesian_point);
    LineSegment2d cur_right_segment(prev_right_cartesian_point,
                                    right_cartesian_point);
    prev_left_cartesian_point = left_cartesian_point;
    prev_right_cartesian_point = right_cartesian_point;
    cur_left_bound_segments.emplace_back(cur_left_segment);
    cur_right_bound_segments.emplace_back(cur_right_segment);
  }
  path_bound_segments.emplace_back(cur_left_bound_segments);
  path_bound_segments.emplace_back(cur_right_bound_segments);
}

}  // namespace planning
}  // namespace apollo
