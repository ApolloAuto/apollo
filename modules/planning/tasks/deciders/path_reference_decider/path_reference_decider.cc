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
#include "modules/planning/common/path/discretized_path.h"
#include "modules/planning/common/path_boundary.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/tasks/task.h"

namespace apollo {
namespace planning {

using common::ErrorCode;
using common::PathPoint;
using common::SLPoint;
using common::Status;
using common::TrajectoryPoint;
using common::math::Box2d;
using common::math::LineSegment2d;
using common::math::Vec2d;

int PathReferenceDecider::valid_path_reference_counter_ = 0;
int PathReferenceDecider::total_path_counter_ = 0;

PathReferenceDecider::PathReferenceDecider(const TaskConfig &config)
    : Task(config) {}

Status PathReferenceDecider::Execute(Frame *frame,
                                     ReferenceLineInfo *reference_line_info) {
  Task::Execute(frame, reference_line_info);
  return Process(frame, reference_line_info);
}

Status PathReferenceDecider::Process(Frame *frame,
                                     ReferenceLineInfo *reference_line_info) {
  // skip using path reference during lane changing
  // There are two reference line during change lane
  if (FLAGS_skip_path_reference_in_change_lane &&
      frame->reference_line_info().size() > 1) {
    reference_line_info->mutable_path_data()->set_is_valid_path_reference(
        false);
    ADEBUG << "Skip path reference when changing lane.";
    ADEBUG << "valid_path_reference_counter[" << valid_path_reference_counter_
           << "] total_path_counter[" << total_path_counter_ << "]";
    return Status::OK();
  }

  // skip using path reference during side pass
  if (FLAGS_skip_path_reference_in_side_pass &&
      reference_line_info->is_path_lane_borrow()) {
    reference_line_info->mutable_path_data()->set_is_valid_path_reference(
        false);
    ADEBUG << "Skip path reference when sidepass.";
    ADEBUG << "valid_path_reference_counter[" << valid_path_reference_counter_
           << "] total_path_counter[" << total_path_counter_ << "]";
    return Status::OK();
  }

  // get path bounds info from reference line info
  const std::vector<PathBoundary> &path_boundaries =
      reference_line_info_->GetCandidatePathBoundaries();
  ADEBUG << "There are " << path_boundaries.size() << " path boundaries.";

  // get learning model output (trajectory) from frame
  const std::vector<common::TrajectoryPoint> &path_reference =
      frame->learning_based_data().learning_data_adc_future_trajectory_points();
  ADEBUG << "There are " << path_reference.size() << " path points.";

  // get regular path bound
  size_t regular_path_bound_idx = GetRegularPathBound(path_boundaries);
  if (regular_path_bound_idx == path_boundaries.size()) {
    reference_line_info->mutable_path_data()->set_is_valid_path_reference(
        false);
    const std::string msg = "No regular path boundary";
    AERROR << msg;
    ADEBUG << "valid_path_reference_counter[" << valid_path_reference_counter_
           << "] total_path_counter[" << total_path_counter_ << "]";
    return Status(ErrorCode::PLANNING_ERROR, msg);
  }

  ++total_path_counter_;
  // check if path reference is valid
  // current, only check if path reference point is within path bounds
  if (!IsValidPathReference(*reference_line_info,
                            path_boundaries[regular_path_bound_idx],
                            path_reference)) {
    reference_line_info->mutable_path_data()->set_is_valid_path_reference(
        false);
    ADEBUG << "Learning model output violates path bounds. Not a validated "
              "path reference";
    ADEBUG << "valid_path_reference_counter[" << valid_path_reference_counter_
           << "] total_path_counter[" << total_path_counter_ << "]";
    return Status::OK();
  }

  // evaluate path reference
  std::vector<PathPoint> evaluated_path_reference;
  EvaluatePathReference(path_boundaries[regular_path_bound_idx], path_reference,
                        &evaluated_path_reference);

  // mark learning trajectory as path reference
  reference_line_info->mutable_path_data()->set_is_valid_path_reference(true);

  reference_line_info->mutable_path_data()->set_path_reference(
      evaluated_path_reference);

  ++valid_path_reference_counter_;
  ADEBUG << "valid_path_reference_counter[" << valid_path_reference_counter_
         << "] total_path_counter[" << total_path_counter_ << "]";

  return Status::OK();
}

size_t PathReferenceDecider::GetRegularPathBound(
    const std::vector<PathBoundary> &path_bounds) const {
  for (auto iter = begin(path_bounds); iter != end(path_bounds); ++iter) {
    if (iter->label().find("regular") != std::string::npos) {
      return distance(begin(path_bounds), iter);
    }
  }
  return path_bounds.size();
}

bool PathReferenceDecider::IsValidPathReference(
    const ReferenceLineInfo &reference_line_info,
    const PathBoundary &regular_path_bound,
    const std::vector<TrajectoryPoint> &path_reference) {
  for (auto path_referece_point : path_reference) {
    const double cur_x = path_referece_point.path_point().x();
    const double cur_y = path_referece_point.path_point().y();
    if (-1 == IsPointWithinPathBounds(reference_line_info, regular_path_bound,
                                      cur_x, cur_y)) {
      return false;
    }
  }
  return true;
}

bool PathReferenceDecider::IsADCBoxAlongPathReferenceWithinPathBounds(
    const std::vector<TrajectoryPoint> &path_reference,
    const PathBoundary &regular_path_bound) {
  /* check adc box has overlap with each path line segment*/
  // loop over output trajectory points
  // check if path reference point is valid or not
  // 1. line segment formed by two adjacent boundary point
  std::vector<std::vector<LineSegment2d>> segmented_path_bounds;
  PathBoundToLineSegments(regular_path_bound, &segmented_path_bounds);
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
      // check if all vehicle boxes along learning model outputhas
      // overlap with ADC box
      // TODO(Shu): early stop when vehicle box is far away.
      for (auto vehicle_box : vehicle_boxes) {
        if (vehicle_box.HasOverlap(line_segment)) {
          ADEBUG << std::setprecision(9) << "Vehicle box:["
                 << vehicle_box.center_x() << "," << vehicle_box.center_y()
                 << "]"
                 << "Violate path bound at [" << line_segment.start().x() << ","
                 << line_segment.start().y() << "];"
                 << "[" << line_segment.end().x() << ","
                 << line_segment.end().y() << "]";
          return false;
        }
      }
    }
  }

  return true;
}

void PathReferenceDecider::PathBoundToLineSegments(
    const PathBoundary &path_bound,
    std::vector<std::vector<LineSegment2d>> *path_bound_segments) {
  const double start_s = path_bound.start_s();
  const double delta_s = path_bound.delta_s();
  const size_t path_bound_size = path_bound.boundary().size();
  std::vector<LineSegment2d> cur_left_bound_segments;
  std::vector<LineSegment2d> cur_right_bound_segments;
  const std::vector<std::pair<double, double>> &path_bound_points =
      path_bound.boundary();
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
  path_bound_segments->emplace_back(cur_left_bound_segments);
  path_bound_segments->emplace_back(cur_right_bound_segments);
}

int PathReferenceDecider::IsPointWithinPathBounds(
    const ReferenceLineInfo &reference_line_info,
    const PathBoundary &path_bound, const double x, const double y) {
  const double kPathBoundsDeciderResolution = 0.5;
  common::SLPoint point_sl;
  reference_line_info.reference_line().XYToSL({x, y}, &point_sl);
  const double start_s = path_bound.start_s();
  const double delta_s = path_bound.delta_s();
  const int path_bound_size = path_bound.boundary().size();
  const double end_s = start_s + delta_s * (path_bound_size - 1);

  if (point_sl.s() > end_s ||
      point_sl.s() < start_s - kPathBoundsDeciderResolution * 2) {
    AERROR << "Longitudinally outside the boundary.";
    return -1;
  }
  int idx_after = 0;
  while (idx_after < path_bound_size &&
         start_s + idx_after * delta_s < point_sl.s()) {
    ++idx_after;
  }
  ADEBUG << "idx_after[" << idx_after << "] point_l[" << point_sl.l() << "]";
  if (idx_after >= 1) {
    int idx_before = idx_after - 1;
    if (std::get<0>(path_bound.boundary().at(idx_before)) <= point_sl.l() &&
        std::get<1>(path_bound.boundary().at(idx_before)) >= point_sl.l() &&
        std::get<0>(path_bound.boundary().at(idx_after)) <= point_sl.l() &&
        std::get<1>(path_bound.boundary().at(idx_after)) >= point_sl.l()) {
      return idx_after;
    }
  }
  ADEBUG << "Laterally outside the boundary.";
  return -1;
}

void PathReferenceDecider::EvaluatePathReference(
    const PathBoundary &path_bound,
    const std::vector<TrajectoryPoint> &path_reference,
    std::vector<PathPoint> *evaluated_path_reference) {
  const double delta_s = path_bound.delta_s();
  const double path_reference_end_s = path_reference.back().path_point().s();
  DiscretizedPath discrete_path_reference;
  for (auto trajectory_point : path_reference) {
    discrete_path_reference.emplace_back(trajectory_point.path_point());
  }
  size_t idx;
  for (idx = 0; idx < path_bound.boundary().size(); ++idx) {
    // relative s
    double cur_s = static_cast<double>(idx) * delta_s;
    if (cur_s > path_reference_end_s) {
      break;
    }
    evaluated_path_reference->emplace_back(
        discrete_path_reference.Evaluate(cur_s));
  }
}

}  // namespace planning
}  // namespace apollo
