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
#include "modules/planning/tasks/deciders/path_reuse_decider/path_reuse_decider.h"

#include <algorithm>
#include <string>
#include <vector>

namespace apollo {
namespace planning {
// #define ADEBUG AINFO
using apollo::common::Status;

int PathReuseDecider::reusable_path_counter_ = 0;
int PathReuseDecider::total_path_counter_ = 0;

PathReuseDecider::PathReuseDecider(const TaskConfig& config)
    : Decider(config) {}

Status PathReuseDecider::Process(Frame* const frame,
                                 ReferenceLineInfo* const reference_line_info) {
  // Sanity checks.
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  // Check if path is reusable
  if (Decider::config_.path_reuse_decider_config().reuse_path() &&
      CheckPathReusable(frame, reference_line_info)) {
    ++reusable_path_counter_;  // count reusable path
  }
  ++total_path_counter_;  // count total path
  ADEBUG << "reusable_path_counter_" << reusable_path_counter_;
  ADEBUG << "total_path_counter_" << total_path_counter_;
  return Status::OK();
}

bool PathReuseDecider::CheckPathReusable(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  return IsSameStopObstacles(frame, reference_line_info);
  //   return IsSameObstacles(reference_line_info);
}

bool PathReuseDecider::IsSameStopObstacles(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  if (history_->GetLastFrame() == nullptr) return false;

  const std::vector<const HistoryObjectDecision*> history_objects_decisions =
      history_->GetLastFrame()->GetStopObjectDecisions();
  const auto& reference_line = reference_line_info->reference_line();
  std::vector<double> history_stop_positions;
  std::vector<double> current_stop_positions;

  GetCurrentStopObstacleS(reference_line_info, &current_stop_positions);
  GetHistoryStopSPosition(reference_line_info, history_objects_decisions,
                          &history_stop_positions);

  // get current vehicle s
  common::math::Vec2d adc_position = {
      common::VehicleStateProvider::Instance()->x(),
      common::VehicleStateProvider::Instance()->y()};
  common::SLPoint adc_position_sl;

  reference_line.XYToSL(adc_position, &adc_position_sl);

  ADEBUG << "ADC_s:" << adc_position_sl.s();

  double nearest_history_stop_s = FLAGS_default_front_clear_distance;
  double nearest_current_stop_s = FLAGS_default_front_clear_distance;

  for (auto history_stop_position : history_stop_positions) {
    ADEBUG << "current_stop_position " << history_stop_position
           << "adc_position_sl.s()" << adc_position_sl.s();
    if (history_stop_position < adc_position_sl.s()) {
      continue;
    } else {
      // find nearest history stop
      nearest_history_stop_s = history_stop_position;
      break;
    }
  }

  for (auto current_stop_position : current_stop_positions) {
    ADEBUG << "current_stop_position " << current_stop_position
           << "adc_position_sl.s()" << adc_position_sl.s();
    if (current_stop_position < adc_position_sl.s()) {
      continue;
    } else {
      // find nearest current stop
      nearest_current_stop_s = current_stop_position;
      break;
    }
  }
  return SameStopS(nearest_history_stop_s, nearest_current_stop_s);
}

// compare history stop position vs current stop position
bool PathReuseDecider::SameStopS(const double history_stop_s,
                                 const double current_stop_s) {
  const double KNegative = 0.1;  // (meter) closer
  const double kPositive = 0.5;  // (meter) further
  ADEBUG << "current_stop_s" << current_stop_s;
  ADEBUG << "history_stop_s" << history_stop_s;
  if ((current_stop_s > history_stop_s &&
       current_stop_s - history_stop_s < kPositive) ||
      (current_stop_s < history_stop_s &&
       history_stop_s - current_stop_s < KNegative))
    return true;
  return false;
}

// get current stop positions
void PathReuseDecider::GetCurrentStopPositions(
    Frame* frame,
    std::vector<const common::PointENU*>* current_stop_positions) {
  auto obstacles = frame->obstacles();
  for (auto obstacle : obstacles) {
    const std::vector<ObjectDecisionType>& current_decisions =
        obstacle->decisions();
    for (auto current_decision : current_decisions) {
      if (current_decision.has_stop())
        current_stop_positions->emplace_back(
            &current_decision.stop().stop_point());
    }
  }
  // sort
  std::sort(current_stop_positions->begin(), current_stop_positions->end(),
            [](const common::PointENU* lhs, const common::PointENU* rhs) {
              return (lhs->x() < rhs->x() ||
                      (lhs->x() == rhs->x() && lhs->y() < rhs->y()));
            });
}

// get current stop obstacle position in s-direction
void PathReuseDecider::GetCurrentStopObstacleS(
    ReferenceLineInfo* const reference_line_info,
    std::vector<double>* current_stop_obstacle) {
  // get all obstacles
  for (auto obstacle :
       reference_line_info->path_decision()->obstacles().Items()) {
    ADEBUG << "current obstacle: "
           << obstacle->PerceptionSLBoundary().start_s();
    if (obstacle->IsLaneBlocking())
      current_stop_obstacle->emplace_back(
          obstacle->PerceptionSLBoundary().start_s());
  }

  // sort w.r.t s
  std::sort(current_stop_obstacle->begin(), current_stop_obstacle->end());
}

// get history stop positions at current reference line
void PathReuseDecider::GetHistoryStopSPosition(
    ReferenceLineInfo* const reference_line_info,
    const std::vector<const HistoryObjectDecision*>& history_objects_decisions,
    std::vector<double>* history_stop_positions) {
  const auto& reference_line = reference_line_info->reference_line();

  for (auto history_object_decision : history_objects_decisions) {
    const std::vector<const ObjectDecisionType*> decisions =
        history_object_decision->GetObjectDecision();

    for (const ObjectDecisionType* decision : decisions) {
      if (decision->has_stop()) {
        common::math::Vec2d stop_position({decision->stop().stop_point().x(),
                                           decision->stop().stop_point().y()});
        common::SLPoint stop_position_sl;

        reference_line.XYToSL(stop_position, &stop_position_sl);
        history_stop_positions->emplace_back(stop_position_sl.s() -
                                             decision->stop().distance_s());

        ADEBUG << "stop_position_x: " << decision->stop().stop_point().x();
        ADEBUG << "stop_position_y: " << decision->stop().stop_point().y();
        ADEBUG << "stop_distance_s: " << decision->stop().distance_s();
        ADEBUG << "stop_distance_s: " << stop_position_sl.s();
        ADEBUG << "adjusted_stop_distance_s: "
               << stop_position_sl.s() - decision->stop().distance_s();
      }
    }
  }
  // sort w.r.t s
  std::sort(history_stop_positions->begin(), history_stop_positions->end());
}

// compare obstacles
bool PathReuseDecider::IsSameObstacles(
    ReferenceLineInfo* const reference_line_info) {
  const auto& history_frame = FrameHistory::Instance()->Latest();
  if (!history_frame) return false;

  const auto& history_reference_line_info =
      history_frame->reference_line_info().front();
  const IndexedList<std::string, Obstacle>& history_obstacles =
      history_reference_line_info.path_decision().obstacles();
  const ReferenceLine& history_reference_line =
      history_reference_line_info.reference_line();
  const ReferenceLine& current_reference_line =
      reference_line_info->reference_line();

  if (reference_line_info->path_decision()->obstacles().Items().size() !=
      history_obstacles.Items().size())
    return false;

  for (auto obstacle :
       reference_line_info->path_decision()->obstacles().Items()) {
    const std::string& obstacle_id = obstacle->Id();
    // same obstacle id
    auto history_obstacle = history_obstacles.Find(obstacle_id);

    if (!history_obstacle ||
        (obstacle->IsStatic() != history_obstacle->IsStatic()) ||
        (IsBlockingDrivingPathObstacle(current_reference_line, obstacle) !=
         IsBlockingDrivingPathObstacle(history_reference_line,
                                       history_obstacle)))
      return false;
  }
  return true;
}  // namespace planning

}  // namespace planning
}  // namespace apollo
