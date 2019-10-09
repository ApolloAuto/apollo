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

#include "modules/planning/common/planning_context.h"

namespace apollo {
namespace planning {
// #define ADEBUG AINFO

using apollo::common::Status;
using apollo::common::math::Polygon2d;
using apollo::common::math::Vec2d;

int PathReuseDecider::reusable_path_counter_ = 0;
int PathReuseDecider::total_path_counter_ = 0;

PathReuseDecider::PathReuseDecider(const TaskConfig& config)
    : Decider(config) {}

Status PathReuseDecider::Process(Frame* const frame,
                                 ReferenceLineInfo* const reference_line_info) {
  if (!Decider::config_.path_reuse_decider_config().reuse_path()) {
    return Status::OK();
  }
  // Sanity checks.
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  // check front static blocking obstacle
  auto* mutable_path_reuse_decider_status = PlanningContext::Instance()
                                                ->mutable_planning_status()
                                                ->mutable_path_reuse_decider();
  auto* mutable_path_decider_status = PlanningContext::Instance()
                                          ->mutable_planning_status()
                                          ->mutable_path_decider();
  constexpr int kWaitCycle = -2;  // wait 2 cycle
  ADEBUG << "reuse or not: "
         << mutable_path_reuse_decider_status->reused_path();
  // T -> F
  if (mutable_path_reuse_decider_status->reused_path()) {
    ADEBUG << "reused path";
    if (CheckPathReusable(frame, reference_line_info)) {
      ADEBUG << "no collision";
      ++reusable_path_counter_;  // count reusable path
      if (!TrimHistoryPath(frame, reference_line_info))
        AERROR << "Failed to trim reused path";
    } else {
      // disable reuse path
      ADEBUG << "collision!!!";
      mutable_path_reuse_decider_status->set_reused_path(false);
    }
  } else {
    // F -> T
    ADEBUG
        << "counter: "
        << mutable_path_decider_status->front_static_obstacle_cycle_counter();
    // far from blocking obstacle or no blocking obstacle for a while
    if (mutable_path_decider_status->front_static_obstacle_cycle_counter() <
            kWaitCycle ||
        IsIgnoredBlockingObstacle(reference_line_info)) {
      // enable reuse path
      mutable_path_reuse_decider_status->set_reused_path(true);
    }
  }
  ++total_path_counter_;
  ADEBUG << "reusable_path_counter_" << reusable_path_counter_;
  ADEBUG << "total_path_counter_" << total_path_counter_;
  return Status::OK();
}

bool PathReuseDecider::CheckPathReusable(
    Frame* const frame, ReferenceLineInfo* const reference_line_info) {
  ADEBUG << "Check Collision";
  return IsCollisionFree(reference_line_info);
}

bool PathReuseDecider::IsIgnoredBlockingObstacle(
    ReferenceLineInfo* const reference_line_info) {
  const ReferenceLine& reference_line = reference_line_info->reference_line();
  constexpr double kSDistBuffer = 30.0;  // meter
  constexpr int kTimeBuffer = 3;         // second
  // vehicle speed
  double adc_speed =
      common::VehicleStateProvider::Instance()->linear_velocity();
  double final_s_buffer = std::max(kSDistBuffer, kTimeBuffer * adc_speed);
  // current vehicle s position
  common::SLPoint adc_position_sl;
  GetADCSLPoint(reference_line, &adc_position_sl);
  // blocking obstacle start s
  double blocking_obstacle_start_s;
  if (GetBlockingObstacleS(reference_line_info, &blocking_obstacle_start_s) &&
      // distance to blocking obstacle
      (blocking_obstacle_start_s - adc_position_sl.s() > final_s_buffer)) {
    ADEBUG << "blocking obstacle distance: "
           << blocking_obstacle_start_s - adc_position_sl.s();
    return true;
  } else {
    return false;
  }
}

bool PathReuseDecider::GetBlockingObstacleS(
    ReferenceLineInfo* const reference_line_info, double* blocking_obstacle_s) {
  auto* mutable_path_decider_status = PlanningContext::Instance()
                                          ->mutable_planning_status()
                                          ->mutable_path_decider();
  // get blocking obstacle ID (front_static_obstacle_id)
  const std::string& blocking_obstacle_ID =
      mutable_path_decider_status->front_static_obstacle_id();
  const IndexedList<std::string, Obstacle>& indexed_obstacles =
      reference_line_info->path_decision()->obstacles();
  const auto* blocking_obstacle = indexed_obstacles.Find(blocking_obstacle_ID);

  if (blocking_obstacle == nullptr) {
    return false;
  }

  const auto& obstacle_sl = blocking_obstacle->PerceptionSLBoundary();
  *blocking_obstacle_s = obstacle_sl.start_s();
  ADEBUG << "blocking obstacle distance: " << obstacle_sl.start_s();
  return true;
}

void PathReuseDecider::GetADCSLPoint(const ReferenceLine& reference_line,
                                     common::SLPoint* adc_position_sl) {
  common::math::Vec2d adc_position = {
      common::VehicleStateProvider::Instance()->x(),
      common::VehicleStateProvider::Instance()->y()};
  reference_line.XYToSL(adc_position, adc_position_sl);
}

bool PathReuseDecider::IsCollisionFree(
    ReferenceLineInfo* const reference_line_info) {
  const ReferenceLine& reference_line = reference_line_info->reference_line();
  constexpr double kMinObstacleArea = 1e-4;
  const double kSBuffer = 0.5;
  constexpr int kNumExtraTailBoundPoint = 20;
  constexpr double kPathBoundsDeciderResolution = 0.5;
  // current vehicle sl position
  common::SLPoint adc_position_sl;
  GetADCSLPoint(reference_line, &adc_position_sl);

  // current obstacles
  std::vector<Polygon2d> obstacle_polygons;
  for (auto obstacle :
       reference_line_info->path_decision()->obstacles().Items()) {
    // filtered all non-static objects and virtual obstacle
    if (!obstacle->IsStatic() || obstacle->IsVirtual()) {
      if (!obstacle->IsStatic()) {
        ADEBUG << "SPOT a dynamic obstacle";
      }
      if (obstacle->IsVirtual()) {
        ADEBUG << "SPOT a virtual obstacle";
      }
      continue;
    }

    const auto& obstacle_sl = obstacle->PerceptionSLBoundary();
    // Ignore obstacles behind ADC
    if ((obstacle_sl.end_s() < adc_position_sl.s() - kSBuffer) ||
        // Ignore too small obstacles.
        (obstacle_sl.end_s() - obstacle_sl.start_s()) *
                (obstacle_sl.end_l() - obstacle_sl.start_l()) <
            kMinObstacleArea) {
      continue;
    }
    obstacle_polygons.push_back(
        Polygon2d({Vec2d(obstacle_sl.start_s(), obstacle_sl.start_l()),
                   Vec2d(obstacle_sl.start_s(), obstacle_sl.end_l()),
                   Vec2d(obstacle_sl.end_s(), obstacle_sl.end_l()),
                   Vec2d(obstacle_sl.end_s(), obstacle_sl.start_l())}));
  }

  if (obstacle_polygons.empty()) {
    return true;
  }

  const auto& history_frame = FrameHistory::Instance()->Latest();
  if (!history_frame) {
    return false;
  }
  const DiscretizedPath& history_path =
      history_frame->current_frame_planned_path();
  // path end point
  common::SLPoint path_end_position_sl;
  common::math::Vec2d path_end_position = {history_path.back().x(),
                                           history_path.back().y()};
  reference_line.XYToSL(path_end_position, &path_end_position_sl);
  for (size_t i = 0; i < history_path.size(); ++i) {
    common::SLPoint path_position_sl;
    common::math::Vec2d path_position = {history_path[i].x(),
                                         history_path[i].y()};
    reference_line.XYToSL(path_position, &path_position_sl);
    if (path_end_position_sl.s() - path_position_sl.s() <
        kNumExtraTailBoundPoint * kPathBoundsDeciderResolution) {
      break;
    }
    if (path_position_sl.s() < adc_position_sl.s() - kSBuffer) {
      continue;
    }
    const auto& vehicle_box =
        common::VehicleConfigHelper::Instance()->GetBoundingBox(
            history_path[i]);
    std::vector<Vec2d> ABCDpoints = vehicle_box.GetAllCorners();
    for (const auto& corner_point : ABCDpoints) {
      // For each corner point, project it onto reference_line
      common::SLPoint curr_point_sl;
      if (!reference_line.XYToSL(corner_point, &curr_point_sl)) {
        AERROR << "Failed to get the projection from point onto "
                  "reference_line";
        return false;
      }
      auto curr_point = Vec2d(curr_point_sl.s(), curr_point_sl.l());
      // Check if it's in any polygon of other static obstacles.
      for (const auto& obstacle_polygon : obstacle_polygons) {
        if (obstacle_polygon.IsPointIn(curr_point)) {
          // for debug
          ADEBUG << "s distance to end point:"
                 << path_end_position_sl.s() - path_position_sl.s();
          ADEBUG << "collision:" << curr_point.x() << ", " << curr_point.y();
          Vec2d xy_point;
          reference_line.SLToXY(curr_point_sl, &xy_point);
          ADEBUG << "collision:" << xy_point.x() << ", " << xy_point.y();

          return false;
        }
      }
    }
  }
  return true;
}

bool PathReuseDecider::TrimHistoryPath(
    Frame* frame, ReferenceLineInfo* const reference_line_info) {
  const ReferenceLine& reference_line = reference_line_info->reference_line();
  const auto& history_frame = FrameHistory::Instance()->Latest();
  if (!history_frame) {
    return false;
  }

  const DiscretizedPath& history_path =
      history_frame->current_frame_planned_path();
  DiscretizedPath trimmed_path;
  // current vehicle sl position
  common::SLPoint adc_position_sl;
  GetADCSLPoint(reference_line, &adc_position_sl);
  double path_start_s = 0.0;
  size_t path_start_index = 0;

  for (size_t i = 0; i < history_path.size(); ++i) {
    common::SLPoint path_position_sl;
    common::math::Vec2d path_position = {history_path[i].x(),
                                         history_path[i].y()};
    reference_line.XYToSL(path_position, &path_position_sl);
    if (path_position_sl.s() < adc_position_sl.s()) {
      path_start_s = path_position_sl.s();
      ++path_start_index;
    } else {
      double updated_s = history_path[i].s() - path_start_s;
      trimmed_path.emplace_back(history_path[i]);
      trimmed_path.back().set_s(updated_s);
    }
  }
  trimmed_path.insert(trimmed_path.begin(), history_path[path_start_index]);
  frame->set_current_frame_planned_path(trimmed_path);
  return true;
}

}  // namespace planning
}  // namespace apollo
