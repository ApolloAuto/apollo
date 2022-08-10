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
#include <memory>
#include <string>

#include "modules/common_msgs/planning_msgs/planning.pb.h"

#include "modules/planning/common/planning_context.h"

namespace apollo {
namespace planning {

using apollo::common::Status;
using apollo::common::math::Polygon2d;
using apollo::common::math::Vec2d;

int PathReuseDecider::reusable_path_counter_ = 0;
int PathReuseDecider::total_path_counter_ = 0;
bool PathReuseDecider::path_reusable_ = false;

PathReuseDecider::PathReuseDecider(
    const TaskConfig& config,
    const std::shared_ptr<DependencyInjector>& injector)
    : Decider(config, injector) {}

Status PathReuseDecider::Process(Frame* const frame,
                                 ReferenceLineInfo* const reference_line_info) {
  // Sanity checks.
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  if (!Decider::config_.path_reuse_decider_config().reuse_path()) {
    ADEBUG << "skipping reusing path: conf";
    reference_line_info->set_path_reusable(false);
    return Status::OK();
  }

  // skip path reuse if not in LANE_FOLLOW_SCENARIO
  const auto scenario_type = injector_->planning_context()
                                 ->planning_status()
                                 .scenario()
                                 .scenario_type();
  if (scenario_type != ScenarioType::LANE_FOLLOW) {
    ADEBUG << "skipping reusing path: not in LANE_FOLLOW scenario";
    reference_line_info->set_path_reusable(false);
    return Status::OK();
  }

  // active path reuse during change_lane only
  auto* lane_change_status = injector_->planning_context()
                                 ->mutable_planning_status()
                                 ->mutable_change_lane();
  ADEBUG << "lane change status: " << lane_change_status->ShortDebugString();

  // skip path reuse if not in_change_lane
  if (lane_change_status->status() != ChangeLaneStatus::IN_CHANGE_LANE &&
      !FLAGS_enable_reuse_path_in_lane_follow) {
    ADEBUG << "skipping reusing path: not in lane_change";
    reference_line_info->set_path_reusable(false);
    return Status::OK();
  }

  // for hybrid model: skip reuse path for valid path reference
  const bool valid_model_output =
      reference_line_info->path_data().is_valid_path_reference();
  if (valid_model_output) {
    ADEBUG << "skipping reusing path: path reference is valid";
    reference_line_info->set_path_reusable(false);
    return Status::OK();
  }

  /*count total_path_ when in_change_lane && reuse_path*/
  ++total_path_counter_;

  /*reuse path when in non_change_lane reference line or
    optimization succeeded in change_lane reference line
  */
  bool is_change_lane_path = reference_line_info->IsChangeLanePath();
  if (is_change_lane_path && !lane_change_status->is_current_opt_succeed()) {
    reference_line_info->set_path_reusable(false);
    ADEBUG << "reusable_path_counter[" << reusable_path_counter_
           << "] total_path_counter[" << total_path_counter_ << "]";
    ADEBUG << "Stop reusing path when optimization failed on change lane path";
    return Status::OK();
  }

  // stop reusing current path:
  // 1. replan path
  // 2. collision
  // 3. failed to trim previous path
  // 4. speed optimization failed on previous path
  bool speed_optimization_successful = false;
  const auto& history_frame = injector_->frame_history()->Latest();
  if (history_frame) {
    const auto history_trajectory_type =
        history_frame->reference_line_info().front().trajectory_type();
    speed_optimization_successful =
        (history_trajectory_type != ADCTrajectory::SPEED_FALLBACK);
  }

  // const auto history_trajectory_type = injector_->FrameHistory()s
  //                                          ->Latest()
  //                                          ->reference_line_info()
  //                                          .front()
  //                                          .trajectory_type();
  if (path_reusable_) {
    if (!frame->current_frame_planned_trajectory().is_replan() &&
        speed_optimization_successful && IsCollisionFree(reference_line_info) &&
        TrimHistoryPath(frame, reference_line_info)) {
      ADEBUG << "reuse path";
      ++reusable_path_counter_;  // count reusable path
    } else {
      // stop reuse path
      ADEBUG << "stop reuse path";
      path_reusable_ = false;
    }
  } else {
    // F -> T
    auto* mutable_path_decider_status = injector_->planning_context()
                                            ->mutable_planning_status()
                                            ->mutable_path_decider();
    static constexpr int kWaitCycle = -2;  // wait 2 cycle

    const int front_static_obstacle_cycle_counter =
        mutable_path_decider_status->front_static_obstacle_cycle_counter();
    const bool ignore_blocking_obstacle =
        IsIgnoredBlockingObstacle(reference_line_info);
    ADEBUG << "counter[" << front_static_obstacle_cycle_counter
           << "] IsIgnoredBlockingObstacle[" << ignore_blocking_obstacle << "]";
    // stop reusing current path:
    // 1. blocking obstacle disappeared or moving far away
    // 2. trimming successful
    // 3. no statical obstacle collision.
    if ((front_static_obstacle_cycle_counter <= kWaitCycle ||
         ignore_blocking_obstacle) &&
        speed_optimization_successful && IsCollisionFree(reference_line_info) &&
        TrimHistoryPath(frame, reference_line_info)) {
      // enable reuse path
      ADEBUG << "reuse path: front_blocking_obstacle ignorable";
      path_reusable_ = true;
      ++reusable_path_counter_;
    }
  }

  reference_line_info->set_path_reusable(path_reusable_);
  ADEBUG << "reusable_path_counter[" << reusable_path_counter_
         << "] total_path_counter[" << total_path_counter_ << "]";
  return Status::OK();
}

bool PathReuseDecider::IsIgnoredBlockingObstacle(
    ReferenceLineInfo* const reference_line_info) {
  const ReferenceLine& reference_line = reference_line_info->reference_line();
  static constexpr double kSDistBuffer = 30.0;  // meter
  static constexpr int kTimeBuffer = 3;         // second
  // vehicle speed
  double adc_speed = injector_->vehicle_state()->linear_velocity();
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
  auto* mutable_path_decider_status = injector_->planning_context()
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
  common::math::Vec2d adc_position = {injector_->vehicle_state()->x(),
                                      injector_->vehicle_state()->y()};
  reference_line.XYToSL(adc_position, adc_position_sl);
}

bool PathReuseDecider::IsCollisionFree(
    ReferenceLineInfo* const reference_line_info) {
  const ReferenceLine& reference_line = reference_line_info->reference_line();
  static constexpr double kMinObstacleArea = 1e-4;
  const double kSBuffer = 0.5;
  static constexpr int kNumExtraTailBoundPoint = 21;
  static constexpr double kPathBoundsDeciderResolution = 0.5;
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

  const auto& history_frame = injector_->frame_history()->Latest();
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
    if (path_end_position_sl.s() - path_position_sl.s() <=
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
          ADEBUG << "s distance to end point:" << path_end_position_sl.s();
          ADEBUG << "s distance to end point:" << path_position_sl.s();
          ADEBUG << "[" << i << "]"
                 << ", history_path[i].x(): " << std::setprecision(9)
                 << history_path[i].x() << ", history_path[i].y()"
                 << std::setprecision(9) << history_path[i].y();
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

// check the length of the path
bool PathReuseDecider::NotShortPath(const DiscretizedPath& current_path) {
  // TODO(shu): use gflag
  static constexpr double kShortPathThreshold = 60;
  return current_path.size() >= kShortPathThreshold;
}

bool PathReuseDecider::TrimHistoryPath(
    Frame* frame, ReferenceLineInfo* const reference_line_info) {
  const ReferenceLine& reference_line = reference_line_info->reference_line();
  const auto& history_frame = injector_->frame_history()->Latest();
  if (!history_frame) {
    ADEBUG << "no history frame";
    return false;
  }

  const common::TrajectoryPoint history_planning_start_point =
      history_frame->PlanningStartPoint();
  common::PathPoint history_init_path_point =
      history_planning_start_point.path_point();
  ADEBUG << "history_init_path_point x:[" << std::setprecision(9)
         << history_init_path_point.x() << "], y["
         << history_init_path_point.y() << "], s: ["
         << history_init_path_point.s() << "]";

  const common::TrajectoryPoint planning_start_point =
      frame->PlanningStartPoint();
  common::PathPoint init_path_point = planning_start_point.path_point();
  ADEBUG << "init_path_point x:[" << std::setprecision(9) << init_path_point.x()
         << "], y[" << init_path_point.y() << "], s: [" << init_path_point.s()
         << "]";

  const DiscretizedPath& history_path =
      history_frame->current_frame_planned_path();
  DiscretizedPath trimmed_path;
  common::SLPoint adc_position_sl;  // current vehicle sl position
  GetADCSLPoint(reference_line, &adc_position_sl);
  ADEBUG << "adc_position_sl.s(): " << adc_position_sl.s();

  size_t path_start_index = 0;

  for (size_t i = 0; i < history_path.size(); ++i) {
    // find previous init point
    if (history_path[i].s() > 0) {
      path_start_index = i;
      break;
    }
  }
  ADEBUG << "!!!path_start_index[" << path_start_index << "]";

  // get current s=0
  common::SLPoint init_path_position_sl;
  reference_line.XYToSL(init_path_point, &init_path_position_sl);
  bool inserted_init_point = false;

  for (size_t i = path_start_index; i < history_path.size(); ++i) {
    common::SLPoint path_position_sl;
    common::math::Vec2d path_position = {history_path[i].x(),
                                         history_path[i].y()};

    reference_line.XYToSL(path_position, &path_position_sl);

    double updated_s = path_position_sl.s() - init_path_position_sl.s();
    // insert init point
    if (updated_s > 0 && !inserted_init_point) {
      trimmed_path.emplace_back(init_path_point);
      trimmed_path.back().set_s(0);
      inserted_init_point = true;
    }

    trimmed_path.emplace_back(history_path[i]);

    // if (i < 50) {
    //   ADEBUG << "path_point:[" << i << "]" << updated_s;
    //   path_position_sl.s();
    //   ADEBUG << std::setprecision(9) << "path_point:[" << i << "]"
    //          << "x: [" << history_path[i].x() << "], y:[" <<
    //          history_path[i].y()
    //          << "]. s[" << history_path[i].s() << "]";
    // }
    trimmed_path.back().set_s(updated_s);
  }

  ADEBUG << "trimmed_path[0]: " << trimmed_path.front().s();
  ADEBUG << "[END] trimmed_path.size(): " << trimmed_path.size();

  if (!NotShortPath(trimmed_path)) {
    ADEBUG << "short path: " << trimmed_path.size();
    return false;
  }

  // set path
  auto path_data = reference_line_info->mutable_path_data();
  ADEBUG << "previous path_data size: " << history_path.size();
  path_data->SetReferenceLine(&reference_line);
  ADEBUG << "previous path_data size: " << path_data->discretized_path().size();
  path_data->SetDiscretizedPath(DiscretizedPath(std::move(trimmed_path)));
  ADEBUG << "not short path: " << trimmed_path.size();
  ADEBUG << "current path size: "
         << reference_line_info->path_data().discretized_path().size();

  return true;
}

}  // namespace planning
}  // namespace apollo
