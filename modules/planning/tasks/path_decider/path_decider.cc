/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/tasks/path_decider/path_decider.h"

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "modules/planning/proto/decision.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/vehicle_state/vehicle_state.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;

PathDecider::PathDecider() : Task("PathDecider") {}

apollo::common::Status PathDecider::Execute(
    Frame *, ReferenceLineInfo *reference_line_info) {
  Task::Execute(nullptr, reference_line_info);
  return Process(reference_line_info->path_data(),
                 reference_line_info->path_decision());
}

Status PathDecider::Process(const PathData &path_data,
                            PathDecision *const path_decision) {
  CHECK_NOTNULL(path_decision);
  if (!MakeObjectDecision(path_data, path_decision)) {
    AERROR << "Failed to make decision based on tunnel";
    return Status(ErrorCode::PLANNING_ERROR, "dp_road_graph decision ");
  }
  return Status::OK();
}

bool PathDecider::MakeObjectDecision(const PathData &path_data,
                                     PathDecision *const path_decision) {
  DCHECK_NOTNULL(path_decision);
  if (!MakeStaticObstacleDecision(path_data, path_decision)) {
    AERROR << "Failed to make decisions for static obstacles";
    return false;
  }
  return true;
}

bool PathDecider::MakeStaticObstacleDecision(
    const PathData &path_data, PathDecision *const path_decision) {
  DCHECK_NOTNULL(path_decision);
  std::vector<common::SLPoint> adc_sl_points;
  const auto &vehicle_param =
      common::VehicleConfigHelper::instance()->GetConfig().vehicle_param();
  const auto &reference_line = reference_line_info_->reference_line();

  const double adc_max_edge_to_center_dist =
      std::hypot(std::max(vehicle_param.front_edge_to_center(),
                          vehicle_param.back_edge_to_center()),
                 std::max(vehicle_param.left_edge_to_center(),
                          vehicle_param.right_edge_to_center()));

  for (const auto &path_point : path_data.discretized_path().path_points()) {
    common::SLPoint adc_sl;
    if (!reference_line.XYToSL({path_point.x(), path_point.y()}, &adc_sl)) {
      AERROR << "get_point_in_Frenet_frame error for ego vehicle "
             << path_point.x() << " " << path_point.y();
      return false;
    } else {
      adc_sl_points.push_back(std::move(adc_sl));
    }
  }

  const double adc_allowed_s_radius =
      adc_max_edge_to_center_dist + FLAGS_static_decision_ignore_s_range;
  const double adc_allowed_l_radius =
      adc_max_edge_to_center_dist + FLAGS_static_decision_nudge_l_buffer;
  for (const auto *path_obstacle : path_decision->path_obstacles().Items()) {
    const auto *obstacle = path_obstacle->obstacle();
    if (!obstacle->IsStatic()) {
      continue;
    }

    // IGNORE by default
    ObjectDecisionType object_decision;
    object_decision.mutable_ignore();

    const auto &sl_boundary = path_obstacle->perception_sl_boundary();
    bool has_stop = false;
    for (const auto &adc_sl : adc_sl_points) {
      if (adc_sl.s() + adc_allowed_s_radius < sl_boundary.start_s() ||
          adc_sl.s() - adc_allowed_s_radius > sl_boundary.end_s()) {
        // ignore: no overlap in s direction
        continue;
      }

      if (adc_sl.l() + adc_allowed_l_radius < sl_boundary.start_l() ||
          adc_sl.l() - adc_allowed_l_radius > sl_boundary.end_l()) {
        // ignore: no overlap in l direction
        continue;
      }

      // check STOP/NUDGE
      double left_width = 0.0;
      double right_width = 0.0;
      if (!reference_line.GetLaneWidth(adc_sl.s(), &left_width, &right_width)) {
        left_width = right_width = FLAGS_default_reference_line_width / 2;
      }
      double driving_width;
      bool left_nudgable = false;
      bool right_nudgable = false;
      if (sl_boundary.start_l() >= 0) {
        // obstacle on the left, check RIGHT_NUDGE
        driving_width = sl_boundary.start_l() -
                        FLAGS_static_decision_nudge_l_buffer + right_width;
        right_nudgable =
            (driving_width >= FLAGS_min_driving_width) ? true : false;
      } else if (sl_boundary.end_l() <= 0) {
        // obstacle on the right, check LEFT_NUDGE
        driving_width = std::fabs(sl_boundary.end_l()) -
                        FLAGS_static_decision_nudge_l_buffer + left_width;
        left_nudgable =
            (driving_width >= FLAGS_min_driving_width) ? true : false;
      } else {
        // obstacle across the central line, decide RIGHT_NUDGE/LEFT_NUDGE
        double driving_width_left = left_width - sl_boundary.end_l() -
                                    FLAGS_static_decision_nudge_l_buffer;
        double driving_width_right = right_width -
                                     std::fabs(sl_boundary.start_l()) -
                                     FLAGS_static_decision_nudge_l_buffer;
        if (std::max(driving_width_right, driving_width_left) >=
            FLAGS_min_driving_width) {
          // nudgable
          left_nudgable =
              driving_width_left > driving_width_right ? true : false;
          right_nudgable = !left_nudgable;
        }
      }

      if (!left_nudgable && !right_nudgable) {
        // STOP: and break
        ObjectStop *object_stop_ptr = object_decision.mutable_stop();
        double stop_distance = 0;
        if (obstacle->Id() == FLAGS_destination_obstacle_id) {
          // destination
          object_stop_ptr->set_reason_code(
              StopReasonCode::STOP_REASON_DESTINATION);
          stop_distance = FLAGS_stop_distance_destination;
        } else {
          // static obstacle
          object_stop_ptr->set_reason_code(
              StopReasonCode::STOP_REASON_OBSTACLE);
          stop_distance = FLAGS_stop_distance_obstacle;
        }
        object_stop_ptr->set_distance_s(-stop_distance);

        double stop_ref_s = sl_boundary.start_s() - stop_distance;
        auto stop_ref_point = reference_line.GetReferencePoint(stop_ref_s);
        object_stop_ptr->mutable_stop_point()->set_x(stop_ref_point.x());
        object_stop_ptr->mutable_stop_point()->set_y(stop_ref_point.y());
        object_stop_ptr->set_stop_heading(stop_ref_point.heading());

        has_stop = true;
        break;
      } else {
        // NUDGE: and continue to check potential STOP along the ref line
        if (!FLAGS_enable_nudge_decision) {
          continue;
        }

        if (!object_decision.has_nudge()) {
          ObjectNudge *object_nudge_ptr = object_decision.mutable_nudge();
          if (left_nudgable) {
            // LEFT_NUDGE
            object_nudge_ptr->set_type(ObjectNudge::LEFT_NUDGE);
            object_nudge_ptr->set_distance_l(FLAGS_nudge_distance_obstacle);
          } else {
            // RIGHT_NUDGE
            object_nudge_ptr->set_type(ObjectNudge::RIGHT_NUDGE);
            object_nudge_ptr->set_distance_l(-FLAGS_nudge_distance_obstacle);
          }
        }
      }
    }

    if (has_stop) {
      // STOP
      path_decision->AddLongitudinalDecision("PathDecider", obstacle->Id(),
                                             object_decision);
    } else {
      // NUDGE / IGNORE
      path_decision->AddLateralDecision("PathDecider", obstacle->Id(),
                                        object_decision);
    }
  }

  return true;
}

}  // namespace planning
}  // namespace apollo
