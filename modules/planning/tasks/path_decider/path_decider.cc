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

#include <string>
#include <utility>
#include <vector>

#include "modules/planning/proto/decision.pb.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::ErrorCode;
using apollo::common::Status;

namespace {
const double kTimeSampleInterval = 0.1;
}

PathDecider::PathDecider() : Task("PathDecider") {}

apollo::common::Status PathDecider::Execute(
    Frame *, ReferenceLineInfo *reference_line_info) {
  reference_line_ = &reference_line_info->reference_line();
  speed_data_ = &reference_line_info->speed_data();
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
  if (!FLAGS_enable_nudge_decision) {
    return true;
  }

  DCHECK_NOTNULL(path_decision);
  std::vector<common::SLPoint> adc_sl_points;
  std::vector<common::math::Box2d> adc_bounding_box;
  const auto &vehicle_param =
      common::VehicleConfigHelper::instance()->GetConfig().vehicle_param();
  const double adc_length = vehicle_param.length();
  const double adc_width = vehicle_param.length();
  const double adc_max_edge_to_center_dist =
      std::hypot(std::max(vehicle_param.front_edge_to_center(),
                          vehicle_param.back_edge_to_center()),
                 std::max(vehicle_param.left_edge_to_center(),
                          vehicle_param.right_edge_to_center()));

  for (const common::PathPoint &path_point :
       path_data.discretized_path().path_points()) {
    adc_bounding_box.emplace_back(
        common::math::Vec2d{path_point.x(), path_point.y()}, path_point.theta(),
        adc_length, adc_width);
    common::SLPoint adc_sl;
    if (!reference_line_->XYToSL({path_point.x(), path_point.y()}, &adc_sl)) {
      AERROR << "get_point_in_Frenet_frame error for ego vehicle "
             << path_point.x() << " " << path_point.y();
      return false;
    } else {
      adc_sl_points.push_back(std::move(adc_sl));
    }
  }

  for (const auto *path_obstacle : path_decision->path_obstacles().Items()) {
    const auto *obstacle = path_obstacle->Obstacle();
    if (obstacle->IsVirtual()) {
      continue;
    }
    if (!obstacle->IsStatic()) {
      continue;
    }

    // IGNORE by default
    ObjectDecisionType object_decision;
    object_decision.mutable_ignore();

    const auto &sl_boundary = path_obstacle->perception_sl_boundary();
    bool has_stop = false;
    for (std::size_t j = 0; j < adc_sl_points.size(); ++j) {
      const auto &adc_sl = adc_sl_points[j];
      if (adc_sl.s() + adc_max_edge_to_center_dist +
                  FLAGS_static_decision_ignore_s_range <
              sl_boundary.start_s() ||
          adc_sl.s() - adc_max_edge_to_center_dist -
                  FLAGS_static_decision_ignore_s_range >
              sl_boundary.end_s()) {
        // ignore: no overlap in s direction
        continue;
      }

      if (adc_sl.l() + adc_max_edge_to_center_dist +
                  FLAGS_static_decision_nudge_l_buffer <
              sl_boundary.start_l() ||
          adc_sl.l() - adc_max_edge_to_center_dist -
                  FLAGS_static_decision_nudge_l_buffer >
              sl_boundary.end_l()) {
        // ignore: no overlap in l direction
        continue;
      }

      // check STOP/NUDGE
      double left_width;
      double right_width;
      if (!reference_line_->get_lane_width(adc_sl.s(), &left_width,
                                           &right_width)) {
        left_width = right_width = FLAGS_default_reference_line_width / 2;
      }
      double driving_width;
      bool left_nudgable = false;
      bool right_nudgable = false;
      if (sl_boundary.start_l() >= 0) {
        // obstacle on the left, check RIGHT_NUDGE
        driving_width = sl_boundary.start_l() -
            FLAGS_static_decision_nudge_l_buffer + right_width;
        right_nudgable = (driving_width >= FLAGS_min_driving_width) ?
            true : false;
      } else if (sl_boundary.end_l() <= 0) {
        // obstacle on the right, check LEFT_NUDGE
        driving_width = std::fabs(sl_boundary.end_l()) -
            FLAGS_static_decision_nudge_l_buffer + left_width;
        left_nudgable = (driving_width >= FLAGS_min_driving_width) ?
            true : false;
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
          left_nudgable = driving_width_left > driving_width_right ?
              true : false;
          right_nudgable = !left_nudgable;
        }
      }

      if (!left_nudgable && !right_nudgable) {
        // STOP: and break
        ObjectDecisionType stop_decision;
        ObjectStop *object_stop_ptr = stop_decision.mutable_stop();
        object_stop_ptr->set_distance_s(-FLAGS_stop_distance_obstacle);
        object_stop_ptr->set_reason_code(StopReasonCode::STOP_REASON_OBSTACLE);

        auto stop_ref_s = sl_boundary.start_s() - FLAGS_stop_distance_obstacle;
        auto stop_ref_point = reference_line_->get_reference_point(stop_ref_s);
        object_stop_ptr->mutable_stop_point()->set_x(stop_ref_point.x());
        object_stop_ptr->mutable_stop_point()->set_y(stop_ref_point.y());
        object_stop_ptr->set_stop_heading(stop_ref_point.heading());
        path_decision->AddLongitudinalDecision("DpRoadGraph", obstacle->Id(),
                                               stop_decision);

        has_stop = true;
        break;
      } else {
        // NUDGE: and continue to check potential STOP along the ref line
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
    if (!has_stop) {
      path_decision->AddLateralDecision("DpRoadGraph", obstacle->Id(),
                                        object_decision);
    }
  }

  return true;
}

bool PathDecider::MakeDynamicObstcleDecision(
    const PathData &path_data, PathDecision *const path_decision) {
  // Compute dynamic obstacle decision
  const double interval = kTimeSampleInterval;
  const double total_time =
      std::min(speed_data_->TotalTime(), FLAGS_prediction_total_time);
  std::size_t evaluate_time_slots =
      static_cast<std::size_t>(std::floor(total_time / interval));
  // list of Box2d for adc given speed profile
  std::vector<common::math::Box2d> adc_by_time;
  if (!ComputeBoundingBoxesForAdc(path_data.frenet_frame_path(),
                                  evaluate_time_slots, &adc_by_time)) {
    AERROR << "fill adc_by_time error";
    return false;
  }

  for (const auto *path_obstacle : path_decision->path_obstacles().Items()) {
    const auto *obstacle = path_obstacle->Obstacle();
    if (obstacle->IsVirtual()) {
      continue;
    }
    if (obstacle->IsStatic()) {
      continue;
    }
    double timestamp = 0.0;
    for (std::size_t k = 0; k < evaluate_time_slots;
         ++k, timestamp += interval) {
      // TODO(all) make nudge decision for dynamic obstacles.
      // const auto obstacle_by_time =
      //     obstacle->GetBoundingBox(obstacle->GetPointAtTime(timestamp));
    }
  }
  return true;
}

bool PathDecider::ComputeBoundingBoxesForAdc(
    const FrenetFramePath &frenet_frame_path,
    const std::size_t evaluate_time_slots,
    std::vector<common::math::Box2d> *adc_boxes) {
  CHECK(adc_boxes != nullptr);

  const auto &vehicle_config =
      common::VehicleConfigHelper::instance()->GetConfig();
  double adc_length = vehicle_config.vehicle_param().length();
  double adc_width = vehicle_config.vehicle_param().length();

  double time_stamp = 0.0;
  const double interval = kTimeSampleInterval;
  for (std::size_t i = 0; i < evaluate_time_slots;
       ++i, time_stamp += interval) {
    common::SpeedPoint speed_point;
    if (!speed_data_->EvaluateByTime(time_stamp, &speed_point)) {
      AINFO << "get_speed_point_with_time for time_stamp[" << time_stamp << "]";
      return false;
    }

    const common::FrenetFramePoint &interpolated_frenet_point =
        frenet_frame_path.EvaluateByS(speed_point.s());
    double s = interpolated_frenet_point.s();
    double l = interpolated_frenet_point.l();
    double dl = interpolated_frenet_point.dl();

    common::math::Vec2d adc_position_cartesian;
    common::SLPoint sl_point;
    sl_point.set_s(s);
    sl_point.set_l(l);
    reference_line_->SLToXY(sl_point, &adc_position_cartesian);

    ReferencePoint reference_point = reference_line_->get_reference_point(s);

    double one_minus_kappa_r_d = 1 - reference_point.kappa() * l;
    double delta_theta = std::atan2(dl, one_minus_kappa_r_d);
    double theta = ::apollo::common::math::NormalizeAngle(
        delta_theta + reference_point.heading());

    adc_boxes->emplace_back(adc_position_cartesian, theta, adc_length,
                            adc_width);
  }
  return true;
}

}  // namespace planning
}  // namespace apollo
