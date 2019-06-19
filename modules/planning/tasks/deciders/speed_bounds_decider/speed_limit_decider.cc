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

#include "modules/planning/tasks/deciders/speed_bounds_decider/speed_limit_decider.h"

#include <limits>
#include <tuple>

#include "modules/common/proto/pnc_point.pb.h"
#include "modules/planning/proto/decision.pb.h"

#include "cyber/common/log.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/util/util.h"
#include "modules/planning/common/planning_gflags.h"

namespace apollo {
namespace planning {

using apollo::common::Status;

SpeedLimitDecider::SpeedLimitDecider(const SpeedBoundsDeciderConfig& config,
                                     const ReferenceLine& reference_line,
                                     const PathData& path_data)
    : speed_bounds_config_(config),
      reference_line_(reference_line),
      path_data_(path_data),
      vehicle_param_(common::VehicleConfigHelper::GetConfig().vehicle_param()) {
}

// TODO(all): remove the code; use kappa from path point directly.
void SpeedLimitDecider::GetAvgKappa(
    const std::vector<common::PathPoint>& path_points,
    std::vector<double>* kappa) const {
  CHECK_NOTNULL(kappa);
  const int kHalfNumPoints = speed_bounds_config_.num_points_to_avg_kappa() / 2;
  CHECK_GT(kHalfNumPoints, 0);
  kappa->clear();
  kappa->resize(path_points.size());
  double sum = 0.0;
  int start = 0;
  int end = 0;
  while (end < static_cast<int>(path_points.size()) &&
         end - start < kHalfNumPoints + 1) {
    sum += path_points[end].kappa();
    ++end;
  }

  int iter = 0;
  while (iter < static_cast<int>(path_points.size())) {
    kappa->at(iter) = sum / (end - start);
    if (start < iter - kHalfNumPoints) {
      sum -= path_points[start].kappa();
      ++start;
    }
    if (end < static_cast<int>(path_points.size())) {
      sum += path_points[end].kappa();
      ++end;
    }
    ++iter;
  }
}

Status SpeedLimitDecider::GetSpeedLimits(
    const IndexedList<std::string, Obstacle>& obstacles,
    SpeedLimit* const speed_limit_data) const {
  CHECK_NOTNULL(speed_limit_data);

  std::vector<double> avg_kappa;
  GetAvgKappa(path_data_.discretized_path(), &avg_kappa);
  const auto& discretized_path = path_data_.discretized_path();
  const auto& frenet_path = path_data_.frenet_frame_path();

  for (uint32_t i = 0; i < discretized_path.size(); ++i) {
    const double path_s = discretized_path.at(i).s();
    const double reference_line_s = frenet_path.at(i).s();
    if (reference_line_s > reference_line_.Length()) {
      AWARN << "path w.r.t. reference line at [" << reference_line_s
            << "] is LARGER than reference_line_ length ["
            << reference_line_.Length() << "]. Please debug before proceeding.";
      break;
    }

    // (1) speed limit from map
    double speed_limit_from_reference_line =
        reference_line_.GetSpeedLimitFromS(reference_line_s);

    // (2) speed limit from path curvature
    //  -- 2.1: limit by centripetal force (acceleration)
    const double speed_limit_from_centripetal_acc =
        std::sqrt(GetCentricAccLimit(std::fabs(avg_kappa[i])) /
                  std::fmax(std::fabs(avg_kappa[i]),
                            speed_bounds_config_.minimal_kappa()));

    // TODO(all): remove this; this is not necessary nor making sense.
    // -- 2.2: limit by centripetal jerk
    double centri_jerk_speed_limit = std::numeric_limits<double>::max();
    if (i + 1 < discretized_path.size()) {
      const double ds =
          discretized_path.at(i + 1).s() - discretized_path.at(i).s();
      DCHECK_GE(ds, 0.0);
      const double kEpsilon = 1e-9;
      const double centri_jerk =
          std::fabs(avg_kappa[i + 1] - avg_kappa[i]) / (ds + kEpsilon);
      centri_jerk_speed_limit = std::fmax(
          10.0, speed_bounds_config_.centri_jerk_speed_coeff() / centri_jerk);
    }

    // (3) speed limit from nudge obstacles
    // TODO(all): in future, expand the speed limit not only to obstacles with
    // nudge decisions.
    double speed_limit_from_nearby_obstacles =
        std::numeric_limits<double>::max();
    const double collision_safety_range =
        speed_bounds_config_.collision_safety_range();
    for (const auto* ptr_obstacle : obstacles.Items()) {
      if (ptr_obstacle->IsVirtual()) {
        continue;
      }
      if (!ptr_obstacle->LateralDecision().has_nudge()) {
        continue;
      }

      /* ref line:
       * -------------------------------
       *    start_s   end_s
       * ------|  adc   |---------------
       * ------------|  obstacle |------
       */

      // TODO(all): potential problem here;
      // frenet and cartesian coordinates are mixed.
      const double vehicle_front_s =
          reference_line_s + vehicle_param_.front_edge_to_center();
      const double vehicle_back_s =
          reference_line_s - vehicle_param_.back_edge_to_center();
      const double obstacle_front_s =
          ptr_obstacle->PerceptionSLBoundary().end_s();
      const double obstacle_back_s =
          ptr_obstacle->PerceptionSLBoundary().start_s();

      if (vehicle_front_s < obstacle_back_s ||
          vehicle_back_s > obstacle_front_s) {
        continue;
      }

      const auto& nudge_decision = ptr_obstacle->LateralDecision().nudge();

      // Please notice the differences between adc_l and frenet_point_l
      const double frenet_point_l = frenet_path.at(i).l();

      // obstacle is on the right of ego vehicle (at path point i)
      bool is_close_on_left =
          (nudge_decision.type() == ObjectNudge::LEFT_NUDGE) &&
          (frenet_point_l - vehicle_param_.right_edge_to_center() -
               collision_safety_range <
           ptr_obstacle->PerceptionSLBoundary().end_l());

      // obstacle is on the left of ego vehicle (at path point i)
      bool is_close_on_right =
          (nudge_decision.type() == ObjectNudge::RIGHT_NUDGE) &&
          (ptr_obstacle->PerceptionSLBoundary().start_l() -
               collision_safety_range <
           frenet_point_l + vehicle_param_.left_edge_to_center());

      // TODO(all): dynamic obstacles do not have nudge decision
      if (is_close_on_left || is_close_on_right) {
        double nudge_speed_ratio = 1.0;
        if (ptr_obstacle->IsStatic()) {
          nudge_speed_ratio =
              speed_bounds_config_.static_obs_nudge_speed_ratio();
        } else {
          nudge_speed_ratio =
              speed_bounds_config_.dynamic_obs_nudge_speed_ratio();
        }
        speed_limit_from_nearby_obstacles =
            nudge_speed_ratio * speed_limit_from_reference_line;
        break;
      }
    }
    double curr_speed_limit = 0.0;
    if (FLAGS_enable_nudge_slowdown) {
      curr_speed_limit = std::fmax(
          speed_bounds_config_.lowest_speed(),
          common::util::MinElement(std::vector<double>{
              speed_limit_from_reference_line, speed_limit_from_centripetal_acc,
              centri_jerk_speed_limit, speed_limit_from_nearby_obstacles}));
    } else {
      curr_speed_limit = std::fmax(
          speed_bounds_config_.lowest_speed(),
          common::util::MinElement(std::vector<double>{
              speed_limit_from_reference_line, speed_limit_from_centripetal_acc,
              centri_jerk_speed_limit}));
    }
    speed_limit_data->AppendSpeedLimit(path_s, curr_speed_limit);
  }
  return Status::OK();
}

double SpeedLimitDecider::GetCentricAccLimit(const double kappa) const {
  // this function uses a linear model with upper and lower bound to determine
  // centric acceleration limit

  // suppose acc = k1 * v + k2
  // consider acc = v ^ 2 * kappa
  // we determine acc by the two functions above, with upper and lower speed
  // bounds
  const double v_high = speed_bounds_config_.high_speed_threshold();
  const double v_low = speed_bounds_config_.low_speed_threshold();

  // TODO(all): remove this; use a unified centripetal acceleration limit
  const double h_v_acc =
      speed_bounds_config_.high_speed_centric_acceleration_limit();
  const double l_v_acc =
      speed_bounds_config_.low_speed_centric_acceleration_limit();

  if (std::fabs(v_high - v_low) < 1.0) {
    AERROR << "High speed and low speed threshold are too close to each other. "
              "Please check config file."
           << " Current high speed threshold = " << v_high
           << ", current low speed threshold = " << v_low;
    return h_v_acc;
  }
  const double kMinKappaEpsilon = 1e-9;
  if (kappa < kMinKappaEpsilon) {
    return h_v_acc;
  }

  const double k1 = (h_v_acc - l_v_acc) / (v_high - v_low);
  const double k2 = h_v_acc - v_high * k1;

  const double v = (k1 + std::sqrt(k1 * k1 + 4.0 * kappa * k2)) / (2.0 * kappa);
  ADEBUG << "v = " << v;

  if (v > v_high) {
    return h_v_acc;
  } else if (v < v_low) {
    return l_v_acc;
  } else {
    return v * k1 + k2;
  }
}

}  // namespace planning
}  // namespace apollo
