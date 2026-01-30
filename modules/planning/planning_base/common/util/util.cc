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

#include "modules/planning/planning_base/common/util/util.h"

#include <algorithm>
#include <limits>
#include <vector>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"

namespace apollo {
namespace planning {
namespace util {

using apollo::common::VehicleState;
using apollo::hdmap::PathOverlap;

bool IsVehicleStateValid(const VehicleState& vehicle_state) {
  if (std::isnan(vehicle_state.x()) || std::isnan(vehicle_state.y()) ||
      std::isnan(vehicle_state.z()) || std::isnan(vehicle_state.heading()) ||
      std::isnan(vehicle_state.kappa()) ||
      std::isnan(vehicle_state.linear_velocity()) ||
      std::isnan(vehicle_state.linear_acceleration())) {
    return false;
  }
  return true;
}

bool IsDifferentRouting(const PlanningCommand& first,
                        const PlanningCommand& second) {
  if (first.has_header() && second.has_header()) {
    const auto& first_header = first.header();
    const auto& second_header = second.header();
    return (first_header.sequence_num() != second_header.sequence_num() ||
            first_header.module_name() != second_header.module_name() ||
            first_header.timestamp_sec() != second_header.timestamp_sec());
  }
  return true;
}

double GetADCStopDeceleration(
    apollo::common::VehicleStateProvider* vehicle_state,
    const double adc_front_edge_s, const double stop_line_s) {
  double adc_speed = vehicle_state->linear_velocity();
  const double max_adc_stop_speed = common::VehicleConfigHelper::Instance()
                                        ->GetConfig()
                                        .vehicle_param()
                                        .max_abs_speed_when_stopped();
  if (adc_speed < max_adc_stop_speed) {
    return 0.0;
  }

  double stop_distance = 0;

  if (stop_line_s > adc_front_edge_s) {
    stop_distance = stop_line_s - adc_front_edge_s;
  }
  if (stop_distance < 1e-5) {
    return std::numeric_limits<double>::max();
  }
  return (adc_speed * adc_speed) / (2 * stop_distance);
}

/*
 * @brief: check if a stop_sign_overlap is still along reference_line
 */
bool CheckStopSignOnReferenceLine(const ReferenceLineInfo& reference_line_info,
                                  const std::string& stop_sign_overlap_id) {
  const std::vector<PathOverlap>& stop_sign_overlaps =
      reference_line_info.reference_line().map_path().stop_sign_overlaps();
  auto stop_sign_overlap_it =
      std::find_if(stop_sign_overlaps.begin(), stop_sign_overlaps.end(),
                   [&stop_sign_overlap_id](const PathOverlap& overlap) {
                     return overlap.object_id == stop_sign_overlap_id;
                   });
  return (stop_sign_overlap_it != stop_sign_overlaps.end());
}

/*
 * @brief: check if a traffic_light_overlap is still along reference_line
 */
bool CheckTrafficLightOnReferenceLine(
    const ReferenceLineInfo& reference_line_info,
    const std::string& traffic_light_overlap_id) {
  const std::vector<PathOverlap>& traffic_light_overlaps =
      reference_line_info.reference_line().map_path().signal_overlaps();
  auto traffic_light_overlap_it =
      std::find_if(traffic_light_overlaps.begin(), traffic_light_overlaps.end(),
                   [&traffic_light_overlap_id](const PathOverlap& overlap) {
                     return overlap.object_id == traffic_light_overlap_id;
                   });
  return (traffic_light_overlap_it != traffic_light_overlaps.end());
}

/*
 * @brief: check if ADC is till inside a pnc-junction
 */
bool CheckInsideJunction(const ReferenceLineInfo& reference_line_info) {
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  const double adc_back_edge_s = reference_line_info.AdcSlBoundary().start_s();

  hdmap::PathOverlap junction_overlap;
  reference_line_info.GetJunction(adc_front_edge_s, &junction_overlap);
  if (junction_overlap.object_id.empty()) {
    return false;
  }

  static constexpr double kIntersectionPassDist = 2.0;  // unit: m
  const double distance_adc_pass_intersection =
      adc_back_edge_s - junction_overlap.end_s;
  ADEBUG << "distance_adc_pass_intersection[" << distance_adc_pass_intersection
         << "] junction_overlap[" << junction_overlap.object_id << "] start_s["
         << junction_overlap.start_s << "]";

  return distance_adc_pass_intersection < kIntersectionPassDist;
}

/*
 * @brief: get files at a path
 */
void GetFilesByPath(const boost::filesystem::path& path,
                    std::vector<std::string>* files) {
  ACHECK(files);
  if (!boost::filesystem::exists(path)) {
    return;
  }
  if (boost::filesystem::is_regular_file(path)) {
    AINFO << "Found record file: " << path.c_str();
    files->push_back(path.c_str());
    return;
  }
  if (boost::filesystem::is_directory(path)) {
    for (auto& entry : boost::make_iterator_range(
             boost::filesystem::directory_iterator(path), {})) {
      GetFilesByPath(entry.path(), files);
    }
  }
}

/*
 * @brief: get path equivalent ego width
 */
double CalculateEquivalentEgoWidth(const ReferenceLineInfo& reference_line_info,
                                   double s, bool* is_left) {
  const auto& vehicle_param =
      common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param();
  double max_kappa = 1.0 / vehicle_param.min_turn_radius();
  double half_wb = 0.5 * vehicle_param.wheel_base();
  double front_l = vehicle_param.front_edge_to_center() - half_wb;
  double half_w = 0.5 * vehicle_param.width();
  double current_heading =
      reference_line_info.reference_line().GetReferencePoint(s).heading();
  double heading_f = reference_line_info.reference_line()
                         .GetReferencePoint(s + front_l)
                         .heading();
  double heading_b = reference_line_info.reference_line()
                         .GetReferencePoint(s - half_wb)
                         .heading();
  // average kappa from front edge to center
  double kappa_f =
      apollo::common::math::NormalizeAngle(heading_f - current_heading) /
      front_l;
  // average kappa from center to rear axle
  double kappa_b =
      apollo::common::math::NormalizeAngle(current_heading - heading_b) /
      half_wb;
  *is_left = (kappa_b < 0.0);
  // prevent devide by 0, and quit if lane bends to difference direction
  if (kappa_f * kappa_b < 0.0 || fabs(kappa_f) < 1e-6) {
    return half_w;
  }

  kappa_f = std::min(fabs(kappa_f), max_kappa);
  kappa_b = std::min(fabs(kappa_b), max_kappa);
  double theta = apollo::common::math::Vec2d(1.0, half_wb * kappa_b).Angle();
  double sint = std::sin(theta);
  double cost = std::cos(theta);
  double r_f = 1.0 / kappa_f;
  double eq_half_w =
      apollo::common::math::Vec2d(front_l * cost - half_w * sint,
                                  r_f + front_l * sint + half_w * cost)
          .Length() -
      r_f;
  return std::max(eq_half_w, half_w);
}

double CalculateEquivalentEgoWidth(
    const apollo::hdmap::LaneInfoConstPtr lane_info, double s, bool* is_left) {
  const auto& vehicle_param =
      common::VehicleConfigHelper::Instance()->GetConfig().vehicle_param();
  double max_kappa = 1.0 / vehicle_param.min_turn_radius();
  double half_wb = 0.5 * vehicle_param.wheel_base();
  double front_l = vehicle_param.front_edge_to_center() - half_wb;
  double half_w = 0.5 * vehicle_param.width();
  double current_heading = lane_info->Heading(s);
  double heading_f = lane_info->Heading(s + front_l);
  double heading_b = lane_info->Heading(s - half_wb);
  // average kappa from front edge to center
  double kappa_f =
      apollo::common::math::NormalizeAngle(heading_f - current_heading) /
      front_l;
  // average kappa from center to rear axle
  double kappa_b =
      apollo::common::math::NormalizeAngle(current_heading - heading_b) /
      half_wb;
  *is_left = (kappa_b < 0.0);
  // prevent devide by 0, and quit if lane bends to difference direction
  if (kappa_f * kappa_b < 0.0 || fabs(kappa_f) < 1e-6) {
    return half_w;
  }

  kappa_f = std::min(fabs(kappa_f), max_kappa);
  kappa_b = std::min(fabs(kappa_b), max_kappa);
  double theta = apollo::common::math::Vec2d(1.0, half_wb * kappa_b).Angle();
  double sint = std::sin(theta);
  double cost = std::cos(theta);
  double r_f = 1.0 / kappa_f;
  double eq_half_w =
      apollo::common::math::Vec2d(front_l * cost - half_w * sint,
                                  r_f + front_l * sint + half_w * cost)
          .Length() -
      r_f;
  return std::max(eq_half_w, half_w);
}

bool left_arc_bound_with_heading(double delta_x, double r, double heading,
                                 double* result) {
  // calculate △L(positive or negative) with an arc with given radius, and given
  // init_heading the circle can be written as (x-Rsin)^2 + (y+Rcos)^2 = R^2 the
  // upper side of the circel = (sqrt(R^2 - (x-Rsin)^2) - Rcos) is what we need
  if (delta_x > r * (1.0 + std::sin(heading)) - 1e-6) {
    *result = std::numeric_limits<double>::lowest();
    return false;
  }

  *result = std::sqrt(r * r - std::pow(delta_x - r * std::sin(heading), 2)) -
            r * std::cos(heading);
  return true;
}

bool right_arc_bound_with_heading(double delta_x, double r, double heading,
                                  double* result) {
  // calculate △L(positive or negative) with an arc with given radius, and given
  // init_heading the circle can be written as (x+Rsin)^2 + (y-Rcos)^2 = R^2 the
  // upper side of the circel = (Rcos - sqrt(R^2 - (x+Rsin)^2) )is what we need
  if (delta_x > r * (1.0 - std::sin(heading)) - 1e-6) {
    *result = std::numeric_limits<double>::max();
    return false;
  }
  *result = r * std::cos(heading) -
            std::sqrt(r * r - std::pow(delta_x + r * std::sin(heading), 2));
  return true;
}

bool left_arc_bound_with_heading_with_reverse_kappa(double delta_x, double r,
                                                    double heading,
                                                    double kappa,
                                                    double* result) {
  // calculate △L(positive or negative) with an arc with given radius, and given
  // init_heading the circle can be written as (x-Rsin)^2 + (y+Rcos)^2 = R^2 the
  // upper side of the circel = (sqrt(R^2 - (x-Rsin)^2) - Rcos) is what we need
  if (heading > 0 || kappa < 0 ||
      delta_x > r * (1.0 - std::sin(heading)) - 1e-6) {
    *result = std::numeric_limits<double>::lowest();
    return false;
  }
  if (delta_x < -r * std::sin(heading)) {
    *result = r * std::cos(heading) -
              std::sqrt(r * r - std::pow(delta_x - r * std::sin(heading), 2));
  } else {
    *result = std::sqrt(r * r - std::pow(delta_x + r * std::sin(heading), 2)) -
              r * (2 - std::cos(heading));
  }
  return true;
}

bool right_arc_bound_with_heading_with_reverse_kappa(double delta_x, double r,
                                                     double heading,
                                                     double kappa,
                                                     double* result) {
  // calculate △L(positive or negative) with an arc with given radius, and given
  // init_heading the circle can be written as (x+Rsin)^2 + (y-Rcos)^2 = R^2 the
  // upper side of the circel = (Rcos - sqrt(R^2 - (x+Rsin)^2) )is what we need
  if (heading < 0 || kappa > 0 ||
      delta_x > r * (1.0 - std::sin(heading)) - 1e-6) {
    *result = std::numeric_limits<double>::max();
    return false;
  }
  if (delta_x < r * std::sin(heading)) {
    *result = std::sqrt(r * r - std::pow(delta_x - r * std::sin(heading), 2)) -
              r * std::cos(heading);
  } else {
    *result = r * (2 - std::cos(heading)) -
              std::sqrt(r * r - std::pow(delta_x - r * std::sin(heading), 2));
  }
  return true;
}

}  // namespace util
}  // namespace planning
}  // namespace apollo
