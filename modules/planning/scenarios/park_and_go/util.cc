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

#include "modules/planning/scenarios/park_and_go/util.h"

#include "modules/planning/planning_base/common/frame.h"

namespace apollo {
namespace planning {

using apollo::common::math::Box2d;
using apollo::common::math::Polygon2d;
using apollo::common::math::Vec2d;

bool CheckADCReadyToCruise(
    const common::VehicleStateProvider* vehicle_state_provider, Frame* frame,
    const apollo::planning::ScenarioParkAndGoConfig& scenario_config) {
  auto vehicle_status = vehicle_state_provider;
  common::math::Vec2d adc_position = {vehicle_status->x(), vehicle_status->y()};
  const double adc_heading = vehicle_status->heading();

  common::SLPoint adc_position_sl;
  // get nearest reference line
  const auto& reference_line_list = frame->reference_line_info();
  const auto reference_line_info = std::min_element(
      reference_line_list.begin(), reference_line_list.end(),
      [&](const ReferenceLineInfo& ref_a, const ReferenceLineInfo& ref_b) {
        common::SLPoint adc_position_sl_a;
        common::SLPoint adc_position_sl_b;
        ref_a.reference_line().XYToSL(adc_position, &adc_position_sl_a);
        ref_b.reference_line().XYToSL(adc_position, &adc_position_sl_b);
        return std::fabs(adc_position_sl_a.l()) <
               std::fabs(adc_position_sl_b.l());
      });
  reference_line_info->reference_line().XYToSL(adc_position, &adc_position_sl);
  bool is_near_front_obstacle =
      CheckADCSurroundObstacles(adc_position, adc_heading, frame,
                                scenario_config.front_obstacle_buffer());
  bool heading_align_w_reference_line =
      CheckADCHeading(adc_position, adc_heading, *reference_line_info,
                      scenario_config.heading_buffer());
  ADEBUG << "is_near_front_obstacle: " << is_near_front_obstacle;
  ADEBUG << "heading_align_w_reference_line: "
         << heading_align_w_reference_line;
  // check gear status
  // TODO(SHU): align with vehicle parameters
  static constexpr double kMinSpeed = 0.1;  // m/s
  return ((vehicle_status->gear() == canbus::Chassis::GEAR_DRIVE ||
           std::fabs(vehicle_status->vehicle_state().linear_velocity()) <
               kMinSpeed) &&
          !is_near_front_obstacle && heading_align_w_reference_line &&
          std::fabs(adc_position_sl.l()) < 0.5);
}

/**
 * @brief: front obstacle is far enough before PardAndGo cruise stage
 *(adc_position: center of rear wheel)
 */
bool CheckADCSurroundObstacles(const common::math::Vec2d adc_position,
                               const double adc_heading, Frame* frame,
                               const double front_obstacle_buffer) {
  const auto& vehicle_config =
      common::VehicleConfigHelper::Instance()->GetConfig();
  const double adc_length = vehicle_config.vehicle_param().length();
  const double adc_width = vehicle_config.vehicle_param().width();
  // ADC box
  Box2d adc_box(adc_position, adc_heading, adc_length + front_obstacle_buffer,
                adc_width);
  double shift_distance = front_obstacle_buffer / 2 +
                          vehicle_config.vehicle_param().back_edge_to_center();
  Vec2d shift_vec{shift_distance * std::cos(adc_heading),
                  shift_distance * std::sin(adc_heading)};
  adc_box.Shift(shift_vec);
  const auto& adc_polygon = Polygon2d(adc_box);
  // obstacle boxes
  auto obstacles = frame->obstacles();
  for (const auto& obstacle : obstacles) {
    if (obstacle->IsVirtual()) {
      continue;
    }
    const auto& obstacle_polygon = obstacle->PerceptionPolygon();
    const Polygon2d& nudge_polygon = obstacle_polygon.ExpandByDistance(
        std::fabs(FLAGS_static_obstacle_nudge_l_buffer));
    if (adc_polygon.HasOverlap(nudge_polygon)) {
      ADEBUG << "blocked obstacle: " << obstacle->Id();
      return true;
    }
  }
  return false;
}

/**
 * @brief Park_and_go: heading angle should be close to reference line before
 * PardAndGo cruise stage
 */
bool CheckADCHeading(const common::math::Vec2d adc_position,
                     const double adc_heading,
                     const ReferenceLineInfo& reference_line_info,
                     const double heading_diff_to_reference_line) {
  const double kReducedHeadingBuffer = 0.2;  // (rad) TODO(Shu) move to config
  const auto& reference_line = reference_line_info.reference_line();
  common::SLPoint adc_position_sl;
  reference_line.XYToSL(adc_position, &adc_position_sl);
  // reference line heading angle at s
  const auto reference_point =
      reference_line.GetReferencePoint(adc_position_sl.s());
  const auto path_point = reference_point.ToPathPoint(adc_position_sl.s());
  AINFO << "heading difference: "
        << common::math::NormalizeAngle(adc_heading - path_point.theta());
  double angle_difference =
      common::math::NormalizeAngle(adc_heading - path_point.theta());
  if (angle_difference >
          -1.0 * (heading_diff_to_reference_line - kReducedHeadingBuffer) &&
      angle_difference < heading_diff_to_reference_line) {
    return true;
  }
  return false;
}

}  // namespace planning
}  // namespace apollo
