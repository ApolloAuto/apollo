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

#include "modules/planning/scenarios/pull_over/util.h"

#include "modules/planning/scenarios/pull_over/proto/pull_over.pb.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/util/util.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/map/pnc_map/path.h"
#include "modules/planning/planning_base/common/planning_context.h"
#include "modules/planning/planning_base/common/reference_line_info.h"

namespace apollo {
namespace planning {

using apollo::common::math::Vec2d;

/**
 * @brief: check adc parked properly
 */
PullOverState CheckADCPullOver(
    const common::VehicleStateProvider* vehicle_state_provider,
    const ReferenceLineInfo& reference_line_info,
    const ScenarioPullOverConfig& scenario_config,
    const PlanningContext* planning_context) {
  const auto& pull_over_status =
      planning_context->planning_status().pull_over();
  if (!pull_over_status.has_position() ||
      !pull_over_status.position().has_x() ||
      !pull_over_status.position().has_y() || !pull_over_status.has_theta()) {
    ADEBUG << "pull_over status not set properly: "
           << pull_over_status.DebugString();
    return PullOverState::UNKNOWN;
  }

  const auto& reference_line = reference_line_info.reference_line();
  common::SLPoint pull_over_sl;
  reference_line.XYToSL(pull_over_status.position(), &pull_over_sl);

  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  double distance = adc_front_edge_s - pull_over_sl.s();
  if (distance >= scenario_config.pass_destination_threshold()) {
    ADEBUG << "ADC passed pull-over spot: distance[" << distance << "]";
    return PullOverState::PASS_DESTINATION;
  }

  const double adc_speed = vehicle_state_provider->linear_velocity();
  const double max_adc_stop_speed = common::VehicleConfigHelper::Instance()
                                        ->GetConfig()
                                        .vehicle_param()
                                        .max_abs_speed_when_stopped();
  if (adc_speed > max_adc_stop_speed) {
    ADEBUG << "ADC not stopped: speed[" << adc_speed << "]";
    return PullOverState::APPROACHING;
  }

  static constexpr double kStartParkCheckRange = 3.0;  // meter
  if (distance <= -kStartParkCheckRange) {
    ADEBUG << "ADC still far: distance[" << distance << "]";
    return PullOverState::APPROACHING;
  }

  const common::math::Vec2d adc_position = {vehicle_state_provider->x(),
                                            vehicle_state_provider->y()};
  const common::math::Vec2d target_position = {pull_over_status.position().x(),
                                               pull_over_status.position().y()};

  const bool position_check = CheckPullOverPositionBySL(
      reference_line_info, scenario_config, adc_position,
      vehicle_state_provider->heading(), target_position,
      pull_over_status.theta(), true);

  return position_check ? PullOverState::PARK_COMPLETE
                        : PullOverState::PARK_FAIL;
}

/**
 * @brief: check path data to see properly
 */
PullOverState CheckADCPullOverPathPoint(
    const ReferenceLineInfo& reference_line_info,
    const ScenarioPullOverConfig& scenario_config,
    const common::PathPoint& path_point,
    const PlanningContext* planning_context) {
  const auto& pull_over_status =
      planning_context->planning_status().pull_over();
  if (!pull_over_status.has_position() ||
      !pull_over_status.position().has_x() ||
      !pull_over_status.position().has_y() || !pull_over_status.has_theta()) {
    ADEBUG << "pull_over status not set properly: "
           << pull_over_status.DebugString();
    return PullOverState::UNKNOWN;
  }

  const common::math::Vec2d target_position = {pull_over_status.position().x(),
                                               pull_over_status.position().y()};
  const bool position_check = CheckPullOverPositionBySL(
      reference_line_info, scenario_config, {path_point.x(), path_point.y()},
      path_point.theta(), target_position, pull_over_status.theta(),
      false);  // check l + theta only

  return position_check ? PullOverState::PARK_COMPLETE
                        : PullOverState::PARK_FAIL;
}

bool CheckPullOverPositionBySL(const ReferenceLineInfo& reference_line_info,
                               const ScenarioPullOverConfig& scenario_config,
                               const common::math::Vec2d& adc_position,
                               const double adc_theta,
                               const common::math::Vec2d& target_position,
                               const double target_theta, const bool check_s) {
  const auto& reference_line = reference_line_info.reference_line();
  common::SLPoint target_sl;
  reference_line.XYToSL(target_position, &target_sl);
  common::SLPoint adc_position_sl;
  reference_line.XYToSL(adc_position, &adc_position_sl);

  const double s_diff = target_sl.s() - adc_position_sl.s();
  const double l_diff = std::fabs(target_sl.l() - adc_position_sl.l());
  const double theta_diff =
      std::fabs(common::math::NormalizeAngle(target_theta - adc_theta));

  ADEBUG << "adc_position_s[" << adc_position_sl.s() << "] adc_position_l["
         << adc_position_sl.l() << "] target_s[" << target_sl.s()
         << "] target_l[" << target_sl.l() << "] s_diff[" << s_diff
         << "] l_diff[" << l_diff << "] theta_diff[" << theta_diff << "]";

  // check s/l/theta diff
  bool ret = (l_diff <= scenario_config.max_l_error_to_end_point() &&
              theta_diff <= scenario_config.max_theta_error_to_end_point());
  if (check_s) {
    ret = (ret && s_diff >= 0 &&
           s_diff <= scenario_config.max_s_error_to_end_point());
  }

  return ret;
}

}  // namespace planning
}  // namespace apollo
