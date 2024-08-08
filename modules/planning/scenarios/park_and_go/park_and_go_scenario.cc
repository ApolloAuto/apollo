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

#include "modules/planning/scenarios/park_and_go/park_and_go_scenario.h"

#include "cyber/common/log.h"
#include "modules/common/util/point_factory.h"
#include "modules/map/hdmap/hdmap_util.h"
#include "modules/planning/scenarios/park_and_go/stage_adjust.h"
#include "modules/planning/scenarios/park_and_go/stage_check.h"
#include "modules/planning/scenarios/park_and_go/stage_cruise.h"
#include "modules/planning/scenarios/park_and_go/stage_pre_cruise.h"

namespace apollo {
namespace planning {

using apollo::hdmap::HDMapUtil;

bool ParkAndGoScenario::Init(std::shared_ptr<DependencyInjector> injector,
                             const std::string& name) {
  if (init_) {
    return true;
  }

  if (!Scenario::Init(injector, name)) {
    AERROR << "failed to init scenario" << Name();
    return false;
  }

  if (!Scenario::LoadConfig<apollo::planning::ScenarioParkAndGoConfig>(
          &context_.scenario_config)) {
    AERROR << "fail to get config of scenario" << Name();
    return false;
  }

  init_ = true;
  return true;
}

bool ParkAndGoScenario::IsTransferable(const Scenario* const other_scenario,
                                       const Frame& frame) {
  if (!frame.local_view().planning_command->has_lane_follow_command()) {
    AINFO << "PARK_AND_GO: Don't has lane follow command!";
    return false;
  }
  if (other_scenario == nullptr || frame.reference_line_info().empty()) {
    AINFO << "PARK_AND_GO: Don't has reference line info or other scenario!";
    return false;
  }
  bool park_and_go = false;
  const auto& scenario_config = context_.scenario_config;
  const auto vehicle_state_provider = injector_->vehicle_state();
  common::VehicleState vehicle_state = vehicle_state_provider->vehicle_state();
  auto adc_point = common::util::PointFactory::ToPointENU(vehicle_state);
  // TODO(SHU) might consider gear == GEAR_PARKING
  double adc_speed = vehicle_state_provider->linear_velocity();
  double s = 0.0;
  double l = 0.0;
  const double max_abs_speed_when_stopped =
      common::VehicleConfigHelper::Instance()
          ->GetConfig()
          .vehicle_param()
          .max_abs_speed_when_stopped();

  hdmap::LaneInfoConstPtr lane;

  // check ego vehicle distance to destination
  const auto routing_end = frame.local_view().end_lane_way_point;
  if (nullptr == routing_end) {
    AINFO << "PARK_AND_GO: Don't has end lane way point!";
    return false;
  }
  common::SLPoint dest_sl;
  const auto& reference_line_info = frame.reference_line_info().front();
  const auto& reference_line = reference_line_info.reference_line();
  reference_line.XYToSL(routing_end->pose(), &dest_sl);
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();

  bool is_ego_on_lane = false;
  bool is_lane_type_city_driving = false;
  HDMapUtil::BaseMap().GetNearestLaneWithDistance(
      adc_point, 5.0, &lane, &s, &l);
  if (lane != nullptr && lane->IsOnLane({adc_point.x(), adc_point.y()})) {
    is_ego_on_lane = true;
    if (lane->lane().type() == hdmap::Lane::CITY_DRIVING) {
      is_lane_type_city_driving = true;
    }
  }

  const double adc_distance_to_dest = dest_sl.s() - adc_front_edge_s;
  ADEBUG << "adc_distance_to_dest:" << adc_distance_to_dest;
  bool is_vehicle_static = (std::fabs(adc_speed) < max_abs_speed_when_stopped);
  bool is_distance_far_enough =
      (adc_distance_to_dest > scenario_config.min_dist_to_dest());
  AINFO << "PARK_AND_GO: " << is_vehicle_static << "," << is_distance_far_enough
        << "," << is_ego_on_lane << "," << is_lane_type_city_driving;
  // if vehicle is static, far enough to destination and (off-lane or not on
  // city_driving lane)
  if (std::fabs(adc_speed) < max_abs_speed_when_stopped &&
      adc_distance_to_dest > scenario_config.min_dist_to_dest() &&
      (!is_ego_on_lane || !is_lane_type_city_driving)) {
    park_and_go = true;
  }
  return park_and_go;
}

}  // namespace planning
}  // namespace apollo
