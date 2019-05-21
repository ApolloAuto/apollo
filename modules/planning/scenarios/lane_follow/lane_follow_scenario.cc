/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/planning/scenarios/lane_follow/lane_follow_scenario.h"

#include <vector>

#include "cyber/common/log.h"

#include "modules/planning/scenarios/bare_intersection/unprotected/bare_intersection_unprotected_scenario.h"
#include "modules/planning/scenarios/lane_follow/lane_follow_stage.h"
#include "modules/planning/scenarios/park/pull_over/pull_over_scenario.h"
#include "modules/planning/scenarios/park/valet_parking/valet_parking_scenario.h"
#include "modules/planning/scenarios/stop_sign/unprotected/stop_sign_unprotected_scenario.h"
#include "modules/planning/scenarios/traffic_light/protected/traffic_light_protected_scenario.h"
#include "modules/planning/scenarios/traffic_light/unprotected_left_turn/traffic_light_unprotected_left_turn_scenario.h"
#include "modules/planning/scenarios/traffic_light/unprotected_right_turn/traffic_light_unprotected_right_turn_scenario.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace lane_follow {
namespace {
constexpr ScenarioConfig::ScenarioType kDefaultScenarioType =
    ScenarioConfig::LANE_FOLLOW;
constexpr double kTrafficLightGroupingMaxDist = 2.0;  // unit: m

ScenarioConfig::ScenarioType SelectChangeLaneScenario(
    const Frame& frame, const ScenarioConfigMap& config_map) {
  if (frame.reference_line_info().size() > 1) {
    // TODO(all): to be implemented
    return kDefaultScenarioType;
  }

  return kDefaultScenarioType;
}

ScenarioConfig::ScenarioType SelectPullOverScenario(
    const Frame& frame, const ScenarioConfigMap& config_map) {
  const auto& scenario_config =
      config_map.at(ScenarioConfig::PULL_OVER).pull_over_config();

  const auto& routing = frame.local_view().routing;
  const auto& routing_end = *(routing->routing_request().waypoint().rbegin());

  common::SLPoint dest_sl;
  const auto& reference_line_info = frame.reference_line_info().front();
  const auto& reference_line = reference_line_info.reference_line();
  reference_line.XYToSL({routing_end.pose().x(), routing_end.pose().y()},
                        &dest_sl);
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();

  const double adc_distance_to_dest = dest_sl.s() - adc_front_edge_s;
  ADEBUG << "adc_distance_to_dest[" << adc_distance_to_dest
         << "] destination_s[" << dest_sl.s() << "] adc_front_edge_s["
         << adc_front_edge_s << "]";

  const bool pull_over_scenario =
      (adc_distance_to_dest > 0 &&
       adc_distance_to_dest <=
           scenario_config.start_pull_over_scenario_distance());

  if (pull_over_scenario) {
    return ScenarioConfig::PULL_OVER;
  }

  return kDefaultScenarioType;
}

ScenarioConfig::ScenarioType SelectStopSignScenario(
    const Frame& frame, const ScenarioConfigMap& config_map,
    const hdmap::PathOverlap& stop_sign_overlap) {
  const auto& scenario_config =
      config_map.at(ScenarioConfig::STOP_SIGN_UNPROTECTED)
          .stop_sign_unprotected_config();

  const auto& reference_line_info = frame.reference_line_info().front();
  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  const double adc_distance_to_stop_sign =
      stop_sign_overlap.start_s - adc_front_edge_s;
  ADEBUG << "adc_distance_to_stop_sign[" << adc_distance_to_stop_sign
         << "] stop_sign[" << stop_sign_overlap.object_id
         << "] stop_sign_overlap_start_s[" << stop_sign_overlap.start_s << "]";

  const bool stop_sign_scenario =
      (adc_distance_to_stop_sign > 0 &&
       adc_distance_to_stop_sign <=
           scenario_config.start_stop_sign_scenario_distance());
  const bool stop_sign_all_way = false;  // TODO(all)
  if (stop_sign_scenario) {
    return stop_sign_all_way ? ScenarioConfig::STOP_SIGN_PROTECTED
                             : ScenarioConfig::STOP_SIGN_UNPROTECTED;
  }

  return kDefaultScenarioType;
}

ScenarioConfig::ScenarioType SelectTrafficLightScenario(
    const Frame& frame, const ScenarioConfigMap& config_map,
    const PathOverlapMap& first_encountered_overlap_map,
    const hdmap::PathOverlap& traffic_light_overlap) {
  const auto& scenario_config =
      config_map.at(ScenarioConfig::TRAFFIC_LIGHT_PROTECTED)
          .traffic_light_unprotected_right_turn_config();

  const auto& reference_line_info = frame.reference_line_info().front();

  // first encountered traffic light overlap
  const auto first_encountered_traffic_light_itr =
      first_encountered_overlap_map.find(ReferenceLineInfo::SIGNAL);
  if (first_encountered_traffic_light_itr ==
      first_encountered_overlap_map.end()) {
    return kDefaultScenarioType;
  }
  const auto& first_encountered_traffic_light =
      first_encountered_traffic_light_itr->second;

  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();

  // find all the traffic light belong to
  // the same group as first encountered traffic light
  std::vector<hdmap::PathOverlap> next_traffic_lights;

  const std::vector<hdmap::PathOverlap>& traffic_light_overlaps =
      reference_line_info.reference_line().map_path().signal_overlaps();
  for (const auto& traffic_light_overlap : traffic_light_overlaps) {
    const double dist =
        traffic_light_overlap.start_s - first_encountered_traffic_light.start_s;
    if (std::fabs(dist) <= kTrafficLightGroupingMaxDist) {
      next_traffic_lights.push_back(traffic_light_overlap);
    }
  }

  bool traffic_light_scenario = false;
  bool red_light = false;

  // note: need iterate all lights to check no RED/YELLOW/UNKNOWN
  for (const auto& traffic_light_overlap : next_traffic_lights) {
    const double adc_distance_to_traffic_light =
        first_encountered_traffic_light.start_s - adc_front_edge_s;
    ADEBUG << "traffic_light[" << traffic_light_overlap.object_id
           << "] start_s[" << traffic_light_overlap.start_s
           << "] adc_distance_to_traffic_light["
           << adc_distance_to_traffic_light << "]";

    // enter traffic-light scenarios: based on distance only
    if (adc_distance_to_traffic_light > 0 &&
        adc_distance_to_traffic_light <=
            scenario_config.start_traffic_light_scenario_distance()) {
      traffic_light_scenario = true;

      const auto& signal_color =
          frame.GetSignal(traffic_light_overlap.object_id).color();
      ADEBUG << "traffic_light_id[" << traffic_light_overlap.object_id
             << "] start_s[" << traffic_light_overlap.start_s << "] color["
             << signal_color << "]";

      if (signal_color != perception::TrafficLight::GREEN) {
        red_light = true;
        break;
      }
    }
  }

  if (traffic_light_scenario) {
    const auto& turn_type = reference_line_info.GetPathTurnType(
        first_encountered_traffic_light.start_s);
    const bool right_turn = (turn_type == hdmap::Lane::RIGHT_TURN);
    const bool left_turn = (turn_type == hdmap::Lane::LEFT_TURN);

    if (right_turn && red_light) {
      return ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_RIGHT_TURN;
    }
    if (left_turn) {
      // TODO(all): switch when ready
      // return ScenarioConfig::TRAFFIC_LIGHT_UNPROTECTED_LEFT_TURN;
      return ScenarioConfig::TRAFFIC_LIGHT_PROTECTED;
    }
    return ScenarioConfig::TRAFFIC_LIGHT_PROTECTED;
  }

  return kDefaultScenarioType;
}

ScenarioConfig::ScenarioType SelectYieldSignScenario(
    const Frame& frame, const ScenarioConfigMap& config_map,
    const hdmap::PathOverlap& yield_sign_overlap) {
  // TODO(all): to be implemented
  return kDefaultScenarioType;
}

ScenarioConfig::ScenarioType SelectBareIntersectionScenario(
    const Frame& frame, const ScenarioConfigMap& config_map,
    const hdmap::PathOverlap& pnc_junction_overlap) {
  const auto& reference_line_info = frame.reference_line_info().front();
  if (reference_line_info.GetIntersectionRightofWayStatus(
          pnc_junction_overlap)) {
    return kDefaultScenarioType;
  }

  const auto& scenario_config =
      config_map.at(ScenarioConfig::BARE_INTERSECTION_UNPROTECTED)
          .bare_intersection_unprotected_config();

  const double adc_front_edge_s = reference_line_info.AdcSlBoundary().end_s();
  const double adc_distance_to_pnc_junction =
      pnc_junction_overlap.start_s - adc_front_edge_s;
  ADEBUG << "adc_distance_to_pnc_junction[" << adc_distance_to_pnc_junction
         << "] pnc_junction[" << pnc_junction_overlap.object_id
         << "] pnc_junction_overlap_start_s[" << pnc_junction_overlap.start_s
         << "]";

  const bool bare_junction_scenario =
      (adc_distance_to_pnc_junction > 0 &&
       adc_distance_to_pnc_junction <=
           scenario_config.start_bare_intersection_scenario_distance());

  if (bare_junction_scenario) {
    return ScenarioConfig::BARE_INTERSECTION_UNPROTECTED;
  }

  return kDefaultScenarioType;
}

ScenarioConfig::ScenarioType SelectValetParkingScenario(
    const Frame& frame, const ScenarioConfigMap& config_map) {
  const auto& scenario_config =
      config_map.at(ScenarioConfig::VALET_PARKING).valet_parking_config();

  // TODO(All) triger valet parking by route message definition as of now
  double parking_spot_range_to_start =
      scenario_config.parking_spot_range_to_start();
  if (scenario::valet_parking::ValetParkingScenario::IsTransferable(
          frame, parking_spot_range_to_start)) {
    return ScenarioConfig::VALET_PARKING;
  }

  return kDefaultScenarioType;
}
}  // namespace

std::unique_ptr<Stage> LaneFollowScenario::CreateStage(
    const ScenarioConfig::StageConfig& stage_config) {
  if (stage_config.stage_type() != ScenarioConfig::LANE_FOLLOW_DEFAULT_STAGE) {
    AERROR << "Follow lane does not support stage type: "
           << ScenarioConfig::StageType_Name(stage_config.stage_type());
    return nullptr;
  }
  return std::unique_ptr<Stage>(new LaneFollowStage(stage_config));
}

ScenarioConfig::ScenarioType LaneFollowScenario::UpdateScenarioType(
    const common::TrajectoryPoint& ego_point, const Frame& frame,
    const ScenarioConfigMap& config_map,
    const PathOverlapMap& first_encountered_overlap_map) const {
  // default: LANE_FOLLOW
  ScenarioConfig::ScenarioType scenario_type = kDefaultScenarioType;

  // intersection scenarios
  if (scenario_type == kDefaultScenarioType) {
    hdmap::PathOverlap* traffic_sign_overlap = nullptr;
    hdmap::PathOverlap* pnc_junction_overlap = nullptr;
    ReferenceLineInfo::OverlapType overlap_type;

    const auto& reference_line_info = frame.reference_line_info().front();
    const auto& first_encountered_overlaps =
        reference_line_info.FirstEncounteredOverlaps();
    // note: first_encountered_overlaps already sorted
    for (const auto& overlap : first_encountered_overlaps) {
      if (overlap.first == ReferenceLineInfo::SIGNAL ||
          overlap.first == ReferenceLineInfo::STOP_SIGN ||
          overlap.first == ReferenceLineInfo::YIELD_SIGN) {
        overlap_type = overlap.first;
        traffic_sign_overlap = const_cast<hdmap::PathOverlap*>(&overlap.second);
        break;
      } else if (overlap.first == ReferenceLineInfo::PNC_JUNCTION) {
        pnc_junction_overlap = const_cast<hdmap::PathOverlap*>(&overlap.second);
      }
    }

    if (traffic_sign_overlap) {
      switch (overlap_type) {
        case ReferenceLineInfo::STOP_SIGN:
          if (FLAGS_enable_scenario_stop_sign) {
            scenario_type = SelectStopSignScenario(frame, config_map,
                                                   *traffic_sign_overlap);
          }
          break;
        case ReferenceLineInfo::SIGNAL:
          if (FLAGS_enable_scenario_traffic_light) {
            scenario_type = SelectTrafficLightScenario(
                frame, config_map, first_encountered_overlap_map,
                *traffic_sign_overlap);
          }
          break;
        case ReferenceLineInfo::YIELD_SIGN:
          // TODO(all): to be added
          // scenario_type = SelectYieldSignScenario(
          //     frame, *traffic_sign_overlap);
          break;
        default:
          break;
      }
    } else if (pnc_junction_overlap) {
      // bare intersection
      if (FLAGS_enable_scenario_bare_intersection) {
        scenario_type = SelectBareIntersectionScenario(frame, config_map,
                                                       *pnc_junction_overlap);
      }
    }
  }

  ////////////////////////////////////////
  // CHANGE_LANE scenario
  if (scenario_type == kDefaultScenarioType) {
    scenario_type = SelectChangeLaneScenario(frame, config_map);
  }

  ////////////////////////////////////////
  // pull-over scenario
  if (scenario_type == kDefaultScenarioType) {
    if (FLAGS_enable_scenario_pull_over) {
      scenario_type = SelectPullOverScenario(frame, config_map);
    }
  }

  ////////////////////////////////////////
  // VALET_PARKING scenario
  if (scenario_type == kDefaultScenarioType) {
    scenario_type = SelectValetParkingScenario(frame, config_map);
  }

  return scenario_type;
}

}  // namespace lane_follow
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
