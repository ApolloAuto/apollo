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

#include "modules/planning/traffic_rules/traffic_decider.h"

#include <limits>

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/traffic_rules/backside_vehicle.h"
#include "modules/planning/traffic_rules/crosswalk.h"
#include "modules/planning/traffic_rules/destination.h"
#include "modules/planning/traffic_rules/keep_clear.h"
#include "modules/planning/traffic_rules/reference_line_end.h"
#include "modules/planning/traffic_rules/rerouting.h"
#include "modules/planning/traffic_rules/stop_sign.h"
#include "modules/planning/traffic_rules/traffic_light.h"
#include "modules/planning/traffic_rules/yield_sign.h"

namespace apollo {
namespace planning {
using apollo::common::Status;

apollo::common::util::Factory<TrafficRuleConfig::RuleId, TrafficRule,
                              TrafficRule *(*)(const TrafficRuleConfig &config)>
    TrafficDecider::s_rule_factory;

void TrafficDecider::RegisterRules() {
  s_rule_factory.Register(TrafficRuleConfig::BACKSIDE_VEHICLE,
                          [](const TrafficRuleConfig &config) -> TrafficRule * {
                            return new BacksideVehicle(config);
                          });
  s_rule_factory.Register(TrafficRuleConfig::CROSSWALK,
                          [](const TrafficRuleConfig &config) -> TrafficRule * {
                            return new Crosswalk(config);
                          });

  s_rule_factory.Register(TrafficRuleConfig::DESTINATION,
                          [](const TrafficRuleConfig &config) -> TrafficRule * {
                            return new Destination(config);
                          });
  s_rule_factory.Register(TrafficRuleConfig::KEEP_CLEAR,
                          [](const TrafficRuleConfig &config) -> TrafficRule * {
                            return new KeepClear(config);
                          });
  s_rule_factory.Register(TrafficRuleConfig::REFERENCE_LINE_END,
                          [](const TrafficRuleConfig &config) -> TrafficRule * {
                            return new ReferenceLineEnd(config);
                          });
  s_rule_factory.Register(TrafficRuleConfig::REROUTING,
                          [](const TrafficRuleConfig &config) -> TrafficRule * {
                            return new Rerouting(config);
                          });
  s_rule_factory.Register(TrafficRuleConfig::STOP_SIGN,
                          [](const TrafficRuleConfig &config) -> TrafficRule * {
                            return new StopSign(config);
                          });
  s_rule_factory.Register(TrafficRuleConfig::YIELD_SIGN,
                          [](const TrafficRuleConfig &config) -> TrafficRule * {
                            return new YieldSign(config);
                          });
  s_rule_factory.Register(TrafficRuleConfig::TRAFFIC_LIGHT,
                          [](const TrafficRuleConfig &config) -> TrafficRule * {
                            return new TrafficLight(config);
                          });
}

bool TrafficDecider::Init(const TrafficRuleConfigs &config) {
  if (s_rule_factory.Empty()) {
    RegisterRules();
  }
  rule_configs_ = config;
  return true;
}

void TrafficDecider::BuildPlanningTarget(
    ReferenceLineInfo *reference_line_info) {
  double min_s = std::numeric_limits<double>::infinity();
  StopPoint stop_point;
  for (const auto *obstacle :
       reference_line_info->path_decision()->obstacles().Items()) {
    if (obstacle->IsVirtual() && obstacle->HasLongitudinalDecision() &&
        obstacle->LongitudinalDecision().has_stop() &&
        obstacle->PerceptionSLBoundary().start_s() < min_s) {
      min_s = obstacle->PerceptionSLBoundary().start_s();
      const auto &stop_code =
          obstacle->LongitudinalDecision().stop().reason_code();
      if (stop_code == StopReasonCode::STOP_REASON_DESTINATION ||
          stop_code == StopReasonCode::STOP_REASON_CROSSWALK ||
          stop_code == StopReasonCode::STOP_REASON_STOP_SIGN ||
          stop_code == StopReasonCode::STOP_REASON_YIELD_SIGN ||
          stop_code == StopReasonCode::STOP_REASON_CREEPER ||
          stop_code == StopReasonCode::STOP_REASON_REFERENCE_END ||
          stop_code == StopReasonCode::STOP_REASON_SIGNAL) {
        stop_point.set_type(StopPoint::HARD);
        ADEBUG << "Hard stop at: " << min_s
               << "REASON: " << StopReasonCode_Name(stop_code);
      } else if (stop_code == StopReasonCode::STOP_REASON_YELLOW_SIGNAL) {
        stop_point.set_type(StopPoint::SOFT);
        ADEBUG << "Soft stop at: " << min_s << "  STOP_REASON_YELLOW_SIGNAL";
      } else {
        ADEBUG << "No planning target found at reference line.";
      }
    }
  }
  if (min_s != std::numeric_limits<double>::infinity()) {
    const auto &vehicle_config =
        common::VehicleConfigHelper::Instance()->GetConfig();
    double front_edge_to_center =
        vehicle_config.vehicle_param().front_edge_to_center();
    stop_point.set_s(min_s - front_edge_to_center +
                     FLAGS_virtual_stop_wall_length / 2.0);
    reference_line_info->SetLatticeStopPoint(stop_point);
  }
}

Status TrafficDecider::Execute(Frame *frame,
                               ReferenceLineInfo *reference_line_info) {
  CHECK_NOTNULL(frame);
  CHECK_NOTNULL(reference_line_info);

  for (const auto &rule_config : rule_configs_.config()) {
    if (!rule_config.enabled()) {
      ADEBUG << "Rule " << rule_config.rule_id() << " not enabled";
      continue;
    }
    auto rule = s_rule_factory.CreateObject(rule_config.rule_id(), rule_config);
    if (!rule) {
      AERROR << "Could not find rule " << rule_config.DebugString();
      continue;
    }
    rule->ApplyRule(frame, reference_line_info);
    ADEBUG << "Applied rule "
           << TrafficRuleConfig::RuleId_Name(rule_config.rule_id());
  }

  BuildPlanningTarget(reference_line_info);
  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
