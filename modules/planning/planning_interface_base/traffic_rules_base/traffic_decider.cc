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

#include "modules/planning/planning_interface_base/traffic_rules_base/traffic_decider.h"

#include <limits>
#include <memory>

#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/planning/planning_base/common/util/config_util.h"
#include "modules/planning/planning_base/gflags/planning_gflags.h"
namespace apollo {
namespace planning {
using apollo::common::Status;

bool TrafficDecider::Init(const std::shared_ptr<DependencyInjector> &injector) {
  if (init_) return true;
  // Load the pipeline config.
  AINFO << "Load config path:" << FLAGS_traffic_rule_config_filename;
  // Load the pipeline of scenario.
  if (!apollo::cyber::common::LoadConfig(FLAGS_traffic_rule_config_filename,
                                         &rule_pipeline_)) {
    AERROR << "Load pipeline of Traffic decider"
           << " failed!";
    return false;
  }
  //-----------------------------------------------
  for (int i = 0; i < rule_pipeline_.rule_size(); i++) {
    auto rule =
        apollo::cyber::plugin_manager::PluginManager::Instance()
            ->CreateInstance<TrafficRule>(ConfigUtil::GetFullPlanningClassName(
                rule_pipeline_.rule(i).type()));
    if (!rule) {
      AERROR << "Init of Traffic rule" << rule_pipeline_.rule(i).name()
             << " failed!";
      return false;
    }
    rule->Init(rule_pipeline_.rule(i).name(), injector);
    rule_list_.push_back(rule);
  }

  init_ = true;
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

  for (const auto &rule : rule_list_) {
    if (!rule) {
      AERROR << "Could not find rule ";
      continue;
    }
    rule->Reset();
    rule->ApplyRule(frame, reference_line_info);
    ADEBUG << "Applied rule " << rule->Getname();
  }

  BuildPlanningTarget(reference_line_info);
  return Status::OK();
}

}  // namespace planning
}  // namespace apollo
