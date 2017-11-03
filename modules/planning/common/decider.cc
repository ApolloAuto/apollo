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

#include <limits>

#include "modules/planning/common/decider.h"

#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/log.h"
#include "modules/common/util/util.h"
#include "modules/common/vehicle_state/vehicle_state.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/common/reference_line_info.h"

namespace apollo {
namespace planning {

void Decider::MakeDecision(const ReferenceLineInfo& reference_line_info,
                           DecisionResult* decision_result) {
  CHECK_NOTNULL(decision_result);
  decision_result->Clear();
  decision_result_ = decision_result;
  const auto& path_decision = reference_line_info.path_decision();

  // cruise by default
  decision_result_->mutable_main_decision()->mutable_cruise();

  // check stop decision
  int error_code = MakeMainStopDecision(reference_line_info);
  if (error_code < 0) {
    MakeEStopDecision(path_decision);
  }
  MakeMainMissionCompleteDecision(reference_line_info);

  SetObjectDecisions(path_decision);
}

void Decider::MakeMainMissionCompleteDecision(
    const ReferenceLineInfo& reference_line_info) {
  if (!decision_result_->main_decision().has_stop()) {
    return;
  }
  auto main_stop = decision_result_->main_decision().stop();
  if (main_stop.reason_code() != STOP_REASON_DESTINATION) {
    return;
  }
  const auto& adc_pos = reference_line_info.AdcPlanningPoint().path_point();
  if (common::util::DistanceXY(adc_pos, main_stop.stop_point()) >
      FLAGS_destination_check_distance) {
    return;
  }
  auto mission_complete =
      decision_result_->mutable_main_decision()->mutable_mission_complete();
  mission_complete->mutable_stop_point()->CopyFrom(main_stop.stop_point());
  mission_complete->set_stop_heading(main_stop.stop_heading());
}

int Decider::MakeMainStopDecision(
    const ReferenceLineInfo& reference_line_info) {
  double min_stop_line_s = std::numeric_limits<double>::infinity();
  const Obstacle* stop_obstacle = nullptr;
  const ObjectStop* stop_decision = nullptr;

  for (const auto path_obstacle :
       reference_line_info.path_decision().path_obstacles().Items()) {
    const auto& obstacle = path_obstacle->obstacle();
    const auto& object_decision = path_obstacle->LongitudinalDecision();
    if (!object_decision.has_stop()) {
      continue;
    }

    const auto& reference_line = reference_line_info.reference_line();
    apollo::common::PointENU stop_point = object_decision.stop().stop_point();
    common::SLPoint stop_line_sl;
    reference_line.XYToSL({stop_point.x(), stop_point.y()}, &stop_line_sl);

    double stop_line_s = stop_line_sl.s();
    if (stop_line_s < 0 || stop_line_s > reference_line.Length()) {
      AERROR << "Ignore object:" << obstacle->Id() << " fence route_s["
             << stop_line_s << "] not in range[0, " << reference_line.Length()
             << "]";
      continue;
    }

    // check stop_line_s vs adc_s
    common::SLPoint adc_sl;
    auto& adc_position = common::VehicleState::instance()->pose().position();
    reference_line.XYToSL({adc_position.x(), adc_position.y()}, &adc_sl);
    const auto& vehicle_param =
        common::VehicleConfigHelper::instance()->GetConfig().vehicle_param();
    if (stop_line_s <= adc_sl.s() + vehicle_param.front_edge_to_center()) {
      AERROR << "object:" << obstacle->Id() << " stop fence route_s["
             << stop_line_s << "] behind adc route_s[" << adc_sl.s() << "]";
      continue;
    }

    if (stop_line_s < min_stop_line_s) {
      min_stop_line_s = stop_line_s;
      stop_obstacle = obstacle;
      stop_decision = &(object_decision.stop());
    }
  }

  if (stop_obstacle != nullptr) {
    MainStop* main_stop =
        decision_result_->mutable_main_decision()->mutable_stop();
    main_stop->set_reason_code(stop_decision->reason_code());
    main_stop->set_reason("stop by " + stop_obstacle->Id());
    main_stop->mutable_stop_point()->set_x(stop_decision->stop_point().x());
    main_stop->mutable_stop_point()->set_y(stop_decision->stop_point().y());
    main_stop->set_stop_heading(stop_decision->stop_heading());

    ADEBUG << " main stop obstacle id:" << stop_obstacle->Id()
           << " stop_line_s:" << min_stop_line_s << " stop_point: ("
           << stop_decision->stop_point().x() << stop_decision->stop_point().y()
           << " ) stop_heading: " << stop_decision->stop_heading();

    return 1;
  }

  return 0;
}

void Decider::SetObjectDecisions(const PathDecision& path_decision) {
  ObjectDecisions* object_decisions =
      decision_result_->mutable_object_decision();

  for (const auto path_obstacle : path_decision.path_obstacles().Items()) {
    if (!path_obstacle->HasNonIgnoreDecision()) {
      continue;
    }
    auto* object_decision = object_decisions->add_decision();

    const auto& obstacle = path_obstacle->obstacle();
    object_decision->set_id(obstacle->Id());
    object_decision->set_perception_id(obstacle->PerceptionId());
    if (path_obstacle->HasLateralDecision() &&
        !path_obstacle->IsLateralIgnore()) {
      object_decision->add_object_decision()->CopyFrom(
          path_obstacle->LateralDecision());
    }
    if (path_obstacle->HasLongitudinalDecision() &&
        !path_obstacle->IsLongitudinalIgnore()) {
      object_decision->add_object_decision()->CopyFrom(
          path_obstacle->LongitudinalDecision());
    }
  }
}

void Decider::MakeEStopDecision(const PathDecision& path_decision) {
  decision_result_->Clear();

  MainEmergencyStop* main_estop =
      decision_result_->mutable_main_decision()->mutable_estop();
  main_estop->set_reason_code(MainEmergencyStop::ESTOP_REASON_INTERNAL_ERR);
  main_estop->set_reason("estop reason to be added");
  main_estop->mutable_cruise_to_stop();

  // set object decisions
  ObjectDecisions* object_decisions =
      decision_result_->mutable_object_decision();
  for (const auto path_obstacle : path_decision.path_obstacles().Items()) {
    auto* object_decision = object_decisions->add_decision();
    const auto& obstacle = path_obstacle->obstacle();
    object_decision->set_id(obstacle->Id());
    object_decision->set_perception_id(obstacle->PerceptionId());
    object_decision->add_object_decision()->mutable_avoid();
  }
}

}  // namespace planning
}  // namespace apollo
