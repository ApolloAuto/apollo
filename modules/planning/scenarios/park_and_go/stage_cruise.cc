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

#include "modules/planning/scenarios/park_and_go/stage_cruise.h"

#include "cyber/common/log.h"

#include "modules/common/vehicle_state/vehicle_state_provider.h"

#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/util/common.h"
#include "modules/planning/scenarios/util/util.h"
#include "modules/planning/tasks/deciders/path_bounds_decider/path_bounds_decider.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace park_and_go {

using apollo::common::TrajectoryPoint;

Stage::StageStatus ParkAndGoStageCruise::Process(
    const TrajectoryPoint& planning_init_point, Frame* frame) {
  ADEBUG << "stage: Cruise";
  CHECK_NOTNULL(frame);

  scenario_config_.CopyFrom(GetContext()->scenario_config);

  bool plan_ok = ExecuteTaskOnReferenceLine(planning_init_point, frame);
  if (!plan_ok) {
    AERROR << "ParkAndGoStageCruise planning error";
  }

  const ReferenceLineInfo& reference_line_info =
      frame->reference_line_info().front();
  // check ADC status:
  // 1. At routing beginning: stage finished
  ParkAndGoStatus status =
      CheckADCParkAndGoCruiseCompleted(reference_line_info);

  if (status == CRUISE_COMPLETE) {
    return FinishStage();
  }
  return Stage::RUNNING;
}

Stage::StageStatus ParkAndGoStageCruise::FinishStage() {
  return FinishScenario();
}

ParkAndGoStageCruise::ParkAndGoStatus
ParkAndGoStageCruise::CheckADCParkAndGoCruiseCompleted(
    const ReferenceLineInfo& reference_line_info) {
  const auto& reference_line = reference_line_info.reference_line();

  // check l delta
  const common::math::Vec2d adc_position = {
      common::VehicleStateProvider::Instance()->x(),
      common::VehicleStateProvider::Instance()->y()};
  common::SLPoint adc_position_sl;
  reference_line.XYToSL(adc_position, &adc_position_sl);

  const double kLBuffer = 0.5;
  if (std::fabs(adc_position_sl.l()) < kLBuffer) {
    ADEBUG << "cruise completed";
    return CRUISE_COMPLETE;
  }

  /* loose heading check, so that ADC can enter LANE_FOLLOW scenario sooner
   * which is more sophisticated
  // heading delta
  const double adc_heading =
      common::VehicleStateProvider::Instance()->heading();
  const auto reference_point =
      reference_line.GetReferencePoint(adc_position_sl.s());
  const auto path_point = reference_point.ToPathPoint(adc_position_sl.s());
  ADEBUG << "adc_position_sl.l():[" << adc_position_sl.l() << "]";
  ADEBUG << "adc_heading - path_point.theta():[" << adc_heading << "]"
         << "[" << path_point.theta() << "]";
  const double kHeadingBuffer = 0.1;
  if (std::fabs(adc_heading - path_point.theta()) < kHeadingBuffer) {
    ADEBUG << "cruise completed";
    return CRUISE_COMPLETE;
  }
  */

  return CRUISING;
}

}  // namespace park_and_go
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
