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

/**
 * @file
 **/
#include "modules/planning/scenarios/valet_parking/valet_parking_scenario.h"
#include "modules/planning/scenarios/valet_parking/stage_parking.h"
#include "modules/planning/scenarios/valet_parking/stage_approaching_parking_spot.h"



namespace apollo {
namespace planning {
namespace scenario {
namespace valet_parking {

apollo::common::util::Factory<
      ScenarioConfig::StageType, Stage,
      Stage* (*)(const ScenarioConfig::StageConfig& stage_config)>
      ValetParkingScenario::s_stage_factory_;

void ValetParkingScenario::Init() {
  if (init_) {
    return;
  }

  Scenario::Init();

  if (!GetScenarioConfig()) {
    AERROR << "fail to get scenario specific config";
    return;
  }
}

void ValetParkingScenario::RegisterStages() {
  if (s_stage_factory_.Empty()) {
    s_stage_factory_.Clear();
  }
  s_stage_factory_.Register(
      ScenarioConfig::VALET_PARKING_APPROACHING_PARKING_SPOT,
      [](const ScenarioConfig::StageConfig& config) -> Stage* {
        return new StageApproachingParkingSpot(config);
      });
  s_stage_factory_.Register(
      ScenarioConfig::VALET_PARKING_PARKING,
      [](const ScenarioConfig::StageConfig& config) -> Stage* {
        return new StageParking(config);
      });
}

std::unique_ptr<Stage> ValetParkingScenario::CreateStage(
    const ScenarioConfig::StageConfig& stage_config) {
  if (s_stage_factory_.Empty()) {
    RegisterStages();
  }
  auto ptr = s_stage_factory_.CreateObjectOrNull(stage_config.stage_type(),
                                                 stage_config);
  if (ptr) {
    ptr->SetContext(&context_);
  }
  return ptr;
}

bool ValetParkingScenario::GetScenarioConfig() {
  if (!config_.has_valet_parking_config()) {
    AERROR << "miss scenario specific config";
    return false;
  }
  context_.scenario_config.CopyFrom(config_.valet_parking_config());
  return true;
}

bool ValetParkingScenario::IsTransferable(const Scenario& current_scenario,
                                          const Frame& frame) {
  return true;
}

}  // namespace valet_parking
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
