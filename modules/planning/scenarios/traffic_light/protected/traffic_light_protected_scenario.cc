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

#include "modules/planning/scenarios/traffic_light/protected/traffic_light_protected_scenario.h"

#include "cyber/common/log.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/scenarios/traffic_light/protected/stage_approach.h"
#include "modules/planning/scenarios/traffic_light/protected/stage_intersection_cruise.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace traffic_light {

using apollo::hdmap::HDMapUtil;

void TrafficLightProtectedScenario::Init() {
  if (init_) {
    return;
  }

  Scenario::Init();

  if (!GetScenarioConfig()) {
    AERROR << "fail to get scenario specific config";
    return;
  }

  const auto& traffic_light_status =
      injector_->planning_context()->planning_status().traffic_light();

  if (traffic_light_status.current_traffic_light_overlap_id().empty()) {
    AERROR << "Could not find traffic-light(s)";
    return;
  }

  context_.current_traffic_light_overlap_ids.clear();
  for (int i = 0;
       i < traffic_light_status.current_traffic_light_overlap_id_size(); i++) {
    const std::string traffic_light_overlap_id =
        traffic_light_status.current_traffic_light_overlap_id(i);
    hdmap::SignalInfoConstPtr traffic_light =
        HDMapUtil::BaseMap().GetSignalById(
            hdmap::MakeMapId(traffic_light_overlap_id));
    if (!traffic_light) {
      AERROR << "Could not find traffic light: " << traffic_light_overlap_id;
    }

    context_.current_traffic_light_overlap_ids.push_back(
        traffic_light_overlap_id);
  }

  init_ = true;
}

apollo::common::util::Factory<
    StageType, Stage,
    Stage* (*)(const ScenarioConfig::StageConfig& stage_config,
               const std::shared_ptr<DependencyInjector>& injector)>
    TrafficLightProtectedScenario::s_stage_factory_;

void TrafficLightProtectedScenario::RegisterStages() {
  if (!s_stage_factory_.Empty()) {
    s_stage_factory_.Clear();
  }
  s_stage_factory_.Register(
      StageType::TRAFFIC_LIGHT_PROTECTED_APPROACH,
      [](const ScenarioConfig::StageConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Stage* {
        return new TrafficLightProtectedStageApproach(config, injector);
      });
  s_stage_factory_.Register(
      StageType::TRAFFIC_LIGHT_PROTECTED_INTERSECTION_CRUISE,
      [](const ScenarioConfig::StageConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Stage* {
        return new TrafficLightProtectedStageIntersectionCruise(config,
                                                                injector);
      });
}

std::unique_ptr<Stage> TrafficLightProtectedScenario::CreateStage(
    const ScenarioConfig::StageConfig& stage_config,
    const std::shared_ptr<DependencyInjector>& injector) {
  if (s_stage_factory_.Empty()) {
    RegisterStages();
  }
  auto ptr = s_stage_factory_.CreateObjectOrNull(stage_config.stage_type(),
                                                 stage_config, injector);
  if (ptr) {
    ptr->SetContext(&context_);
  }
  return ptr;
}

/*
 * read scenario specific configs and set in context_ for stages to read
 */
bool TrafficLightProtectedScenario::GetScenarioConfig() {
  if (!config_.has_traffic_light_protected_config()) {
    AERROR << "miss scenario specific config";
    return false;
  }
  context_.scenario_config.CopyFrom(config_.traffic_light_protected_config());
  return true;
}

}  // namespace traffic_light
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
