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

#include "modules/planning/scenarios/bare_intersection/unprotected/bare_intersection_unprotected_scenario.h"

#include "modules/planning/proto/planning_config.pb.h"

#include "cyber/common/log.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/scenarios/bare_intersection/unprotected/stage_approach.h"
#include "modules/planning/scenarios/bare_intersection/unprotected/stage_intersection_cruise.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace bare_intersection {

using apollo::hdmap::HDMapUtil;

void BareIntersectionUnprotectedScenario::Init() {
  if (init_) {
    return;
  }

  Scenario::Init();

  if (!GetScenarioConfig()) {
    AERROR << "fail to get scenario specific config";
    return;
  }

  const std::string& pnc_junction_overlap_id =
      PlanningContext::Instance()
          ->planning_status()
          .bare_intersection()
          .current_pnc_junction_overlap_id();
  if (pnc_junction_overlap_id.empty()) {
    AERROR << "Could not find pnc_junction";
    return;
  }
  hdmap::PNCJunctionInfoConstPtr pnc_junction =
      HDMapUtil::BaseMap().GetPNCJunctionById(
          hdmap::MakeMapId(pnc_junction_overlap_id));
  if (!pnc_junction) {
    AERROR << "Could not find pnc_junction: " << pnc_junction_overlap_id;
    return;
  }

  context_.current_pnc_junction_overlap_id = pnc_junction_overlap_id;

  init_ = true;
}

apollo::common::util::Factory<
    ScenarioConfig::StageType, Stage,
    Stage* (*)(const ScenarioConfig::StageConfig& stage_config)>
    BareIntersectionUnprotectedScenario::s_stage_factory_;

void BareIntersectionUnprotectedScenario::RegisterStages() {
  if (!s_stage_factory_.Empty()) {
    s_stage_factory_.Clear();
  }
  s_stage_factory_.Register(
      ScenarioConfig::BARE_INTERSECTION_UNPROTECTED_APPROACH,
      [](const ScenarioConfig::StageConfig& config) -> Stage* {
        return new BareIntersectionUnprotectedStageApproach(config);
      });
  s_stage_factory_.Register(
      ScenarioConfig::BARE_INTERSECTION_UNPROTECTED_INTERSECTION_CRUISE,
      [](const ScenarioConfig::StageConfig& config) -> Stage* {
        return new BareIntersectionUnprotectedStageIntersectionCruise(config);
      });
}

std::unique_ptr<Stage> BareIntersectionUnprotectedScenario::CreateStage(
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

/*
 * read scenario specific configs and set in context_ for stages to read
 */
bool BareIntersectionUnprotectedScenario::GetScenarioConfig() {
  if (!config_.has_bare_intersection_unprotected_config()) {
    AERROR << "miss scenario specific config";
    return false;
  }
  context_.scenario_config = config_.bare_intersection_unprotected_config();
  return true;
}

}  // namespace bare_intersection
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
