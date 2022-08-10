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

#include "modules/planning/scenarios/stop_sign/unprotected/stop_sign_unprotected_scenario.h"

#include "cyber/common/log.h"
#include "modules/common_msgs/perception_msgs/perception_obstacle.pb.h"
#include "modules/planning/common/frame.h"
#include "modules/planning/common/planning_context.h"
#include "modules/planning/common/planning_gflags.h"
#include "modules/planning/proto/planning_config.pb.h"
#include "modules/planning/scenarios/stop_sign/unprotected/stage_creep.h"
#include "modules/planning/scenarios/stop_sign/unprotected/stage_intersection_cruise.h"
#include "modules/planning/scenarios/stop_sign/unprotected/stage_pre_stop.h"
#include "modules/planning/scenarios/stop_sign/unprotected/stage_stop.h"

namespace apollo {
namespace planning {
namespace scenario {
namespace stop_sign {

using apollo::hdmap::HDMapUtil;
using apollo::hdmap::StopSignInfo;
using apollo::hdmap::StopSignInfoConstPtr;

using StopSignLaneVehicles =
    std::unordered_map<std::string, std::vector<std::string>>;

apollo::common::util::Factory<
    StageType, Stage,
    Stage* (*)(const ScenarioConfig::StageConfig& stage_config,
               const std::shared_ptr<DependencyInjector>& injector)>
    StopSignUnprotectedScenario::s_stage_factory_;

void StopSignUnprotectedScenario::Init() {
  if (init_) {
    return;
  }

  Scenario::Init();

  if (!GetScenarioConfig()) {
    AERROR << "fail to get scenario specific config";
    return;
  }

  const std::string stop_sign_overlap_id = injector_->planning_context()
                                               ->planning_status()
                                               .stop_sign()
                                               .current_stop_sign_overlap_id();
  if (stop_sign_overlap_id.empty()) {
    AERROR << "Could not find stop sign";
    return;
  }
  hdmap::StopSignInfoConstPtr stop_sign = HDMapUtil::BaseMap().GetStopSignById(
      hdmap::MakeMapId(stop_sign_overlap_id));
  if (!stop_sign) {
    AERROR << "Could not find stop sign: " << stop_sign_overlap_id;
    return;
  }

  context_.current_stop_sign_overlap_id = stop_sign_overlap_id;
  context_.watch_vehicles.clear();

  GetAssociatedLanes(*stop_sign);

  init_ = true;
}

void StopSignUnprotectedScenario::RegisterStages() {
  if (!s_stage_factory_.Empty()) {
    s_stage_factory_.Clear();
  }
  s_stage_factory_.Register(
      StageType::STOP_SIGN_UNPROTECTED_PRE_STOP,
      [](const ScenarioConfig::StageConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Stage* {
        return new StopSignUnprotectedStagePreStop(config, injector);
      });
  s_stage_factory_.Register(
      StageType::STOP_SIGN_UNPROTECTED_STOP,
      [](const ScenarioConfig::StageConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Stage* {
        return new StopSignUnprotectedStageStop(config, injector);
      });
  s_stage_factory_.Register(
      StageType::STOP_SIGN_UNPROTECTED_CREEP,
      [](const ScenarioConfig::StageConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Stage* {
        return new StopSignUnprotectedStageCreep(config, injector);
      });
  s_stage_factory_.Register(
      StageType::STOP_SIGN_UNPROTECTED_INTERSECTION_CRUISE,
      [](const ScenarioConfig::StageConfig& config,
         const std::shared_ptr<DependencyInjector>& injector) -> Stage* {
        return new StopSignUnprotectedStageIntersectionCruise(config, injector);
      });
}

std::unique_ptr<Stage> StopSignUnprotectedScenario::CreateStage(
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
bool StopSignUnprotectedScenario::GetScenarioConfig() {
  if (!config_.has_stop_sign_unprotected_config()) {
    AERROR << "miss scenario specific config";
    return false;
  }
  context_.scenario_config.CopyFrom(config_.stop_sign_unprotected_config());
  return true;
}

/*
 * get all the lanes associated/guarded by a stop sign
 */
int StopSignUnprotectedScenario::GetAssociatedLanes(
    const StopSignInfo& stop_sign_info) {
  context_.associated_lanes.clear();

  std::vector<StopSignInfoConstPtr> associated_stop_signs;
  HDMapUtil::BaseMap().GetStopSignAssociatedStopSigns(stop_sign_info.id(),
                                                      &associated_stop_signs);

  for (const auto stop_sign : associated_stop_signs) {
    if (stop_sign == nullptr) {
      continue;
    }

    const auto& associated_lane_ids = stop_sign->OverlapLaneIds();
    for (const auto& lane_id : associated_lane_ids) {
      const auto lane = HDMapUtil::BaseMap().GetLaneById(lane_id);
      if (lane == nullptr) {
        continue;
      }
      for (const auto& stop_sign_overlap : lane->stop_signs()) {
        auto over_lap_info =
            stop_sign_overlap->GetObjectOverlapInfo(stop_sign.get()->id());
        if (over_lap_info != nullptr) {
          context_.associated_lanes.push_back(
              std::make_pair(lane, stop_sign_overlap));
          ADEBUG << "stop_sign: " << stop_sign_info.id().id()
                 << "; associated_lane: " << lane_id.id()
                 << "; associated_stop_sign: " << stop_sign.get()->id().id();
        }
      }
    }
  }

  return 0;
}

}  // namespace stop_sign
}  // namespace scenario
}  // namespace planning
}  // namespace apollo
