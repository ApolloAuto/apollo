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

#include "modules/planning/scenarios/scenario.h"

#include <map>
#include <utility>

namespace apollo {
namespace planning {
namespace scenario {

Scenario::Scenario(const ScenarioConfig& config, const ScenarioContext* context)
    : config_(config), scenario_context_(context) {
      name_ = ScenarioConfig::ScenarioType_Name(config.scenario_type());
    }

bool Scenario::LoadConfig(const std::string& config_file,
                          ScenarioConfig* config) {
  return apollo::common::util::GetProtoFromFile(config_file, config);
}

void Scenario::Init() {
  CHECK(!config_.stage_type().empty());
  for (const auto& stage_config : config_.stage_config()) {
    stage_config_map_[stage_config.stage_type()] = &stage_config;
  }
  for (int i = 0; i < config_.stage_type_size(); ++i) {
    auto stage_type = config_.stage_type(i);
    auto iter = stage_config_map_.find(stage_type);
    CHECK(iter != stage_config_map_.end())
        << "stage type : " << ScenarioConfig::StageType_Name(stage_type)
        << " has no config";
  }
  AINFO << "init stage "
        << ScenarioConfig::StageType_Name(config_.stage_type(0));
  current_stage_ = CreateStage(*stage_config_map_[config_.stage_type(0)]);
}

Scenario::ScenarioStatus Scenario::Process(
    const common::TrajectoryPoint& planning_init_point, Frame* frame) {
  if (current_stage_->stage_type() == ScenarioConfig::NO_STAGE) {
    return STATUS_DONE;
  }
  auto ret = current_stage_->Process(planning_init_point, frame);
  if (ret == Stage::ERROR) {
    AERROR << "Stage '" << current_stage_->Name() << "' returns error";
    return STATUS_UNKNOWN;
  } else if (ret == Stage::RUNNING) {
    return STATUS_PROCESSING;
  } else if (ret == Stage::FINISHED) {
    auto next_stage = current_stage_->NextStage();
    if (next_stage != current_stage_->stage_type()) {
      AINFO << "switch stage from " << current_stage_->Name() << "to "
            << ScenarioConfig::StageType_Name(next_stage);
      current_stage_ = CreateStage(*stage_config_map_[next_stage]);
    }
    if (current_stage_ != nullptr &&
        current_stage_->stage_type() != ScenarioConfig::NO_STAGE) {
      return STATUS_PROCESSING;
    } else {
      return STATUS_DONE;
    }
  } else {
    AWARN << "Unexpected Stage return value: " << ret;
    return STATUS_UNKNOWN;
  }
}

const std::string& Scenario::Name() const { return name_; }

}  // namespace scenario
}  // namespace planning
}  // namespace apollo
