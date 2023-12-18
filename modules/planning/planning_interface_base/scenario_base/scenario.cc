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

#include "modules/planning/planning_interface_base/scenario_base/scenario.h"

#include <cxxabi.h>

#include "cyber/class_loader/class_loader_manager.h"
#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/util/config_util.h"
#include "modules/planning/planning_interface_base/scenario_base/stage.h"

namespace apollo {
namespace planning {

Scenario::Scenario()
    : scenario_result_(ScenarioResult(ScenarioStatusType::STATUS_UNKNOWN)),
      current_stage_(nullptr),
      msg_(""),
      injector_(nullptr),
      config_path_(""),
      config_dir_(""),
      name_("") {}

bool Scenario::Init(std::shared_ptr<DependencyInjector> injector,
                    const std::string& name) {
  name_ = name;
  injector_ = injector;
  // set scenario_type in PlanningContext
  auto* scenario = injector_->planning_context()
                       ->mutable_planning_status()
                       ->mutable_scenario();
  scenario->Clear();
  scenario->set_scenario_type(name_);

  // Generate the default config path.
  int status;
  // Get the name of this class.
  std::string class_name =
      abi::__cxa_demangle(typeid(*this).name(), 0, 0, &status);

  config_dir_ = apollo::cyber::plugin_manager::PluginManager::Instance()
                    ->GetPluginClassHomePath<Scenario>(class_name);
  config_dir_ += "/conf";
  AINFO << "config_dir : " << config_dir_;
  // Generate the default task config path from PluginManager.
  config_path_ = apollo::cyber::plugin_manager::PluginManager::Instance()
                     ->GetPluginConfPath<Scenario>(class_name,
                                                   "conf/scenario_conf.pb.txt");

  // Load the pipeline config.
  std::string pipeline_config_path =
      apollo::cyber::plugin_manager::PluginManager::Instance()
          ->GetPluginConfPath<Scenario>(class_name, "conf/pipeline.pb.txt");
  AINFO << "Load config path:" << pipeline_config_path;
  // Load the pipeline of scenario.
  if (!apollo::cyber::common::GetProtoFromFile(pipeline_config_path,
                                               &scenario_pipeline_config_)) {
    AERROR << "Load pipeline of " << name_ << " failed!";
    return false;
  }
  for (const auto& stage : scenario_pipeline_config_.stage()) {
    stage_pipeline_map_[stage.name()] = &stage;
  }
  return true;
}

ScenarioResult Scenario::Process(
    const common::TrajectoryPoint& planning_init_point, Frame* frame) {
  if (current_stage_ == nullptr) {
    current_stage_ = CreateStage(
        *stage_pipeline_map_[scenario_pipeline_config_.stage(0).name()]);
    if (nullptr == current_stage_) {
      AERROR << "Create stage " << scenario_pipeline_config_.stage(0).name()
             << " failed!";
      scenario_result_.SetStageResult(StageStatusType::ERROR);
      return scenario_result_;
    }
    AINFO << "Create stage " << current_stage_->Name();
  }
  if (current_stage_->Name().empty()) {
    scenario_result_.SetScenarioStatus(ScenarioStatusType::STATUS_DONE);
    return scenario_result_;
  }
  auto ret = current_stage_->Process(planning_init_point, frame);
  scenario_result_.SetStageResult(ret);
  switch (ret.GetStageStatus()) {
    case StageStatusType::ERROR: {
      AERROR << "Stage '" << current_stage_->Name() << "' returns error";
      scenario_result_.SetScenarioStatus(ScenarioStatusType::STATUS_UNKNOWN);
      break;
    }
    case StageStatusType::RUNNING: {
      scenario_result_.SetScenarioStatus(ScenarioStatusType::STATUS_PROCESSING);
      break;
    }
    case StageStatusType::FINISHED: {
      auto next_stage = current_stage_->NextStage();
      if (next_stage != current_stage_->Name()) {
        AINFO << "switch stage from " << current_stage_->Name() << " to "
              << next_stage;
        if (next_stage.empty()) {
          scenario_result_.SetScenarioStatus(ScenarioStatusType::STATUS_DONE);
          return scenario_result_;
        }
        if (stage_pipeline_map_.find(next_stage) == stage_pipeline_map_.end()) {
          AERROR << "Failed to find config for stage: " << next_stage;
          scenario_result_.SetScenarioStatus(
              ScenarioStatusType::STATUS_UNKNOWN);
          return scenario_result_;
        }
        current_stage_ = CreateStage(*stage_pipeline_map_[next_stage]);
        if (current_stage_ == nullptr) {
          AWARN << "Current stage is a null pointer.";
          scenario_result_.SetScenarioStatus(
              ScenarioStatusType::STATUS_UNKNOWN);
          return scenario_result_;
        }
      }
      if (current_stage_ != nullptr && !current_stage_->Name().empty()) {
        scenario_result_.SetScenarioStatus(
            ScenarioStatusType::STATUS_PROCESSING);
      } else {
        scenario_result_.SetScenarioStatus(ScenarioStatusType::STATUS_DONE);
      }
      break;
    }
    default: {
      AWARN << "Unexpected Stage return value: "
            << static_cast<int>(ret.GetStageStatus());
      scenario_result_.SetScenarioStatus(ScenarioStatusType::STATUS_UNKNOWN);
    }
  }
  return scenario_result_;
}

std::shared_ptr<Stage> Scenario::CreateStage(
    const StagePipeline& stage_pipeline) {
  auto stage_ptr =
      apollo::cyber::plugin_manager::PluginManager::Instance()
          ->CreateInstance<Stage>(
              ConfigUtil::GetFullPlanningClassName(stage_pipeline.type()));
  if (nullptr == stage_ptr ||
      !stage_ptr->Init(stage_pipeline, injector_, config_dir_, GetContext())) {
    AERROR << "Create stage " << stage_pipeline.name() << " of " << name_
           << " failed!";
    return nullptr;
  }
  return stage_ptr;
}

const std::string Scenario::GetStage() const {
  return current_stage_ ? current_stage_->Name() : "";
}

void Scenario::Reset() {
  scenario_result_ = ScenarioResult(ScenarioStatusType::STATUS_UNKNOWN);
  current_stage_ = nullptr;
}

}  // namespace planning
}  // namespace apollo
