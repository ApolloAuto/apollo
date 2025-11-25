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
  // 1.设置 PlanningContext 中当前 Scenario 类型
  auto* scenario = injector_->planning_context()
                       ->mutable_planning_status()
                       ->mutable_scenario();
  scenario->Clear();
  scenario->set_scenario_type(name_);

  // 2. 获取当前 Scenario 类的名称
  // Generate the default config path.
  int status;
  // Get the name of this class.
  std::string class_name =
      abi::__cxa_demangle(typeid(*this).name(), 0, 0, &status);
  // 3. 找到该 Scenario 插件所在目录（配置目录）
  config_dir_ = apollo::cyber::plugin_manager::PluginManager::Instance()
                    ->GetPluginClassHomePath<Scenario>(class_name);
  config_dir_ += "/conf";
  AINFO << "config_dir : " << config_dir_;
  // Generate the default task config path from PluginManager.
  // 4. 获取 scenario_conf.pb.txt 的路径
  config_path_ = apollo::cyber::plugin_manager::PluginManager::Instance()
                     ->GetPluginConfPath<Scenario>(class_name,
                                                   "conf/scenario_conf.pb.txt");
  
  // 5.加载 pipeline（流水线）配置
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
  // 6. 将 pipeline 中的 stage 映射进 map
  for (const auto& stage : scenario_pipeline_config_.stage()) {
    stage_pipeline_map_[stage.name()] = &stage;
  }
  return true;
}
/// @brief 执行特定场景中各个阶段（stage）的核心函数，涉及到场景状态的管理以及阶段的处理和切换
/// @param planning_init_point 
/// @param frame 
/// @return 
ScenarioResult Scenario::Process(
    const common::TrajectoryPoint& planning_init_point, Frame* frame) {
// 如果当前阶段为空
  if (current_stage_ == nullptr) {
  //  从scenario_pipeline_config_ 中获取第一个阶段的配置信息,并通过 CreateStage 创建该阶段
    current_stage_ = CreateStage(
        *stage_pipeline_map_[scenario_pipeline_config_.stage(0).name()]);
  // 创建失败,打印错误信息并将阶段状态设置为 ERROR，然后返回场景结果
    if (nullptr == current_stage_) {
      AERROR << "Create stage " << scenario_pipeline_config_.stage(0).name()
             << " failed!";
      scenario_result_.SetStageResult(StageStatusType::ERROR);
      return scenario_result_;
    }
    AINFO << "Create stage " << current_stage_->Name();
  }
  // 如果当前阶段的名称为空，表示当前场景已经完成，不再需要处理，直接将场景的状态设置为 STATUS_DONE（已完成），然后返回场景结
  if (current_stage_->Name().empty()) {
    scenario_result_.SetScenarioStatus(ScenarioStatusType::STATUS_DONE);
    return scenario_result_;
  }
  auto ret = current_stage_->Process(planning_init_point, frame);  // lane_follow_stage.cc
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
    //  如果阶段返回 FINISHED，表示当前阶段已经完成，接下来要判断是否需要切换到下一个阶段
    case StageStatusType::FINISHED: {
      auto next_stage = current_stage_->NextStage();
    // 如果 next_stage 不为空，表示需要切换阶段
      if (next_stage != current_stage_->Name()) {
        AINFO << "switch stage from " << current_stage_->Name() << " to "
              << next_stage;
        if (next_stage.empty()) {
          scenario_result_.SetScenarioStatus(ScenarioStatusType::STATUS_DONE);
          return scenario_result_;
        }
        // 如果下一个阶段的配置找不到，打印错误并返回 STATUS_UNKNOWN
        if (stage_pipeline_map_.find(next_stage) == stage_pipeline_map_.end()) {
          AERROR << "Failed to find config for stage: " << next_stage;
          scenario_result_.SetScenarioStatus(
              ScenarioStatusType::STATUS_UNKNOWN);
          return scenario_result_;
        }
        // 如果下一个阶段的配置存在，尝试创建下一个阶段。如果创建失败，则打印警告并返回 STATUS_UNKNOWN
        current_stage_ = CreateStage(*stage_pipeline_map_[next_stage]);
        if (current_stage_ == nullptr) {
          AWARN << "Current stage is a null pointer.";
          scenario_result_.SetScenarioStatus(
              ScenarioStatusType::STATUS_UNKNOWN);
          return scenario_result_;
        }
      }
      // 如果当前阶段继续有效，设置场景状态为 STATUS_PROCESSING（处理中）
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
