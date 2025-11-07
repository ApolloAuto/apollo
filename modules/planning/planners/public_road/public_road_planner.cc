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

#include "modules/planning/planners/public_road/public_road_planner.h"

#include "modules/planning/planning_base/gflags/planning_gflags.h"
#include "modules/planning/planning_interface_base/scenario_base/scenario.h"

namespace apollo {
namespace planning {

using apollo::common::Status;
using apollo::common::TrajectoryPoint;
/// @brief 实例化一个全局的scenario_manager_对象来进行场景管理
/// @param injector 
/// @param config_path 
/// @return 
// 初始化 PublicRoadPlanner 类的对象，包括初始化父类、加载配置文件、初始化场景管理器，并最终返回一个成功状态
Status PublicRoadPlanner::Init(
    const std::shared_ptr<DependencyInjector>& injector,
    const std::string& config_path) {
  // 先调用父类的初始化操作
  Planner::Init(injector, config_path);
  // 加载配置文件，并将其内容存储到 config_ 成员变量中
  LoadConfig<PlannerPublicRoadConfig>(config_path, &config_);  // config_path： public_road_planner_config.pb.txt
  scenario_manager_.Init(injector, config_);
  return Status::OK();
}

Status PublicRoadPlanner::Plan(const TrajectoryPoint& planning_start_point,
                               Frame* frame,
                               ADCTrajectory* ptr_computed_trajectory) {
   // 更新场景，决策当前应该执行什么场景                              
  scenario_manager_.Update(planning_start_point, frame);
  // 获取当前场景
  scenario_ = scenario_manager_.mutable_scenario();
  if (!scenario_) {
    return Status(apollo::common::ErrorCode::PLANNING_ERROR,
                  "Unknown Scenario");
  }
  // 调用Scenario的Process函数，对具体的场景进行处理
  auto result = scenario_->Process(planning_start_point, frame);

  if (FLAGS_enable_record_debug) {
    auto scenario_debug = ptr_computed_trajectory->mutable_debug()
                              ->mutable_planning_data()
                              ->mutable_scenario();
    scenario_debug->set_scenario_plugin_type(scenario_->Name());
    scenario_debug->set_stage_plugin_type(scenario_->GetStage());
    scenario_debug->set_msg(scenario_->GetMsg());
  }
 // 当前场景完成
  if (result.GetScenarioStatus() == ScenarioStatusType::STATUS_DONE) {
    // only updates scenario manager when previous scenario's status is
    // STATUS_DONE
    scenario_manager_.Update(planning_start_point, frame);
  } else if (result.GetScenarioStatus() == ScenarioStatusType::STATUS_UNKNOWN) {
    // 当前场景失败
    return Status(common::PLANNING_ERROR,
                  result.GetTaskStatus().error_message());
  }
  return Status(common::OK, result.GetTaskStatus().error_message());
}

}  // namespace planning
}  // namespace apollo
