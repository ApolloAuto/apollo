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

Status PublicRoadPlanner::Init(
    const std::shared_ptr<DependencyInjector>& injector,
    const std::string& config_path) {
  // 根据配置注册不同的场景
  // 支持的场景 modules/planning/planners/public_road/conf/planner_config.pb.txt
  Planner::Init(injector, config_path);
  LoadConfig<PlannerPublicRoadConfig>(config_path, &config_);
  scenario_manager_.Init(injector, config_);
  return Status::OK();
}

// 公路场景规划器的Plan函数，用于生成规划轨迹
Status PublicRoadPlanner::Plan(const TrajectoryPoint& planning_start_point,
                               Frame* frame,
                               ADCTrajectory* ptr_computed_trajectory) {
  // 更新场景管理器 frame包含场景指令信息
  scenario_manager_.Update(planning_start_point, frame);

  // 获取场景
  scenario_ = scenario_manager_.mutable_scenario();
  if (!scenario_) {
    // 如果场景为空，返回错误
    return Status(apollo::common::ErrorCode::PLANNING_ERROR,
                  "Unknown Scenario");
  }

  // 处理场景并获取结果
  auto result = scenario_->Process(planning_start_point, frame);

  // 如果启用了调试记录，则记录场景调试信息
  if (FLAGS_enable_record_debug) {
    auto scenario_debug = ptr_computed_trajectory->mutable_debug()
                              ->mutable_planning_data()
                              ->mutable_scenario();
    scenario_debug->set_scenario_plugin_type(scenario_->Name());
    scenario_debug->set_stage_plugin_type(scenario_->GetStage());
    scenario_debug->set_msg(scenario_->GetMsg());
  }

  // 如果场景的状态为STATUS_DONE，则仅在前一个场景的状态为STATUS_DONE时更新场景管理器
  if (result.GetScenarioStatus() == ScenarioStatusType::STATUS_DONE) {
    // only updates scenario manager when previous scenario's status is
    // STATUS_DONE
    scenario_manager_.Update(planning_start_point, frame);
  } else if (result.GetScenarioStatus() == ScenarioStatusType::STATUS_UNKNOWN) {
    // 如果场景的状态为STATUS_UNKNOWN，返回错误
    return Status(common::PLANNING_ERROR,
                  result.GetTaskStatus().error_message());
  }

  // 返回处理结果的状态
  return Status(common::OK, result.GetTaskStatus().error_message());
}

}  // namespace planning
}  // namespace apollo
