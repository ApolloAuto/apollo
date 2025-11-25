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

#include "modules/planning/planners/public_road/scenario_manager.h"

#include <algorithm>
#include <string>
#include <vector>

#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/common/status/status.h"
#include "modules/planning/planning_base/common/util/config_util.h"
#include "modules/planning/planning_interface_base/scenario_base/scenario.h"

namespace apollo {
namespace planning {

using apollo::cyber::plugin_manager::PluginManager;
/// @brief 场景注册
/// @param injector 
/// @param planner_config 
/// @return 
bool ScenarioManager::Init(const std::shared_ptr<DependencyInjector>& injector,
                           const PlannerPublicRoadConfig& planner_config) {
  if (init_) {
    return true;
  }
  // 保存依赖注入器，用于后面各个场景的初始化
  injector_ = injector;
  // 加载并创建所有 Scenario
  for (int i = 0; i < planner_config.scenario_size(); i++) {
    // 根据配置中的类型字符串（如 "LAYER_FOLLOW", "PARK_AND_GO" 等），通过 插件系统动态创建对应的 Scenario 对象
    auto scenario = PluginManager::Instance()->CreateInstance<Scenario>(
        ConfigUtil::GetFullPlanningClassName(
            planner_config.scenario(i).type()));
    // 用依赖注入器和场景名称初始化该 Scenario
    ACHECK(scenario->Init(injector_, planner_config.scenario(i).name()))
        << "Can not init scenario" << planner_config.scenario(i).name();
    // 把创建好的场景加入场景列表
    scenario_list_.push_back(scenario);
    if (planner_config.scenario(i).name() == "LANE_FOLLOW") {
      default_scenario_type_ = scenario;
    }
  }
  AINFO << "Load scenario list:" << planner_config.DebugString();
  // 默认为LANE_FOLLOW
  current_scenario_ = default_scenario_type_;
  init_ = true;
  return true;
}
/// @brief 更新场景状态
/// @param ego_point 自车规划的起始点
/// @param frame 
void ScenarioManager::Update(const common::TrajectoryPoint& ego_point,
                             Frame* frame) {
  // 如果frame为空指针，程序将触发断言失败，终止执行
  CHECK_NOTNULL(frame);
  for (auto scenario : scenario_list_) {
    // current_scenario_是一个智能指针，get()方法返回原始指针
    if (current_scenario_.get() == scenario.get() &&
        current_scenario_->GetStatus() ==
            ScenarioStatusType::STATUS_PROCESSING) {
      // The previous scenario has higher priority
      // 如果当前场景与遍历到的场景相同，并且当前场景正在处理中，则返回，意味着不需要切换场景
      return;
    }
    // 如果当前场景不是在正在处理状态，接着判断场景是否可以切换
    if (scenario->IsTransferable(current_scenario_.get(), *frame)) {
      // 如果场景切换可行，退出当前场景
      current_scenario_->Exit(frame);
      AINFO << "switch scenario from" << current_scenario_->Name() << " to "
            << scenario->Name();
      current_scenario_ = scenario;
      current_scenario_->Reset();
      current_scenario_->Enter(frame);
      return;
    }
  }
}

void ScenarioManager::Reset(Frame* frame) {
  if (current_scenario_) {
    current_scenario_->Exit(frame);
  }
  AINFO << "Reset to default scenario:" << default_scenario_type_->Name();
  current_scenario_ = default_scenario_type_;
}
}  // namespace planning
}  // namespace apollo
