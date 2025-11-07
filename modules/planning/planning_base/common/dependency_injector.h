/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
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

#pragma once

#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/planning/planning_base/common/ego_info.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/history.h"
#include "modules/planning/planning_base/common/learning_based_data.h"
#include "modules/planning/planning_base/common/planning_context.h"

namespace apollo {
namespace planning {

class DependencyInjector {
 public:
  DependencyInjector() = default;
  ~DependencyInjector() = default;
  // PlanningContext 存储了当前规划任务的上下文（如前一帧信息、紧急状态等）
  PlanningContext* planning_context() { return &planning_context_; }
  // FrameHistory 存储了过去的多个帧数据，用于规划模块进行时序分析
  FrameHistory* frame_history() { return &frame_history_; }
  // History 存储了全局历史信息，用于回溯过去的规划决策
  History* history() { return &history_; }
  // EgoInfo 存储了当前自动驾驶车辆的状态（如速度、位置、加速度等）
  EgoInfo* ego_info() { return &ego_info_; }
  // VehicleStateProvider 提供了车辆状态信息
  // 当前位置、速度、方向角、车辆运动学信息
  apollo::common::VehicleStateProvider* vehicle_state() {
    return &vehicle_state_;
  }
  // LearningBasedData 用于存储学习数据，供离线/在线机器学习模块使用，优化规划算法
  LearningBasedData* learning_based_data() { return &learning_based_data_; }

 private:
  PlanningContext planning_context_;
  FrameHistory frame_history_;
  History history_;
  EgoInfo ego_info_;
  apollo::common::VehicleStateProvider vehicle_state_;
  LearningBasedData learning_based_data_;
};

}  // namespace planning
}  // namespace apollo
