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

#include "modules/control/control_component/controller_task_base/control_task_agent.h"

#include <utility>

#include "cyber/common/log.h"
#include "cyber/time/clock.h"
#include "modules/control/control_component/common/control_gflags.h"

namespace apollo {
namespace control {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::cyber::Clock;
using apollo::cyber::plugin_manager::PluginManager;

Status ControlTaskAgent::Init(std::shared_ptr<DependencyInjector> injector,
                              const ControlPipeline &control_pipeline) {
  if (control_pipeline.controller_size() == 0) {
    AERROR << "control_pipeline is empty";
    return Status(ErrorCode::CONTROL_INIT_ERROR, "Empty control_pipeline");
  }

  injector_ = injector;
  for (int i = 0; i < control_pipeline.controller_size(); i++) {
    auto controller = PluginManager::Instance()->CreateInstance<ControlTask>(
        "apollo::control::" + control_pipeline.controller(i).type());
    if (!controller->Init(injector_).ok()) {
      AERROR << "Can not init controller " << controller->Name();
      return Status(
          ErrorCode::CONTROL_INIT_ERROR,
          "Failed to init Controller:" + control_pipeline.controller(i).name());
    }
    controller_list_.push_back(controller);
    AINFO << "Controller <" << controller->Name() << "> init done!";
  }
  return Status::OK();
}

Status ControlTaskAgent::ComputeControlCommand(
    const localization::LocalizationEstimate *localization,
    const canbus::Chassis *chassis, const planning::ADCTrajectory *trajectory,
    control::ControlCommand *cmd) {
  // 遍历 controller_list_，每一个控制器（如 LQR、MPC 等）都将参与控制命令计算
  for (auto &controller : controller_list_) {
  // 打印当前正在处理的控制器的名称，帮助调试和性能分析
    ADEBUG << "controller:" << controller->Name() << " processing ...";
  // 获取当前系统时间（秒），用于统计该控制器的计算耗时
    double start_timestamp = Clock::NowInSeconds();
    controller->ComputeControlCommand(localization, chassis, trajectory, cmd);
  // 获取控制器计算后的时间，用于计算耗时
    double end_timestamp = Clock::NowInSeconds();
    const double time_diff_ms = (end_timestamp - start_timestamp) * 1000;

    ADEBUG << "controller: " << controller->Name()
           << " calculation time is: " << time_diff_ms << " ms.";
    cmd->mutable_latency_stats()->add_controller_time_ms(time_diff_ms);
  }
  return Status::OK();
}

Status ControlTaskAgent::Reset() {
  for (auto &controller : controller_list_) {
    ADEBUG << "controller:" << controller->Name() << " reset...";
    controller->Reset();
  }
  return Status::OK();
}

}  // namespace control
}  // namespace apollo
