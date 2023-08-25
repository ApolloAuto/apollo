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

#include "modules/control/controllers/demo_control_task/demo_control_task.h"

#include <algorithm>
#include <iomanip>
#include <utility>

namespace apollo {
namespace control {

using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::TrajectoryPoint;
using apollo::common::VehicleStateProvider;

DemoControlTask::DemoControlTask() : name_("demo control task") {
  AINFO << "Using " << name_;
}

DemoControlTask::~DemoControlTask() {}

Status DemoControlTask::Init(std::shared_ptr<DependencyInjector> injector) {
  if (!ControlTask::LoadConfig<DemoControlTaskConf>(&demo_control_task_conf_)) {
    AERROR << "failed to load control conf";
    return Status(ErrorCode::CONTROL_INIT_ERROR,
                  "failed to load lat control_conf");
  }

  injector_ = injector;
  low_bound_acceleration_ = demo_control_task_conf_.bound_acc();
  return Status::OK();
}

void DemoControlTask::Stop() {}

Status DemoControlTask::Reset() { return Status::OK(); }

std::string DemoControlTask::Name() const { return name_; }

Status DemoControlTask::ComputeControlCommand(
    const localization::LocalizationEstimate *localization,
    const canbus::Chassis *chassis,
    const planning::ADCTrajectory *planning_published_trajectory,
    ControlCommand *cmd) {
  auto debug = cmd->mutable_debug()->mutable_simple_lon_debug();

  double controller_calculate_acceleration = cmd->acceleration();
  double command_acceleration =
      abs(controller_calculate_acceleration) < low_bound_acceleration_
          ? controller_calculate_acceleration /
                abs(controller_calculate_acceleration) * low_bound_acceleration_
          : controller_calculate_acceleration;

  cmd->set_acceleration(command_acceleration);
  debug->set_acceleration_cmd(command_acceleration);

  return Status::OK();
}

}  // namespace control
}  // namespace apollo
