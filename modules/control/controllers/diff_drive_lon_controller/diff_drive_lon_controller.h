// Copyright 2024 daohu527@gmail.com
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//  Created Date: 2024-12-30
//  Author: daohu527

#pragma once

#include <memory>

#include "modules/common_msgs/config_msgs/vehicle_config.pb.h"
#include "modules/control/controllers/lon_based_pid_controller/proto/lon_based_pid_controller_conf.pb.h"

#include "modules/common/math/math_utils.h"
#include "modules/control/control_component/controller_task_base/common/leadlag_controller.h"
#include "modules/control/control_component/controller_task_base/common/pid_controller.h"
#include "modules/control/control_component/controller_task_base/common/trajectory_analyzer.h"
#include "modules/control/control_component/controller_task_base/control_task.h"

namespace apollo {
namespace control {

class DiffDriveLonController : public ControlTask {
 public:
  DiffDriveLonController();
  virtual ~DiffDriveLonController();

  common::Status Init(std::shared_ptr<DependencyInjector> injector) override;

  common::Status ComputeControlCommand(
      const localization::LocalizationEstimate *localization,
      const canbus::Chassis *chassis, const planning::ADCTrajectory *trajectory,
      control::ControlCommand *cmd) override;

  common::Status Reset() override;

  std::string Name() const override;

  void Stop() override;

 private:
  void CloseLogFile();
  void ComputeLongitudinalErrors(const TrajectoryAnalyzer *trajectory_analyzer,
                                 const double preview_time, const double ts,
                                 SimpleLongitudinalDebug *debug);
  virtual bool Shifting(control::ControlCommand *cmd);
  virtual bool EPB();

 private:
  const localization::LocalizationEstimate *localization_ = nullptr;
  const canbus::Chassis *chassis_ = nullptr;
  const planning::ADCTrajectory *trajectory_message_ = nullptr;

  std::unique_ptr<TrajectoryAnalyzer> trajectory_analyzer_;

  PIDController station_pid_controller_;

  std::shared_ptr<DependencyInjector> injector_;

  std::string name_;

  common::VehicleParam vehicle_param_;

  FILE *speed_log_file_ = nullptr;
  LonBasedPidControllerConf lon_based_pidcontroller_conf_;
  double previous_acceleration_ = 0.0;
  double previous_acceleration_reference_ = 0.0;
  LeadlagController station_leadlag_controller_;
};

}  // namespace control
}  // namespace apollo
