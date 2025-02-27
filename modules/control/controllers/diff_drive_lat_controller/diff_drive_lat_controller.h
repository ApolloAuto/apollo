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
#include "modules/control/controllers/diff_drive_lat_controller/proto/diff_drive_lat_controller_conf.pb.h"

#include "modules/common/filters/digital_filter.h"
#include "modules/common/filters/mean_filter.h"
#include "modules/common/math/math_utils.h"
#include "modules/control/control_component/controller_task_base/common/pid_controller.h"
#include "modules/control/control_component/controller_task_base/common/trajectory_analyzer.h"
#include "modules/control/control_component/controller_task_base/control_task.h"

namespace apollo {
namespace control {

class DiffDriveLatController : public ControlTask {
 public:
  DiffDriveLatController();
  virtual ~DiffDriveLatController();

  common::Status Init(std::shared_ptr<DependencyInjector> injector) override;

  common::Status ComputeControlCommand(
      const localization::LocalizationEstimate *localization,
      const canbus::Chassis *chassis, const planning::ADCTrajectory *trajectory,
      control::ControlCommand *cmd) override;

  common::Status Reset() override;

  std::string Name() const override;

  void Stop() override;

 private:
  void ComputeLateralErrors(const double x, const double y, const double theta,
                            const double linear_v, const double angular_v,
                            const double linear_a,
                            const TrajectoryAnalyzer &trajectory_analyzer,
                            SimpleLateralDebug *debug,
                            const canbus::Chassis *chassis);

  void CloseLogFile();

  void InitializeFilters();

  void UpdateDrivingOrientation();

  bool LoadControlConf();

 private:
  const planning::ADCTrajectory *trajectory_message_ = nullptr;

  std::unique_ptr<TrajectoryAnalyzer> trajectory_analyzer_;

  common::DigitalFilter digital_filter_;

  DiffDriveLatControllerConf diff_drive_lat_controller_conf_;

  // vehicle parameter
  common::VehicleParam vehicle_param_;

  // MeanFilter heading_rate_filter_;
  common::MeanFilter lateral_error_filter_;
  common::MeanFilter heading_error_filter_;

  PIDController curv_pid_controller_;

  std::shared_ptr<DependencyInjector> injector_;

  std::string name_;

  std::ofstream steer_log_file_;

  double driving_orientation_ = 0.0;
  double query_relative_time_;

  // the following parameters are vehicle physics related.
  // control time interval
  double ts_ = 0.0;

  // distance between front and rear wheel center
  double wheelbase_ = 0.0;
  // mass of the vehicle
  double mass_ = 0.0;
  // distance from front wheel center to COM
  double lf_ = 0.0;
  // distance from rear wheel center to COM
  double lr_ = 0.0;
  // rotational inertia
  double iz_ = 0.0;
  // the ratio between the turn of the steering wheel and the turn of the wheels
  double steer_ratio_ = 0.0;

  // for compute the differential valute to estimate acceleration/lon_jerk
  double previous_lateral_acceleration_ = 0.0;

  double previous_heading_rate_ = 0.0;
  double previous_ref_heading_rate_ = 0.0;

  double previous_heading_acceleration_ = 0.0;
  double previous_ref_heading_acceleration_ = 0.0;

  double low_speed_bound_ = 0.0;
  double low_speed_window_ = 0.0;

  // longitudial length for look-ahead lateral error estimation during forward
  // driving and look-back lateral error estimation during backward driving
  // (look-ahead controller)
  double lookahead_station_low_speed_ = 0.0;
  double lookback_station_low_speed_ = 0.0;
  double lookahead_station_high_speed_ = 0.0;
  double lookback_station_high_speed_ = 0.0;

  double pre_ang_vel_ = 0.0;
};

}  // namespace control
}  // namespace apollo
