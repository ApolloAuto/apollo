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

/**
 * @file vehicle_state_proxy.h
 **/

#ifndef MODULES_PLANNING_PROXY_VEHICLE_STATE_PROXY_H_
#define MODULES_PLANNING_PROXY_VEHICLE_STATE_PROXY_H_

#include "modules/canbus/proto/chassis.pb.h"
#include "modules/common/configs/proto/vehicle_config.pb.h"
#include "modules/common/proto/path_point.pb.h"
#include "modules/localization/proto/localization.pb.h"

namespace apollo {
namespace planning {

class Frame;
class VehicleStateProxy {
 public:
  VehicleStateProxy() = default;

  void set_vehicle_chassis(
      const ::apollo::common::config::VehicleConfig& config,
      const ::apollo::localization::LocalizationEstimate& localization_estimate,
      const ::apollo::canbus::Chassis& chassis);

  const ::apollo::localization::LocalizationEstimate& localization_estimate();
  const ::apollo::canbus::Chassis& vehicle_chassis() const;
  const ::apollo::canbus::Chassis::GearPosition gear() const;

  // adc stand for autonomous driving car detected status
  double linear_velocity() const;
  double heading() const;
  double linear_acceleration() const;
  double curvature() const;
  double timestamp() const;

  // planned status based on car status projection and forward motion prediction
  // if projection is failed (return false),
  // init_point with adc_status and index = 0 is returned;
  // init point relative time with respect to current car status proto
  const ::apollo::common::TrajectoryPoint init_point(
      const Frame* const prev_frame) const;
  bool need_planner_reinit(const Frame* const prev_frame) const;

 private:
  void init(const ::apollo::common::config::VehicleConfig& config);

 private:
  ::apollo::common::config::VehicleConfig _config;
  ::apollo::localization::LocalizationEstimate _localization_estimate;
  ::apollo::canbus::Chassis _chassis;
  double _linear_velocity = 0.0;
  double _heading = 0.0;
  double _linear_acceleration = 0.0;
  double _curvature = 0.0;
};

}  // namespace planning
}  // namespace apollo

#endif  // MODULES_PLANNING_PROXY_VEHICLE_STATE_PROXY_H_
