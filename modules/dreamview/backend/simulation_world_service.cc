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

#include "modules/dreamview/backend/simulation_world_service.h"

#include <algorithm>
#include <chrono>
#include <vector>

#include "google/protobuf/util/json_util.h"
#include "modules/canbus/proto/chassis.pb.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/proto/geometry.pb.h"
#include "modules/common/time/time.h"
#include "modules/dreamview/backend/trajectory_point_collector.h"
#include "modules/dreamview/proto/simulation_world.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/planning/proto/planning.pb.h"

using apollo::common::Point3D;
using apollo::common::adapter::AdapterManager;
using apollo::common::adapter::MonitorAdapter;
using apollo::common::adapter::LocalizationAdapter;
using apollo::common::adapter::ChassisAdapter;
using apollo::common::adapter::PlanningTrajectoryAdapter;
using apollo::common::config::VehicleConfigHelper;
using apollo::common::monitor::MonitorMessage;
using apollo::common::monitor::MonitorMessageItem;
using apollo::localization::LocalizationEstimate;
using apollo::planning::ADCTrajectory;
using apollo::planning::ADCTrajectoryPoint;
using apollo::canbus::Chassis;
using apollo::common::time::Clock;
using apollo::common::time::ToSecond;
using apollo::common::time::millis;

using Json = nlohmann::json;

namespace apollo {
namespace dreamview {

namespace {

double CalculateAcceleration(const Point3D &acceleration,
                             const Point3D &velocity) {
  // Calculates the dot product of acceleration and velocity. The sign
  // of this projection indicates whether this is acceleration or
  // deceleration.
  double projection =
      acceleration.x() * velocity.x() + acceleration.y() * velocity.y();

  // Calculates the magnitude of the acceleration. Negate the value if
  // it is indeed a deceleration.
  double magnitude = std::hypot(acceleration.x(), acceleration.y());
  return std::signbit(projection) ? -magnitude : magnitude;
}

Object::DisengageType DeduceDisengageType(const ::Chassis &chassis) {
  if (chassis.error_code() != Chassis::NO_ERROR) {
    return Object::DISENGAGE_UNKNOWN;
  }

  if (chassis.driving_mode() == Chassis::COMPLETE_AUTO_DRIVE) {
    return Object::DISENGAGE_NONE;
  }

  if (chassis.driving_mode() == Chassis::COMPLETE_MANUAL) {
    return Object::DISENGAGE_MANUAL;
  }

  return Object::DISENGAGE_UNKNOWN;
}

}  // namespace

namespace internal {

template <>
void UpdateSimulationWorld<MonitorAdapter>(const MonitorMessage &monitor_msg,
                                           SimulationWorld *world) {
  std::vector<MonitorMessageItem> updated;
  updated.reserve(SimulationWorldService::kMaxMonitorItems);
  // Save the latest messages at the top of the history.
  int remove_size =
      monitor_msg.item_size() > SimulationWorldService::kMaxMonitorItems
          ? monitor_msg.item_size() - SimulationWorldService::kMaxMonitorItems
          : 0;
  std::copy(monitor_msg.item().begin(),
            std::prev(monitor_msg.item().end(), remove_size),
            std::back_inserter(updated));

  // Copy over the previous messages until there is no more history or
  // the max number of total messages has been hit.
  auto history = world->monitor().item();
  remove_size = history.size() + monitor_msg.item_size() -
                SimulationWorldService::kMaxMonitorItems;
  if (remove_size < 0) {
    remove_size = 0;
  }
  std::copy(history.begin(), std::prev(history.end(), remove_size),
            std::back_inserter(updated));

  // Refresh the monitor message list in simulation_world.
  ::google::protobuf::RepeatedPtrField<MonitorMessageItem> items(
      updated.begin(), updated.end());
  world->mutable_monitor()->mutable_item()->Swap(&items);
  world->mutable_monitor()->mutable_header()->set_timestamp_sec(
      ToSecond(Clock::Now()));
}

template <>
void UpdateSimulationWorld<LocalizationAdapter>(
    const LocalizationEstimate &localization, SimulationWorld *world) {
  Object *auto_driving_car = world->mutable_auto_driving_car();
  const auto &pose = localization.pose();

  // Updates position with the input localization message.
  auto_driving_car->set_position_x(pose.position().x());
  auto_driving_car->set_position_y(pose.position().y());

  // Updates heading with the input localization message. The pose
  // within the localization message can carry the heading information
  // in either heading() or orientation(). Which one to use depends on
  // which one is correctly set.
  auto_driving_car->set_heading(pose.heading());

  // Updates acceleration with the input localization message.
  auto_driving_car->set_speed_acceleration(CalculateAcceleration(
      pose.linear_acceleration(), pose.linear_velocity()));

  // Updates the timestamp with the timestamp inside the localization
  // message header. It is done on both the SimulationWorld object
  // itself and its auto_driving_car() field.
  auto_driving_car->set_timestamp_sec(localization.header().timestamp_sec());
  world->set_timestamp_sec(localization.header().timestamp_sec());
}

template <>
void UpdateSimulationWorld<ChassisAdapter>(const Chassis &chassis,
                                           SimulationWorld *world) {
  Object *auto_driving_car = world->mutable_auto_driving_car();

  auto_driving_car->set_speed(chassis.speed_mps());
  auto_driving_car->set_throttle_percentage(chassis.throttle_percentage());
  auto_driving_car->set_brake_percentage(chassis.brake_percentage());

  // In case of out-of-range percentages, reduces it to zero.
  int angle_percentage = chassis.steering_percentage();
  if (angle_percentage > 100 || angle_percentage < -100) {
    angle_percentage = 0;
  }
  auto_driving_car->set_steering_angle(angle_percentage);

  if (chassis.signal().turn_signal() == ::apollo::canbus::Signal::TURN_LEFT) {
    auto_driving_car->set_current_signal("LEFT");
  } else if (chassis.signal().turn_signal() ==
             ::apollo::canbus::Signal::TURN_RIGHT) {
    auto_driving_car->set_current_signal("RIGHT");
  } else {
    auto_driving_car->set_current_signal("");
  }

  auto_driving_car->set_disengage_type(DeduceDisengageType(chassis));

  const auto &vehicle_param = VehicleConfigHelper::GetConfig().vehicle_param();
  auto_driving_car->set_height(vehicle_param.height());
  auto_driving_car->set_width(vehicle_param.width());
  auto_driving_car->set_length(vehicle_param.length());

  // Updates the timestamp with the timestamp inside the chassis
  // message header. It is done on both the SimulationWorld object
  // itself and its auto_driving_car() field.
  auto_driving_car->set_timestamp_sec(chassis.header().timestamp_sec());
  world->set_timestamp_sec(chassis.header().timestamp_sec());
}

template <>
void UpdateSimulationWorld<PlanningTrajectoryAdapter>(
    const ADCTrajectory &trajectory, SimulationWorld *world) {
  const double cutoff_time = world->auto_driving_car().timestamp_sec();
  const double header_time = trajectory.header().timestamp_sec();
  const size_t trajectory_length = trajectory.adc_trajectory_point_size();

  util::TrajectoryPointCollector collector(world);

  size_t i = 0;
  bool collecting_started = false;
  while (i < trajectory_length) {
    const ADCTrajectoryPoint &point = trajectory.adc_trajectory_point(i);
    // Trajectory points with a timestamp older than the cutoff time
    // (which is effectively the timestamp of the most up-to-date
    // localization/chassis message) will be dropped.
    //
    // Note that the last two points are always included.
    if (collecting_started ||
        point.relative_time() + header_time >= cutoff_time) {
      collecting_started = true;
      collector.Collect(point);
      if (i == trajectory_length - 1) {
        // Break if the very last point is collected.
        break;
      } else if (i == trajectory_length - 2) {
        // Move on to the last point if the last but one is collected.
        i = trajectory_length - 1;
      } else if (i < trajectory_length - 2) {
        // When collecting the trajectory points, downsample with a
        // ratio of 10.
        i += 10;
        if (i > trajectory_length - 2) {
          i = trajectory_length - 2;
        }
      } else {
        break;
      }
    } else {
      ++i;
    }
  }

  world->set_timestamp_sec(header_time);
}

}  // namespace internal

constexpr int SimulationWorldService::kMaxMonitorItems;

SimulationWorldService::SimulationWorldService() {
  world_.set_map_md5("initialize");
  AdapterManager::Init();
  VehicleConfigHelper::Init();
  RegisterDataCallback("Monitor", AdapterManager::GetMonitor());
  RegisterDataCallback("Chassis", AdapterManager::GetChassis());
  RegisterDataCallback("Localization", AdapterManager::GetLocalization());
  RegisterDataCallback("Planning", AdapterManager::GetPlanningTrajectory());
}

Json SimulationWorldService::GetUpdateAsJson() const {
  std::string sim_world_json;
  ::google::protobuf::util::MessageToJsonString(world_, &sim_world_json);

  Json update;
  update["type"] = "sim_world_update";
  update["world"] = Json::parse(sim_world_json);
  update["timestamp"] = apollo::common::time::AsInt64<millis>(Clock::Now());

  return update;
}

}  // namespace dreamview
}  // namespace apollo
