/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include <limits>
#include <memory>
#include <string>

#include "modules/common_msgs/control_msgs/control_cmd.pb.h"
#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/common_msgs/planning_msgs/planning_command.pb.h"
#include "modules/common_msgs/prediction_msgs/prediction_obstacle.pb.h"
#include "modules/common_msgs/routing_msgs/routing.pb.h"
#include "modules/dreamview/backend/common/sim_control_manager/proto/sim_control_internal.pb.h"

#include "cyber/common/log.h"
#include "cyber/cyber.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/math/linear_interpolation.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/util/message_util.h"
#include "modules/dreamview/backend/common/dreamview_gflags.h"
#include "modules/dreamview/backend/common/map_service/map_service.h"
#include "modules/dreamview/backend/common/sim_control_manager/common/sim_control_gflags.h"
#include "modules/dreamview/backend/common/sim_control_manager/core/sim_control_base.h"

/**
 * @namespace apollo::dreamview
 * @brief apollo::dreamview
 */
namespace apollo {
namespace dreamview {

/**
 * @class sim control base for dynamic models (perfect model excluded)
 */
class SimControlWithModelBase : public SimControlBase {
 public:
  /**
   * @brief Construct a new Sim Control With Model Base object
   * @param node_name
   */
  explicit SimControlWithModelBase(const std::string& node_name);

  /**
   * @brief Starts the timer to publish simulated localization and chassis
   * messages.
   */
  void Start() override;

  void Start(double x, double y, double v = 0.0, double a = 0.0) override;

  /**
   * @brief Set vehicle position.
   */
  void ReSetPoinstion(double x, double y, double heading) override;

  void Stop() override;

  /**
   * @brief Resets the internal state.
   */
  void Reset() override;

 protected:
  void InitTimerAndIO();

  void UpdateGearPosition();
  void InitStartPoint(nlohmann::json start_point_attr,
                      bool use_start_point_position = false);

  void InternalReset();
  void OnControlCommand(const apollo::control::ControlCommand& control_command);
  void OnPlanningCommand(
      const std::shared_ptr<apollo::planning::PlanningCommand>&
          planning_command);
  void OnPredictionObstacles(
      const std::shared_ptr<apollo::prediction::PredictionObstacles>&
          obstacles);

  virtual void SetStartPoint(const ::apollo::sim_control::SimCarStatus& point);

  void PublishChassis(const std::string model_name);
  void PublishLocalization(const std::string model_name);
  void PublishDummyPrediction();

  // Whether the sim control is initialized.
  bool start_auto_ = false;
  bool auto_car_is_moving_ = false;
  bool planning_command_is_arrival_ = false;
  bool control_command_is_arrival_ = false;

  std::unique_ptr<cyber::Node> node_;

  std::shared_ptr<cyber::Reader<apollo::control::ControlCommand>>
      control_command_reader_;
  std::shared_ptr<cyber::Reader<apollo::planning::PlanningCommand>>
      planning_command_reader_;
  std::shared_ptr<cyber::Reader<apollo::localization::LocalizationEstimate>>
      localization_reader_;
  std::shared_ptr<cyber::Reader<apollo::prediction::PredictionObstacles>>
      prediction_reader_;

  std::shared_ptr<cyber::Writer<apollo::canbus::Chassis>> chassis_writer_;
  std::shared_ptr<cyber::Writer<apollo::prediction::PredictionObstacles>>
      prediction_writer_;
  std::shared_ptr<cyber::Writer<apollo::localization::LocalizationEstimate>>
      localization_writer_;

  apollo::sim_control::SimCarStatus previous_point_;
  apollo::sim_control::SimCarStatus current_point_;

  apollo::control::ControlCommand control_cmd_;

  // Filtered control command, used to fulfill chassis feedback
  apollo::control::ControlCommand filtered_control_cmd_;

  // The header of the routing planning is following.
  apollo::common::Header current_routing_header_;

  apollo::common::PathPoint adc_position_;

  // vehicle parameter
  apollo::common::VehicleParam vehicle_param_;

  int gear_position_;
  double dt_;

  // Time interval of the timer, in milliseconds.
  static constexpr double kModelIntervalMs = 10;
  static constexpr double kSimPredictionIntervalMs = 100;

  // Whether planning has requested a re-routing.
  bool re_routing_triggered_ = false;

  // Whether to send dummy predictions
  bool send_dummy_prediction_ = true;
  MapService* map_service_;
};

}  // namespace dreamview
}  // namespace apollo
