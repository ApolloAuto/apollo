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

/**
 * @file
 */

#ifndef MODEULES_ULTANALYSE_ULTANALYSE_H_
#define MODEULES_ULTANALYSE_ULTANALYSE_H_

#include <map>
#include <mutex>
#include <queue>
#include <string>

// #include "modules/canbus/proto/chassis.pb.h"
#include "modules/common/apollo_app.h"
#include "modules/common/macro.h"
#include "modules/drivers/ultanalyse/proto/ultanalyse.pb.h"
#include "modules/monitor/proto/system_status.pb.h"
#include "ros/include/ros/ros.h"
#include "std_msgs/Int32MultiArray.h"

/**
 * @namespace apollo::ultanalyse
 * @brief apollo::ultanalyse
 */
namespace apollo {
namespace ultanalyse {

class Ultanalyse : public apollo::common::ApolloApp {
 public:
  std::string Name() const override;
  apollo::common::Status Init() override;
  apollo::common::Status Start() override;
  void Stop() override;

 private:
  // void OnChassis(const apollo::canbus::Chassis& message);
  void OnSystemStatus(const apollo::monitor::SystemStatus & message);
  void ObstAnalyse();
  // void canbus_ObstAnalyse();
  void OnUltrasound(const std_msgs::Int32MultiArray & message);

  // apollo::canbus::Chassis chassis_;
  apollo::monitor::SystemStatus system_status_;
  apollo::ultanalyse::ObstAnalyse obst_analyse_;
  apollo::ultanalyse::ChassisUlt ChassisUlt_;

  std::mutex mutex_;
  // ros::Timer timer_;
};

}  // namespace ultanalyse
}  // namespace apollo

#endif  // MODULES_ULTANALYSE_ULTANALYSE_H_
