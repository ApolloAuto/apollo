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
 * @file
 */

#ifndef MODEULES_L3_PERCEPTION_L3_PERCEPTION_H_
#define MODEULES_L3_PERCEPTION_L3_PERCEPTION_H_

#include <string>
#include <map>
#include <mutex>
#include <queue>

#include "modules/common/apollo_app.h"
#include "modules/common/macro.h"
#include "modules/drivers/proto/delphi_esr.pb.h"
#include "modules/drivers/proto/mobileye.pb.h"
#include "modules/l3_perception/proto/radar_obstacle.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/perception/proto/perception_obstacle.pb.h"
#include "ros/include/ros/ros.h"

/**
 * @namespace apollo::l3_perception
 * @brief apollo::l3_perception
 */
namespace apollo {
namespace l3_perception {

class L3Perception : public apollo::common::ApolloApp {
 public:
  std::string Name() const override;
  apollo::common::Status Init() override;
  apollo::common::Status Start() override;
  void Stop() override;

 private:
  // Upon receiving mobileye data
  void OnMobileye(const apollo::drivers::Mobileye& message);
  // Upon receiving radar data
  void OnDelphiESR(const apollo::drivers::DelphiESR& message);
  void OnLocalization(
      const apollo::localization::LocalizationEstimate& message);
  void OnTimer(const ros::TimerEvent&);

  RadarObstacles FilterRadarObstacles(
      const RadarObstacles& radar_obstacles);

  ros::Timer timer_;
  apollo::perception::PerceptionObstacles mobileye_obstacles_;
  apollo::localization::LocalizationEstimate localization_;
  RadarObstacles current_radar_obstacles_;
  std::queue<RadarObstacles> last_radar_obstacles_;
  std::mutex l3_mutex_;

};

}  // namespace l3_perception
}  // namespace apollo

#endif  // MODULES_L3_PERCEPTION_L3_PERCEPTION_H_
