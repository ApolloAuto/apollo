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

/**
 * @file
 */

#pragma once

#include <memory>
#include <mutex>
#include <string>

#include "cyber/node/node.h"
#include "cyber/node/reader.h"
#include "modules/common_msgs/chassis_msgs/chassis.pb.h"
#include "modules/common/status/status.h"
#include "modules/common_msgs/sensor_msgs/sensor_image.pb.h"
#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/common_msgs/perception_msgs/perception_obstacle.pb.h"
#include "modules/third_party_perception/common/third_party_perception_gflags.h"
#include "modules/third_party_perception/proto/radar_obstacle.pb.h"

/**
 * @namespace apollo::third_party_perception
 * @brief apollo::third_party_perception
 */
namespace apollo {
namespace third_party_perception {

class ThirdPartyPerception {
 public:
  explicit ThirdPartyPerception(apollo::cyber::Node* const node);
  ThirdPartyPerception() = default;
  virtual ~ThirdPartyPerception() = default;
  std::string Name() const;
  apollo::common::Status Init();
  apollo::common::Status Start();
  void Stop();

  // Upon receiving localization data
  void OnLocalization(
      const apollo::localization::LocalizationEstimate& message);
  // Upon receiving chassis data
  void OnChassis(const apollo::canbus::Chassis& message);
  // publish perception obstacles when timer is triggered
  virtual bool Process(apollo::perception::PerceptionObstacles* const response);

 protected:
  std::mutex third_party_perception_mutex_;
  apollo::localization::LocalizationEstimate localization_;
  apollo::canbus::Chassis chassis_;
  RadarObstacles current_radar_obstacles_;
  RadarObstacles last_radar_obstacles_;
  std::shared_ptr<apollo::cyber::Node> node_ = nullptr;
  std::shared_ptr<
      apollo::cyber::Reader<apollo::localization::LocalizationEstimate>>
      localization_reader_ = nullptr;
  std::shared_ptr<apollo::cyber::Reader<apollo::canbus::Chassis>>
      chassis_reader_ = nullptr;
};

}  // namespace third_party_perception
}  // namespace apollo
