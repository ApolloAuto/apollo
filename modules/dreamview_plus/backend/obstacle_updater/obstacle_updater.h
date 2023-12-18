/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/common_msgs/localization_msgs/pose.pb.h"
#include "modules/common_msgs/perception_msgs/perception_obstacle.pb.h"
#include "modules/dreamview_plus/proto/obstacle.pb.h"

#include "cyber/cyber.h"
#include "cyber/service_discovery/specific_manager/channel_manager.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/dreamview/backend/common/handlers/websocket_handler.h"
#include "modules/dreamview_plus/backend/updater/updater_with_channels_base.h"
namespace apollo {
namespace dreamview {

using apollo::localization::LocalizationEstimate;
using apollo::localization::Pose;
using apollo::perception::PerceptionObstacle;
using apollo::perception::PerceptionObstacles;

struct ObstacleChannelUpdater {
  std::string curr_channel_name_;
  std::shared_ptr<cyber::Reader<PerceptionObstacles>>
      perception_obstacle_reader_;
  std::unordered_map<std::string, Object> obj_map_;
  std::unique_ptr<cyber::Timer> timer_;
  std::vector<PerceptionObstacle> obstacles_;
  Obstacles obstacle_objects_;
  explicit ObstacleChannelUpdater(std::string channel_name)
      : curr_channel_name_(channel_name),
        perception_obstacle_reader_(nullptr) {}
};
/**
 * @class ObstacleUpdater
 * @brief A wrapper around WebSocketHandler to keep pushing obstalcles to
 * frontend via websocket.
 */
class ObstacleUpdater : public UpdaterWithChannelsBase {
 public:
  explicit ObstacleUpdater(WebSocketHandler* websocket);
  void StartStream(const double& time_interval_ms,
                   const std::string& channel_name = "",
                   nlohmann::json* subscribe_param = nullptr) override;
  void StopStream(const std::string& channel_name) override;
  void OnTimer(const std::string& channel_name = "");
  void PublishMessage(const std::string& channel_name = "") override;
  void GetChannelMsg(std::vector<std::string>* channels);
  void Stop();
  void Init();

 private:
  bool enabled_ = false;
  WebSocketHandler* websocket_;
  std::unique_ptr<cyber::Node> node_;
  std::map<std::string, ObstacleChannelUpdater*> obstacle_channel_updater_map_;
  std::mutex channel_updater_map_mutex_;
  std::mutex updater_publish_mutex_;
  std::shared_ptr<cyber::Reader<LocalizationEstimate>> localization_reader_;
  Pose adc_pose_;
  ObstacleChannelUpdater* GetObstacleChannelUpdater(
      const std::string& channel_name);
  void SetObstacleSource(const PerceptionObstacle& obstacle, Object* obj);
  void SetObstacleType(const PerceptionObstacle::Type obstacle_type,
                       const PerceptionObstacle::SubType obstalce_sub_type,
                       Object* obj);
  void SetObstaclePolygon(const PerceptionObstacle& obstalce, Object* obj);
  void SetObstacleInfo(const PerceptionObstacle& obstalce, Object* obj);
  void SetObstacleSensorMeasurements(const PerceptionObstacle& obstacle,
                                     Object* obj,
                                     ObstacleChannelUpdater* channel_updater);
  void SetADCPosition(Object* auto_driving_car);
  void OnObstacles(const std::shared_ptr<PerceptionObstacles>& obstacles,
                   const std::string& channel_name);
  void GetObjects(std::string* to_send, std::string channel_name);
  void OnLocalization(
      const std::shared_ptr<LocalizationEstimate>& localization);
};

}  // namespace dreamview
}  // namespace apollo
