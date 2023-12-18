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

#include <deque>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "Eigen/Dense"

#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/common_msgs/localization_msgs/pose.pb.h"
#include "modules/common_msgs/perception_msgs/perception_obstacle.pb.h"
#include "modules/common_msgs/sensor_msgs/sensor_image.pb.h"
#include "modules/common_msgs/transform_msgs/transform.pb.h"
#include "modules/dreamview/proto/camera_update.pb.h"
#include "modules/dreamview_plus/proto/data_handler.pb.h"

#include "cyber/cyber.h"
#include "cyber/service_discovery/specific_manager/channel_manager.h"
#include "modules/dreamview/backend/common/handlers/websocket_handler.h"
#include "modules/dreamview_plus/backend/updater/updater_with_channels_base.h"
#include "modules/transform/buffer.h"

namespace apollo {
namespace dreamview {

struct CameraChannelUpdater {
  std::string curr_channel_name_;
  std::string curr_obstacle_channel_name_;
  std::shared_ptr<cyber::Reader<drivers::Image>> perception_camera_reader_;
  std::shared_ptr<cyber::Reader<apollo::perception::PerceptionObstacles>>
      perception_obstacle_reader_;
  std::vector<uint8_t> image_buffer_;
  CameraUpdate camera_update_;
  double current_image_timestamp_;
  std::unique_ptr<cyber::Timer> timer_;
  std::deque<std::shared_ptr<apollo::localization::LocalizationEstimate>>
      localization_queue_;
  std::mutex obstacle_mutex_;
  std::mutex image_mutex_;
  std::mutex localization_mutex_;
  std::vector<apollo::perception::BBox2D> bbox2ds;
  std::vector<int32_t> obstacle_id;
  std::vector<int32_t> obstacle_sub_type;
  bool enabled_;
  explicit CameraChannelUpdater(std::string channel_name)
      : curr_channel_name_(channel_name),
        perception_camera_reader_(nullptr),
        perception_obstacle_reader_(nullptr),
        current_image_timestamp_(0.0),
        enabled_(false) {}
};

class PerceptionCameraUpdater : public UpdaterWithChannelsBase {
 public:
  /**
   * @class PerceptionCameraUpdater
   *
   * @brief A module that collects camera image and localization (by collecting
   * localization & static transforms) to adjust camera as its real position/
   * rotation in front-end camera view for projecting HDmap to camera image
   */
  explicit PerceptionCameraUpdater(WebSocketHandler *websocket);
  void Stop();
  void StartStream(const double &time_interval_ms,
                   const std::string &channel_name = "",
                   nlohmann::json *subscribe_param = nullptr) override;
  void StopStream(const std::string& channel_name = "") override;
  void OnTimer(const std::string& channel_name = "");
  void PublishMessage(const std::string& channel_name = "") override;


  void GetUpdate(std::string *camera_update, const std::string& channel_name);

  void GetChannelMsg(std::vector<std::string> *channels) override;

 private:
  static constexpr double kImageScale = 0.6;
  void InitReaders();
  void OnCompressedImage(
      const std::shared_ptr<apollo::drivers::CompressedImage> &image);
  void OnImage(const std::shared_ptr<apollo::drivers::Image> &image,
               const std::string &channel_name);
  void OnLocalization(
      const std::shared_ptr<apollo::localization::LocalizationEstimate> &loc);
  void OnObstacles(
      const std::shared_ptr<apollo::perception::PerceptionObstacles>& obstacles,
      const std::string &channel_name);
  /**
   * @brief Since localization is updated more frequently than camera image,
   * compare image and localization timestamp to get the nearest
   * localization for image.
   */
  void GetImageLocalization(std::vector<double> *localization,
                            const std::string &channel_name);

  apollo::transform::Buffer *tf_buffer_ = apollo::transform::Buffer::Instance();
  bool QueryStaticTF(const std::string &frame_id,
                     const std::string &child_frame_id,
                     Eigen::Matrix4d *matrix);
  void GetLocalization2CameraTF(std::vector<double> *localization2camera_tf);
  std::shared_ptr<cyber::Reader<apollo::perception::PerceptionObstacles>>
  GetObstacleReader(const std::string &channel_name);
  CameraChannelUpdater *GetCameraChannelUpdater(
      const std::string &channel_name);

  WebSocketHandler *websocket_;

  bool perception_obstacle_enable_ = false;

  std::unique_ptr<cyber::Node> node_;
  std::map<std::string, CameraChannelUpdater *> channel_updaters_;
  std::mutex channel_updater_map_mutex_;
};

}  // namespace dreamview
}  // namespace apollo
