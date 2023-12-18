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

#include "cyber/cyber.h"
#include "cyber/service_discovery/specific_manager/channel_manager.h"
#include "modules/dreamview/backend/common/handlers/websocket_handler.h"
#include "modules/transform/buffer.h"

namespace apollo {
namespace dreamview {

class PerceptionCameraUpdater {
 public:
  /**
   * @class PerceptionCameraUpdater
   *
   * @brief A module that collects camera image and localization (by collecting
   * localization & static transforms) to adjust camera as its real position/
   * rotation in front-end camera view for projecting HDmap to camera image
   */
  explicit PerceptionCameraUpdater(WebSocketHandler *websocket);
  using DvCallback = std::function<bool(const std::string &string)>;

  void Start(DvCallback callback_api);
  void Stop();

  bool IsEnabled() const { return enabled_; }

  void GetUpdate(std::string *camera_update);

  void GetChannelMsg(std::vector<std::string> *channels);

  bool ChangeChannel(std::string channel);

 private:
  static constexpr double kImageScale = 0.6;
  std::vector<std::string> channels_;
  void InitReaders();
  void OnCompressedImage(
      const std::shared_ptr<apollo::drivers::CompressedImage> &image);
  void OnImage(const std::shared_ptr<apollo::drivers::Image> &image);
  void OnLocalization(
      const std::shared_ptr<apollo::localization::LocalizationEstimate> &loc);
  void OnObstacles(
      const std::shared_ptr<apollo::perception::PerceptionObstacles>
          &obstacles);
  /**
   * @brief Since localization is updated more frequently than camera image,
   * compare image and localization timestamp to get the nearest
   * localization for image.
   */
  void GetImageLocalization(std::vector<double> *localization);

  apollo::transform::Buffer *tf_buffer_ = apollo::transform::Buffer::Instance();
  bool QueryStaticTF(const std::string &frame_id,
                     const std::string &child_frame_id,
                     Eigen::Matrix4d *matrix);
  void GetLocalization2CameraTF(std::vector<double> *localization2camera_tf);

  WebSocketHandler *websocket_;
  CameraUpdate camera_update_;

  bool enabled_ = false;
  bool perception_obstacle_enable_ = false;
  double current_image_timestamp_;
  std::string curr_channel_name = "";

  std::unique_ptr<cyber::Node> node_;

  std::deque<std::shared_ptr<apollo::localization::LocalizationEstimate>>
      localization_queue_;
  std::shared_ptr<cyber::Reader<drivers::Image>> perception_camera_reader_;
  std::vector<uint8_t> image_buffer_;
  std::vector<double> tf_static_;
  std::vector<apollo::perception::BBox2D> bbox2ds;
  std::vector<int32_t> obstacle_id;
  std::vector<int32_t> obstacle_sub_type;
  std::mutex image_mutex_;
  std::mutex localization_mutex_;
  std::mutex obstacle_mutex_;
  DvCallback callback_api_;
};

}  // namespace dreamview
}  // namespace apollo
