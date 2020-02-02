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

#include "cyber/cyber.h"
#include "modules/dreamview/backend/handlers/websocket_handler.h"
#include "modules/dreamview/proto/camera_update.pb.h"
#include "modules/drivers/proto/sensor_image.pb.h"
#include "modules/localization/proto/localization.pb.h"
#include "modules/localization/proto/pose.pb.h"
#include "modules/transform/buffer.h"
#include "modules/transform/proto/transform.pb.h"

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

  void Start();
  void Stop();

  bool IsEnabled() const { return enabled_; }

  void GetUpdate(std::string *camera_update);

 private:
  static constexpr double kImageScale = 0.6;

  void InitReaders();
  void OnImage(const std::shared_ptr<apollo::drivers::CompressedImage> &image);
  void OnLocalization(
      const std::shared_ptr<apollo::localization::LocalizationEstimate> &loc);

  /**
   * @brief Since localization is updated more frequently than camera image,
   * compare image and localization timestamp to get the nearest localization
   * for image.
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
  double current_image_timestamp_;

  std::unique_ptr<cyber::Node> node_;

  std::deque<std::shared_ptr<apollo::localization::LocalizationEstimate>>
      localization_queue_;
  std::vector<uint8_t> image_buffer_;
  std::vector<double> tf_static_;

  std::mutex image_mutex_;
  std::mutex localization_mutex_;
};

}  // namespace dreamview
}  // namespace apollo
