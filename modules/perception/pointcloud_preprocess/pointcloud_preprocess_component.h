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

#include <atomic>
#include <limits>
#include <memory>
#include <string>

#include "modules/common_msgs/sensor_msgs/pointcloud.pb.h"
#include "modules/perception/pointcloud_preprocess/proto/pointcloud_preprocess_component_config.pb.h"

#include "cyber/component/component.h"
#include "modules/perception/common/onboard/inner_component_messages/lidar_inner_component_messages.h"
#include "modules/perception/common/onboard/transform_wrapper/transform_wrapper.h"
#include "modules/perception/pointcloud_preprocess/preprocessor/pointcloud_preprocessor.h"

namespace apollo {
namespace perception {
namespace lidar {

class PointCloudPreprocessComponent
    : public cyber::Component<drivers::PointCloud> {
 public:
  PointCloudPreprocessComponent() = default;
  virtual ~PointCloudPreprocessComponent() = default;
  /**
   * @brief Init pointcloud preprocess component
   *
   * @return true
   * @return false
   */
  bool Init() override;
  /**
   * @brief Process of pointcloud preprocess
   *
   * @param message Pointcloud message from driver
   * @return true
   * @return false
   */
  bool Proc(const std::shared_ptr<drivers::PointCloud>& message) override;

 private:
  bool InitAlgorithmPlugin();
  bool InternalProc(
      const std::shared_ptr<const drivers::PointCloud>& in_message,
      const std::shared_ptr<onboard::LidarFrameMessage>& out_message);

 private:
  static std::atomic<uint32_t> seq_num_;
  std::string sensor_name_;
  std::string config_path_;
  std::string config_file_;
  float lidar_query_tf_offset_ = 20.0f;
  std::string lidar2novatel_tf2_child_frame_id_;
  std::string output_channel_name_;
  std::string pointcloud_preprocess_config_file_;
  onboard::TransformWrapper lidar2world_trans_;
  base::SensorInfo sensor_info_;

  std::shared_ptr<apollo::cyber::Writer<onboard::LidarFrameMessage>> writer_;

  // preprocessor
  BasePointCloudPreprocessor* cloud_preprocessor_;
};

CYBER_REGISTER_COMPONENT(PointCloudPreprocessComponent);

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
