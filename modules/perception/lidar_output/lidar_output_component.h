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
#include <queue>

#include <boost/filesystem.hpp>

#include "modules/common_msgs/perception_msgs/perception_benchmark.pb.h"
#include "modules/perception/lidar_output/proto/lidar_output_component_config.pb.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "cyber/component/component.h"
#include "modules/perception/common/onboard/inner_component_messages/lidar_inner_component_messages.h"

namespace apollo {
namespace perception {
namespace lidar {

using onboard::LidarFrameMessage;
using onboard::SensorFrameMessage;

class LidarOutputComponent : public cyber::Component<SensorFrameMessage> {
 public:
  LidarOutputComponent() {}
  virtual ~LidarOutputComponent();
  /**
   * @brief Init lidar output component
   *
   * @return true
   * @return false
   */
  bool Init() override;
  /**
   * @brief Process of lidar output component
   *
   * @param message sensor frame message
   * @return true
   * @return false
   */
  bool Proc(const std::shared_ptr<SensorFrameMessage>& message) override;

  /**
   * @brief Save benchmark frame for end-to-end evaluation
   *
   * @param message sensor frame message
   * @return true
   * @return false
   */
  bool SaveBenchmarkFrame(const std::shared_ptr<SensorFrameMessage>& message);

  bool SaveBenchmarkLidarFrame(
      const std::shared_ptr<LidarFrameMessage>& message);

  /**
   * @brief Benchmark thread fucntion
   *
   */
  void BenchmarkThreadFunc();

 private:
  std::string output_channel_name_;
  std::shared_ptr<apollo::cyber::Writer<PerceptionObstacles>> writer_;
  // for saving benchmark frame
  bool save_benchmark_frame_ = false;
  std::string benchmark_frame_output_dir_;
  std::mutex benchmark_mutex_;
  std::atomic<bool> is_terminate_;
  std::queue<std::shared_ptr<SensorFrameMessage>> message_buffer_;
  std::unique_ptr<std::thread> benchmark_thread_;

  std::string lidar_frame_output_dir_;
  std::shared_ptr<apollo::cyber::Reader<LidarFrameMessage>> lidar_frame_reader_;
  bool use_lidar_cooridinate_ = false;
  bool timestamp_two_decimal_format_ = false;
};

CYBER_REGISTER_COMPONENT(LidarOutputComponent);

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
