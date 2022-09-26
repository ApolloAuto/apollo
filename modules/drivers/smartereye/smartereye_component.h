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

#pragma once

#include <atomic>
#include <future>
#include <memory>
#include <vector>

#include "cyber/cyber.h"
#include "modules/common_msgs/sensor_msgs/sensor_image.pb.h"
#include "modules/common_msgs/sensor_msgs/smartereye.pb.h"
#include "modules/drivers/smartereye/proto/config.pb.h"
#include "modules/drivers/smartereye/smartereye_device.h"
#include "third_party/camera_library/smartereye/include/frameext.h"
#include "third_party/camera_library/smartereye/include/obstacleData.h"
#include "third_party/camera_library/smartereye/include/obstaclepainter.h"
#include "third_party/camera_library/smartereye/include/roadwaypainter.h"
#include "third_party/camera_library/smartereye/include/yuv2rgb.h"

namespace apollo {
namespace drivers {
namespace smartereye {

using apollo::cyber::Component;
using apollo::cyber::Reader;
using apollo::cyber::Writer;
using apollo::drivers::Image;
using apollo::drivers::smartereye::config::Config;

class SmartereyeComponent : public Component<> {
 public:
  bool Init() override;
  ~SmartereyeComponent();

 protected:
  void run();
  bool SetCallback();
  bool Callback(RawImageFrame *rawFrame);
  void processFrame(int frameId, char *image, char *extended, int64_t time,
                    int width, int height);
  void processFrame(int frameId, char *image, uint32_t dataSize, int width,
                    int height, int frameFormat);

 private:
  std::shared_ptr<Writer<Image>> writer_ = nullptr;
  std::shared_ptr<Writer<SmartereyeObstacles>> SmartereyeObstacles_writer_ =
      nullptr;
  std::shared_ptr<Writer<SmartereyeLanemark>> SmartereyeLanemark_writer_ =
      nullptr;
  std::unique_ptr<SmartereyeDevice> camera_device_ = nullptr;
  std::shared_ptr<Config> camera_config_ = nullptr;
  uint32_t spin_rate_ = 200;
  uint32_t device_wait_ = 2000;
  std::future<void> async_result_;
  std::atomic<bool> running_ = {false};
  bool b_ispolling_ = false;
};

CYBER_REGISTER_COMPONENT(SmartereyeComponent)
}  // namespace smartereye
}  // namespace drivers
}  // namespace apollo
