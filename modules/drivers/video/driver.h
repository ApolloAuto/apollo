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

#include <memory>
#include "cyber/cyber.h"
#include "modules/drivers/proto/sensor_image.pb.h"
#include "modules/drivers/video/proto/video_h265cfg.pb.h"
#include "modules/drivers/video/socket_input.h"

namespace apollo {
namespace drivers {
namespace video {

using apollo::drivers::CompressedImage;
using apollo::drivers::video::config::CameraH265Config;

class CameraDriver {
 public:
  explicit CameraDriver(const CameraH265Config *h265_cfg);
  ~CameraDriver() {}

  bool Poll(std::shared_ptr<CompressedImage> h265);
  void Init();
  int Port() { return config_.udp_port(); }
  int Record() { return config_.record(); }

 protected:
  CameraH265Config config_;
  std::shared_ptr<SocketInput> input_;
  bool PollByFrame(std::shared_ptr<CompressedImage> h265);
};

}  // namespace video
}  // namespace drivers
}  // namespace apollo
