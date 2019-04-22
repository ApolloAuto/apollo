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

#include "modules/drivers/video/driver.h"

namespace apollo {
namespace drivers {
namespace video {

using apollo::drivers::CompressedImage;
using apollo::drivers::video::config::CameraH265Config;

CameraDriver::CameraDriver(const CameraH265Config *h265_cfg) {
  config_ = *h265_cfg;
}

bool CameraDriver::Poll(std::shared_ptr<CompressedImage> h265) {
  return PollByFrame(h265);
}

bool CameraDriver::PollByFrame(std::shared_ptr<CompressedImage> h265Pb) {
  int ret = input_->GetFramePacket(h265Pb);
  if (ret < 0) {
    return false;
  }

  h265Pb->set_frame_id(config_.frame_id());
  uint64_t camera_timestamp = h265Pb->mutable_header()->camera_timestamp();
  uint64_t current_time = cyber::Time().Now().ToNanosecond();
  AINFO << "Get frame from port: " << config_.udp_port()
        << ", size: " << h265Pb->data().size() << ", ts: camera/host "
        << camera_timestamp << "/" << current_time << ", diff: "
        << static_cast<double>(current_time - camera_timestamp) * 1e-6;

  return true;
}

void CameraDriver::Init() {
  input_.reset(new SocketInput());
  input_->Init(config_.udp_port());
}

}  // namespace video
}  // namespace drivers
}  // namespace apollo
