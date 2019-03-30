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
#include <string>
#include <thread>

#include "cyber/base/concurrent_object_pool.h"
#include "cyber/cyber.h"
#include "modules/drivers/proto/sensor_image.pb.h"
#include "modules/drivers/video/driver.h"
#include "modules/drivers/video/proto/video_h265cfg.pb.h"

namespace apollo {
namespace drivers {
namespace video {

using apollo::cyber::Component;
using apollo::cyber::Writer;
using apollo::cyber::base::CCObjectPool;
using apollo::drivers::video::config::CameraH265Config;

class CompCameraH265Compressed : public Component<> {
 public:
  ~CompCameraH265Compressed() {
    if (video_thread_->joinable()) {
      video_thread_->join();
    }
  }
  bool Init();

 private:
  void VideoPoll();

  std::shared_ptr<apollo::cyber::Writer<CompressedImage>> writer_;
  std::shared_ptr<std::thread> video_thread_;
  volatile bool runing_;
  std::unique_ptr<CameraDriver> camera_deivce_;
  std::string record_folder_;
  std::shared_ptr<CompressedImage> pb_image_ = nullptr;
};

CYBER_REGISTER_COMPONENT(CompCameraH265Compressed);

}  // namespace video
}  // namespace drivers
}  // namespace apollo
