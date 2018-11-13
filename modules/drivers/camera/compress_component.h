/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#include "cyber/base/concurrent_object_pool.h"
#include "cyber/cyber.h"
#include "modules/drivers/camera/proto/config.pb.h"
#include "modules/drivers/proto/sensor_image.pb.h"

namespace apollo {
namespace drivers {
namespace camera {

using apollo::cyber::Component;
using apollo::cyber::Writer;
using apollo::cyber::base::CCObjectPool;
using apollo::drivers::Image;
using apollo::drivers::camera::config::Config;

class CompressComponent : public Component<Image> {
 public:
  bool Init() override;
  bool Proc(const std::shared_ptr<Image>& image) override;

 private:
  std::shared_ptr<CCObjectPool<CompressedImage>> image_pool_;
  std::shared_ptr<Writer<CompressedImage>> writer_ = nullptr;
  Config config_;
};

CYBER_REGISTER_COMPONENT(CompressComponent)
}  // namespace camera
}  // namespace drivers
}  // namespace apollo
