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

#include <memory>
#include <string>

#include "modules/drivers/lidar/hesai/proto/config.pb.h"
#include "cyber/cyber.h"
#include "modules/drivers/lidar/hesai/driver/driver.h"
#include "modules/drivers/lidar/hesai/parser/parser_factory.h"

namespace apollo {
namespace drivers {
namespace hesai {

class HesaiComponent : public ::apollo::cyber::Component<> {
 public:
  ~HesaiComponent() {}
  bool Init() override;

 private:
  std::shared_ptr<HesaiDriver> driver_;
  std::shared_ptr<Parser> parser_;
  Config conf_;
};

CYBER_REGISTER_COMPONENT(HesaiComponent)

}  // namespace hesai
}  // namespace drivers
}  // namespace apollo
