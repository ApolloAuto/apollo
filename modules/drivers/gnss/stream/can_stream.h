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

#include "modules/common_msgs/drivers_msgs/can_card_parameter.pb.h"

#include "modules/drivers/canbus/can_client/can_client.h"
#include "modules/drivers/gnss/stream/stream.h"

namespace apollo {
namespace drivers {
namespace gnss {

class CanStream : public Stream {
 public:
  explicit CanStream(
      const apollo::drivers::canbus::CANCardParameter &parameter);
  ~CanStream();

  virtual bool Connect();
  virtual bool Disconnect();
  virtual size_t read(uint8_t *buffer, size_t max_length);
  virtual size_t write(const uint8_t *data, size_t length);

 private:
  apollo::drivers::canbus::CANCardParameter parameter_;
  std::unique_ptr<apollo::drivers::canbus::CanClient> can_client_;
};

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
