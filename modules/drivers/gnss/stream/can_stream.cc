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

#include "modules/drivers/gnss/stream/can_stream.h"

#include <cstring>
#include <vector>

#include "modules/common_msgs/basic_msgs/error_code.pb.h"

#include "cyber/cyber.h"
#include "modules/drivers/canbus/can_client/can_client_factory.h"
#include "modules/drivers/gnss/stream/stream.h"

namespace apollo {
namespace drivers {
namespace gnss {

CanStream::CanStream(const apollo::drivers::canbus::CANCardParameter& parameter)
    : parameter_(parameter) {}

CanStream::~CanStream() {}

bool CanStream::Connect() {
  // Init can client
  auto can_factory = apollo::drivers::canbus::CanClientFactory::Instance();
  can_factory->RegisterCanClients();
  can_client_ = can_factory->CreateCANClient(parameter_);
  if (!can_client_) {
    AERROR << "Failed to create can client.";
    return false;
  }
  AINFO << "Can client is successfully created.";
  if (can_client_->Start() != apollo::common::ErrorCode::OK) {
    AERROR << "Failed to start can client";
    return false;
  }
  AINFO << "Can client is started.";
  return true;
}

bool CanStream::Disconnect() { return true; }

size_t CanStream::read(uint8_t* buffer, size_t max_length) {
  std::vector<apollo::drivers::canbus::CanFrame> frames;
  static int32_t recv_frame_num = 1;
  if (can_client_->Receive(&frames, &recv_frame_num) !=
      apollo::common::ErrorCode::OK) {
    AERROR << "Failed to receive can frame.";
    return 0;
  }
  std::memcpy(buffer, &frames[0], sizeof(frames[0]));
  return sizeof(frames[0]);
}

size_t CanStream::write(const uint8_t* buffer, size_t length) {
  // TODO(dev): implement
  return 0;
}

Stream* Stream::create_can(
    const apollo::drivers::canbus::CANCardParameter& parameter) {
  return new CanStream(parameter);
}

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
