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

#include "modules/drivers/canbus/can_client/fake/fake_can_client.h"

#include <cstring>
#include <thread>

namespace apollo {
namespace drivers {
namespace canbus {
namespace can {

using apollo::common::ErrorCode;

bool FakeCanClient::Init(const CANCardParameter &param) { return true; }

ErrorCode FakeCanClient::Start() { return ErrorCode::OK; }

void FakeCanClient::Stop() {}

ErrorCode FakeCanClient::Send(const std::vector<CanFrame> &frames,
                              int32_t *const frame_num) {
  if (frame_num == nullptr) {
    AERROR << "frame_num pointer is null";
    return ErrorCode::CAN_CLIENT_ERROR_BASE;
  }

  if (static_cast<size_t>(*frame_num) != frames.size()) {
    AERROR << "frame num is incorrect.";
    return ErrorCode::CAN_CLIENT_ERROR_FRAME_NUM;
  }
  for (size_t i = 0; i < frames.size(); ++i) {
    ADEBUG << "send frame i:" << i;
    ADEBUG << frames[i].CanFrameString();
    frame_info_ << frames[i].CanFrameString();
  }
  ++send_counter_;
  return ErrorCode::OK;
}

ErrorCode FakeCanClient::Receive(std::vector<CanFrame> *const frames,
                                 int32_t *const frame_num) {
  if (frame_num == nullptr || frames == nullptr) {
    AERROR << "frames or frame_num pointer is null";
    return ErrorCode::CAN_CLIENT_ERROR_BASE;
  }
  frames->resize(*frame_num);
  const int MOCK_LEN = 8;
  for (size_t i = 0; i < frames->size(); ++i) {
    for (int j = 0; j < MOCK_LEN; ++j) {
      (*frames)[i].data[j] = static_cast<uint8_t>(j);
    }
    (*frames)[i].id = static_cast<uint32_t>(i);
    (*frames)[i].len = MOCK_LEN;
    ADEBUG << (*frames)[i].CanFrameString() << "frame_num[" << i << "]";
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  ++recv_counter_;
  return ErrorCode::OK;
}

std::string FakeCanClient::GetErrorString(const int32_t /*status*/) {
  return "";
}

}  // namespace can
}  // namespace canbus
}  // namespace drivers
}  // namespace apollo
