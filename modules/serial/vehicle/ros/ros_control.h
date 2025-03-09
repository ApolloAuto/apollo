// Copyright 2025 WheelOS. All Rights Reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//  Created Date: 2025-01-16
//  Author: daohu527

#pragma once

#include "modules/serial/base_control.h"
#include "modules/serial/common/ring_buffer.h"

namespace apollo {
namespace serial {

class ROSControl : public BaseControl {
 public:
  // 数据解析状态
  enum class ParseState { SEEK_HEADER, READ_DATA, SEEK_TAIL, PROCESS_FRAME };

  // 帧类型
  enum class FrameType { UNKNOWN, DATA1, DATA2 };

  ROSControl();
  virtual ~ROSControl() = default;

  bool IsRunning() const;

  ::apollo::common::ErrorCode Start();

  void Stop();

  bool Send(const ControlCommand& control_command) override;

  Chassis GetChassis() const override {
    std::lock_guard<std::mutex> lock(mutex_);
    return chassis_;
  }

 private:
  void RecvThreadFunc();

 private:
  std::mutex mutex_;
  std::atomic<bool> is_running_ = {false};

  std::future<void> async_result_;

  ParseState state_;
  FrameType current_frame_type_;
  size_t data_payload_length_;
  std::vector<uint8_t> current_frame_data_;
  RingBuffer ring_buffer_;

  Chassis chassis_;

  DISALLOW_COPY_AND_ASSIGN(ROSControl);
};

}  // namespace serial
}  // namespace apollo
