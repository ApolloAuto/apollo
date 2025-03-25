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

#include "modules/serial/vehicle/ros/ros_control.h"

#include "cyber/common/log.h"
#include "cyber/task/task.h"
#include "cyber/time/time.h"
#include "modules/serial/common/util.h"
#include "modules/serial/vehicle/ros/ros_parser.h"

namespace apollo {
namespace serial {

constexpr u_int32_t SERIAL_MAX_LENGTH = 128;

ROSControl::ROSControl(const SerialConf& serial_conf)
    : BaseControl(serial_conf),
      state_(ParseState::SEEK_HEADER),
      current_frame_type_(FrameType::UNKNOWN),
      data_payload_length_(0),
      ring_buffer_(256) {}

bool ROSControl::Send(const ControlCommand& control_command) {
  uint8_t data[11] = {0};
  size_t length = 11;
  ROSParser::Encode(control_command, data, length);
  serial_stream_->write(data, length);
  return true;
}

::apollo::common::ErrorCode ROSControl::Start() {
  if (is_running_.exchange(true)) {
    AERROR << "ROSControl has already started.";
    return ::apollo::common::ErrorCode::CANBUS_ERROR;
  }

  async_result_ = cyber::Async(&ROSControl::RecvThreadFunc, this);
  return ::apollo::common::ErrorCode::OK;
}

void ROSControl::Stop() {
  if (IsRunning()) {
    AINFO << "Stopping ROSControl ...";
    is_running_.exchange(false);
    async_result_.wait();
  } else {
    AINFO << "ROSControl is not running.";
  }
  AINFO << "ROSControl stopped [ok].";
}

bool ROSControl::IsRunning() const { return is_running_.load(); }

void ROSControl::RecvThreadFunc() {
  while (IsRunning()) {
    // 1. Read data from the serial port and write it to the ring buffer
    uint8_t read_buffer[SERIAL_MAX_LENGTH];
    size_t bytes_read = serial_stream_->read(read_buffer, sizeof(read_buffer));
    if (bytes_read > 0) {
      ring_buffer_.Write(read_buffer, bytes_read);
      // 2. Parse the data in the ring buffer
      ParseData();
    }
    cyber::USleep(1000);
  }
}

bool ROSControl::ProcessHeader() {
  uint8_t header;
  if (ring_buffer_.Read(&header, 1) == 1) {
    ADEBUG << "[FRAME HEADER] 0x" << std::hex << static_cast<int>(header);

    if (header == 0x7B) {
      current_frame_type_ = FrameType::DATA1;
      data_payload_length_ = 22;
      state_ = ParseState::READ_DATA;
      current_frame_data_.push_back(header);
    } else if (header == 0xFA) {
      current_frame_type_ = FrameType::DATA2;
      data_payload_length_ = 17;
      state_ = ParseState::READ_DATA;
      current_frame_data_.push_back(header);
    } else {
      AERROR << "Invalid header: 0x" << std::hex << static_cast<int>(header)
             << " | Raw byte: " << HexToString(&header, 1);
      return false;
    }
    return true;
  }
  return false;
}

bool ROSControl::ProcessPayLoad() {
  size_t bytes_to_read = data_payload_length_;
  if (ring_buffer_.Count() >= bytes_to_read) {
    std::vector<uint8_t> pay_load(bytes_to_read);
    ring_buffer_.Read(pay_load.data(), bytes_to_read);
    current_frame_data_.insert(current_frame_data_.end(), pay_load.begin(),
                               pay_load.end());
    state_ = ParseState::SEEK_TAIL;
    return true;
  }
  return false;
}

bool ROSControl::ProcessTail() {
  uint8_t tail;
  if (ring_buffer_.Read(&tail, 1) == 1) {
    ADEBUG << "[FRAME TAIL] 0x" << std::hex << static_cast<int>(tail);

    bool tail_valid = false;
    if (current_frame_type_ == FrameType::DATA1 && tail == 0x7D) {
      tail_valid = true;
    } else if (current_frame_type_ == FrameType::DATA2 && tail == 0xFC) {
      tail_valid = true;
    }

    if (tail_valid) {
      current_frame_data_.push_back(tail);
      state_ = ParseState::PROCESS_FRAME;
      return true;
    } else {
      AERROR << "Tail mismatch! Expected: "
             << (current_frame_type_ == FrameType::DATA1 ? "0x7D" : "0xFC")
             << " Received: 0x" << std::hex << static_cast<int>(tail);
      return false;
    }
  }
  return false;
}

bool ROSControl::IsFrameChecksumValid(const std::vector<uint8_t>& frame_data) {
  uint8_t checksum_calculated = 0;
  for (size_t i = 0; i < frame_data.size() - 1; ++i) {
    checksum_calculated += frame_data[i];
  }

  const bool valid = (checksum_calculated == frame_data.back());
  if (!valid) {
    AERROR << "Checksum failed! Calculated: 0x" << std::hex
           << static_cast<int>(checksum_calculated) << " Received: 0x"
           << std::hex << static_cast<int>(frame_data.back())
           << "\nFull frame: "
           << HexToString(frame_data.data(), frame_data.size());
  }
  return valid;
}

bool ROSControl::ProcessFrame() {
  std::lock_guard<std::mutex> lock(mutex_);

  ADEBUG << "Processing frame : "
         << HexToString(current_frame_data_.data(), current_frame_data_.size());

  bool is_checksum_valid = IsFrameChecksumValid(current_frame_data_);
  if (!is_checksum_valid) {
    AERROR << "Frame checksum validation failed!";
  }

  size_t length = current_frame_data_.size();
  switch (current_frame_type_) {
    case FrameType::DATA1:
      ROSParser::DecodeTwistFb(current_frame_data_.data(), length, &chassis_);
      break;
    case FrameType::DATA2:
      ROSParser::DecodeMiscFb(current_frame_data_.data(), length, &chassis_);
      break;
    default:
      AERROR << "Unknown frame type: " << static_cast<int>(current_frame_type_);
      break;
  }

  // Default setting is COMPLETE_AUTO_DRIVE mode
  chassis_.set_driving_mode(Chassis::COMPLETE_AUTO_DRIVE);

  return is_checksum_valid;
}

void ROSControl::ResetFrameState() {
  state_ = ParseState::SEEK_HEADER;
  current_frame_data_.clear();
  current_frame_type_ = FrameType::UNKNOWN;
  data_payload_length_ = 0;
}

void ROSControl::ParseData() {
  while (ring_buffer_.Count() > 0) {
    switch (state_) {
      case ParseState::SEEK_HEADER: {
        if (!ProcessHeader()) {
          ResetFrameState();
        }
        break;
      }
      case ParseState::READ_DATA: {
        if (!ProcessPayLoad()) {
          ResetFrameState();
        }
        break;
      }
      case ParseState::SEEK_TAIL: {
        if (!ProcessTail()) {
          ResetFrameState();
        }
        break;
      }
      case ParseState::PROCESS_FRAME: {
        ProcessFrame();
        ResetFrameState();
        break;
      }
      default:
        state_ = ParseState::SEEK_HEADER;
        break;
    }
  }
}

}  // namespace serial
}  // namespace apollo
