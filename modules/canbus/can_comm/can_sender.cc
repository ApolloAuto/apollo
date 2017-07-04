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

#include "modules/canbus/can_comm/can_sender.h"

#include "modules/common/log.h"
#include "modules/common/time/time.h"

namespace apollo {
namespace canbus {

namespace {

using ::apollo::common::time::Clock;
using ::apollo::common::ErrorCode;
using micros = ::apollo::common::time::micros;

const uint32_t kSenderInterval = 6000;

}  // namespace
std::mutex SenderMessage::mutex_;

SenderMessage::SenderMessage(const uint32_t message_id,
                             ProtocolData* protocol_data)
    : SenderMessage(message_id, protocol_data, false) {}

SenderMessage::SenderMessage(const uint32_t message_id,
                             ProtocolData* protocol_data, bool init_with_one)
    : message_id_(message_id), protocol_data_(protocol_data) {
  if (init_with_one) {
    for (int32_t i = 0; i < protocol_data->GetLength(); ++i) {
      can_frame_to_update_.data[i] = 0xFF;
    }
  }
  int32_t len = protocol_data_->GetLength();

  can_frame_to_update_.id = message_id_;
  can_frame_to_update_.len = len;

  period_ = protocol_data_->GetPeriod();
  curr_period_ = period_;

  Update();
}

void SenderMessage::UpdateCurrPeriod(const int32_t period_delta) {
  curr_period_ -= period_delta;
  if (curr_period_ <= 0) {
    curr_period_ = period_;
  }
}

void SenderMessage::Update() {
  if (protocol_data_ == nullptr) {
    AERROR << "Attention: ProtocolData is nullptr!";
    return;
  }
  protocol_data_->UpdateData(can_frame_to_update_.data);

  std::lock_guard<std::mutex> lock(mutex_);
  can_frame_to_send_ = can_frame_to_update_;
}

uint32_t SenderMessage::message_id() const { return message_id_; }

struct CanFrame SenderMessage::CanFrame() {
  std::lock_guard<std::mutex> lock(mutex_);
  return can_frame_to_send_;
}

int32_t SenderMessage::curr_period() const { return curr_period_; }

void CanSender::PowerSendThreadFunc() {
  CHECK_NOTNULL(can_client_);

  const int32_t INIT_PERIOD = 5000;  // 5ms
  int32_t delta_period = INIT_PERIOD;
  int32_t new_delta_period = INIT_PERIOD;

  int64_t tm_start = 0;
  int64_t tm_end = 0;
  int64_t sleep_interval = 0;

  AINFO << "Can client sender thread starts.";

  while (is_running_) {
    tm_start = ::apollo::common::time::AsInt64<micros>(Clock::Now());
    new_delta_period = INIT_PERIOD;

    for (auto& message : send_messages_) {
      bool need_send = NeedSend(message, delta_period);
      message.UpdateCurrPeriod(delta_period);
      new_delta_period = std::min(new_delta_period, message.curr_period());

      if (!need_send) {
        continue;
      }
      std::vector<CanFrame> can_frames;
      CanFrame can_frame = message.CanFrame();
      can_frames.push_back(can_frame);
      if (can_client_->SendSingleFrame(can_frames) != ErrorCode::OK) {
        AERROR << "Send msg failed:" << can_frame.CanFrameString();
      }
      if (enable_log()) {
        ADEBUG << "send_can_frame#" << can_frame.CanFrameString();
      }
    }
    delta_period = new_delta_period;
    tm_end = ::apollo::common::time::AsInt64<micros>(Clock::Now());
    sleep_interval = delta_period - (tm_end - tm_start);

    if (sleep_interval > 0) {
      std::this_thread::sleep_for(std::chrono::microseconds(sleep_interval));
    } else {
      // do not sleep
      AWARN << "Too much time for calculation: " << tm_end - tm_start
            << "us is more than minimum period: " << delta_period << "us";
    }
  }
  AINFO << "Can client sender thread stopped!";
}

ErrorCode CanSender::Init(CanClient* can_client, bool enable_log) {
  if (is_init_) {
    AERROR << "Duplicated Init request.";
    return ErrorCode::CANBUS_ERROR;
  }
  if (can_client == nullptr) {
    AERROR << "Invalid can client.";
    return ErrorCode::CANBUS_ERROR;
  }
  is_init_ = true;
  can_client_ = can_client;
  enable_log_ = enable_log;
  return ErrorCode::OK;
}

void CanSender::AddMessage(uint32_t message_id, ProtocolData* protocol_data,
                           bool init_with_one) {
  if (protocol_data == nullptr) {
    AERROR << "invalid protocol data.";
    return;
  }
  send_messages_.emplace_back(
      SenderMessage(message_id, protocol_data, init_with_one));
  AINFO << "Add send message:" << std::hex << message_id;
}

ErrorCode CanSender::Start() {
  if (is_running_) {
    AERROR << "Cansender has already started.";
    return ErrorCode::CANBUS_ERROR;
  }
  is_running_ = true;
  thread_.reset(new std::thread([this] { PowerSendThreadFunc(); }));

  return ErrorCode::OK;
}

void CanSender::Update() {
  for (auto& message : send_messages_) {
    message.Update();
  }
}

void CanSender::Stop() {
  if (is_running_) {
    AINFO << "Stopping can sender ...";
    is_running_ = false;
    if (thread_ != nullptr && thread_->joinable()) {
      thread_->join();
    }
    thread_.reset();
  } else {
    AERROR << "CanSender is not running.";
  }

  AINFO << "Can client sender stopped [ok].";
}

bool CanSender::IsRunning() const { return is_running_; }

bool CanSender::enable_log() const { return enable_log_; }

bool CanSender::NeedSend(const SenderMessage& msg, const int32_t delta_period) {
  return msg.curr_period() <= delta_period;
}

}  // namespace canbus
}  // namespace apollo
