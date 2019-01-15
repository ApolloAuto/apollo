/******************************************************************************
 * Copyright 2019 The CiDi Authors. All Rights Reserved.
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

#include <string.h>
#include <cmath>
#include <ctime>
#include <memory>
#include <thread>
#include <vector>

#include "cyber/cyber.h"

#include "modules/drivers/cidiv2x/stream/proto/config.pb.h"
#include "modules/drivers/cidiv2x/stream/raw_stream.h"
#include "modules/drivers/cidiv2x/stream/stream.h"
#include "modules/drivers/cidiv2x/stream/util/utils.h"

namespace apollo {
namespace drivers {
namespace cidiv2x {

void switch_stream_status(
    const apollo::drivers::cidiv2x::Stream::Status &status,
    StreamStatus_Type *report_status_type) {
  switch (status) {
    case apollo::drivers::cidiv2x::Stream::Status::CONNECTED:
      *report_status_type = StreamStatus::CONNECTED;
      break;

    case apollo::drivers::cidiv2x::Stream::Status::DISCONNECTED:
      *report_status_type = StreamStatus::DISCONNECTED;
      break;

    case apollo::drivers::cidiv2x::Stream::Status::ERROR:
    default:
      *report_status_type = StreamStatus::DISCONNECTED;
      break;
  }
}

Stream *create_stream(const config::Stream &sd) {
  switch (sd.type_case()) {
    case config::Stream::kSerial:
      if (!sd.serial().has_device()) {
        AERROR << "Serial def has no device field.";
        return nullptr;
      }
      if (!sd.serial().has_baud_rate()) {
        AERROR << "Serial def has no baud_rate field. Use default baud rate "
               << sd.serial().baud_rate();
        return nullptr;
      }
      return Stream::create_serial(sd.serial().device().c_str(),
                                   sd.serial().baud_rate());

    case config::Stream::kTcp:
      if (!sd.tcp().has_address()) {
        AERROR << "tcp def has no address field.";
        return nullptr;
      }
      if (!sd.tcp().has_port()) {
        AERROR << "tcp def has no port field.";
        return nullptr;
      }
      return Stream::create_tcp(sd.tcp().address().c_str(),
                                static_cast<uint16_t>(sd.tcp().port()));

    case config::Stream::kUdp:
      if (!sd.udp().has_address()) {
        AERROR << "udp def has no address field.";
        return nullptr;
      }
      if (!sd.udp().has_port()) {
        AERROR << "udp def has no port field.";
        return nullptr;
      }
      if (!sd.udp().has_broadcast_port()) {
        AERROR << "udp def has no broadcast port field.";
        return nullptr;
      }
      AINFO << "UDP Addr: " << sd.udp().address().c_str()
            << ", port: " << sd.udp().port()
            << ", broadcast port: " << sd.udp().broadcast_port();
      return Stream::create_udp(
          sd.udp().address().c_str(), static_cast<uint16_t>(sd.udp().port()),
          static_cast<uint16_t>(sd.udp().broadcast_port()));
    default:
      return nullptr;
  }
}

RawStream::RawStream(const config::Config &config,
                     const std::shared_ptr<apollo::cyber::Node> &node)
    : config_(config), node_(node) {
  data_parser_ptr_.reset(new CidiV2xParser(config_, node_));
}

RawStream::~RawStream() {
  this->Logout();
  this->Disconnect();
}

bool RawStream::Init() {
  CHECK_NOTNULL(data_parser_ptr_);
  if (!data_parser_ptr_->Init()) {
    AERROR << "Init data parser failed.";
    return false;
  }

  stream_status_.set_stream_type(StreamStatus::DISCONNECTED);
  // Creates streams.
  Stream *s = nullptr;
  if (!config_.has_data()) {
    AINFO << "Error: Config file must provide the data stream.";
    return false;
  }
  s = create_stream(config_.data());
  if (s == nullptr) {
    AERROR << "Failed to create data stream.";
    return false;
  }
  data_stream_.reset(s);

  Status *status = new Status();
  if (!status) {
    AERROR << "Failed to create data stream status.";
    return false;
  }
  data_stream_status_.reset(status);

  if (config_.has_command()) {
    s = create_stream(config_.command());
    if (s == nullptr) {
      AERROR << "Failed to create command stream.";
      return false;
    }
    command_stream_.reset(s);

    status = new Status();
    if (!status) {
      AERROR << "Failed to create command stream status.";
      return false;
    }
    command_stream_status_.reset(status);
  } else {
    command_stream_ = data_stream_;
    command_stream_status_ = data_stream_status_;
  }

  if (config_.login_commands_size() == 0) {
    AWARN << "No login_commands in config file.";
  }

  if (config_.logout_commands_size() == 0) {
    AWARN << "No logout_commands in config file.";
  }

  // connect and login
  if (!Connect()) {
    AERROR << "stream driver connect failed.";
    return false;
  }

  if (!Login()) {
    AERROR << "stream driver login failed.";
    return false;
  }

  return true;
}

void RawStream::Start() {
  is_running_ = true;
  data_thread_ptr_.reset(new std::thread(&RawStream::DataSpin, this));
  if (data_thread_ptr_ == nullptr) {
    AERROR << "Unable to create rawstream data thread.";
    return;
  }
  // heart_thread_ptr_.reset(new std::thread(&RawStream::HeartSpin, this));
}

bool RawStream::Connect() {
  if (data_stream_) {
    if (data_stream_->get_status() != Stream::Status::CONNECTED) {
      if (!data_stream_->Connect()) {
        AERROR << "data stream connect failed.";
        return false;
      }
      data_stream_status_->status = Stream::Status::CONNECTED;
    }
  }

  if (command_stream_) {
    if (command_stream_->get_status() != Stream::Status::CONNECTED) {
      if (!command_stream_->Connect()) {
        AERROR << "command stream connect failed.";
        return false;
      }
      command_stream_status_->status = Stream::Status::CONNECTED;
    }
  }

  return true;
}

bool RawStream::Disconnect() {
  if (data_stream_) {
    if (data_stream_->get_status() == Stream::Status::CONNECTED) {
      if (!data_stream_->Disconnect()) {
        AERROR << "data stream disconnect failed.";
        return false;
      }
    }
  }

  if (command_stream_) {
    if (command_stream_->get_status() == Stream::Status::CONNECTED) {
      if (!command_stream_->Disconnect()) {
        AERROR << "command stream disconnect failed.";
        return false;
      }
    }
  }

  return true;
}

bool RawStream::Login() {
  std::vector<std::string> login_data;
  for (const auto &login_command : config_.login_commands()) {
    data_stream_->write(login_command);
    login_data.emplace_back(login_command);
    AINFO << "Login command: " << login_command;
    // sleep a little to avoid overrun of the slow serial interface.
    cyber::Duration(0.5).Sleep();
  }
  data_stream_->RegisterLoginData(login_data);

  return true;
}

bool RawStream::Logout() {
  for (const auto &logout_command : config_.logout_commands()) {
    data_stream_->write(logout_command);
    AINFO << "Logout command: " << logout_command;
  }
  return true;
}

void RawStream::StreamStatusCheck() {
  StreamStatus_Type report_stream_status;

  if (data_stream_ &&
      (data_stream_->get_status() != data_stream_status_->status)) {
    data_stream_status_->status = data_stream_->get_status();
    switch_stream_status(data_stream_status_->status, &report_stream_status);
    stream_status_.set_stream_type(report_stream_status);
    ADEBUG << "stream status: "
           << stream_status_.ShortDebugString();
  }
}

void RawStream::DataSpin() {
  static uint8_t heartBeatCount = 0;
  char buf[BUFFER_SIZE] = "apollo";
  std::chrono::duration<double, std::micro> default_period{100 * 1000};
  while (isRunning()) {
    size_t length = 0;
    ADEBUG << "is healthy: " << is_healthy_;
    memset(buffer_, 0, BUFFER_SIZE);
    if (!is_healthy_) {
      length = data_stream_->read(buffer_, BUFFER_SIZE, 1);
      if (length == 320) {
        std::string tempStr = std::string(reinterpret_cast<char *>(buffer_));
        ADEBUG << "Received data: " << tempStr;
        is_healthy_ = true;
        char buf[BUFFER_SIZE] = "apollo";
        size_t len = data_stream_->write(reinterpret_cast<uint8_t *>(buf),
                                         BUFFER_SIZE, 0);
        if (len > 0) {
          AINFO << "success sento to broadcaster: " << buf;

        } else {
          AINFO << "Failed send to broadcaster.";
        }
      } else {
        is_healthy_ = false;
        memset(buffer_, 0, BUFFER_SIZE);
        std::this_thread::sleep_for(default_period);
        continue;
      }
    } else {
      length = data_stream_->read(buffer_, BUFFER_SIZE, 0);
      if (length == 320) {
        std::string tempStr = std::string(reinterpret_cast<char *>(buffer_));
        ADEBUG << "Received data: " << tempStr;
        // memset(buffer_, 0, BUFFER_SIZE);
      } else {
        // Todo(inumo): set timeout to switch to broadcast mode
        is_healthy_ = false;
      }
    }
    ADEBUG << "receive length: " << length;
    if (length == 320) {
      std::shared_ptr<RawData> msg_pub = std::make_shared<RawData>();
      if (!msg_pub) {
        AERROR << "New data sting msg failed.";
        continue;
      }
      msg_pub->set_data(reinterpret_cast<const char *>(buffer_), length);
      ADEBUG << "Call data parser";
      data_parser_ptr_->ParseRawData(msg_pub->data());
    }

    if (is_healthy_) {
      heartBeatCount++;
      if (heartBeatCount == 10) {
        heartBeatCount = 0;
        data_stream_->write(reinterpret_cast<uint8_t *>(buf), BUFFER_SIZE, 0);
      }
    } else {
      heartBeatCount = 0;
    }

    StreamStatusCheck();
    std::this_thread::yield();
  }
  AINFO << "Cidiv2x rawstream thread stopped.";
}

// void RawStream::HeartSpin() {
//   // heart beat msg
//   char buf[BUFFER_SIZE] = "apollo";
//   while(ros::ok()) {
//     if(is_healthy_) {
//       // send Heart beat
//       data_stream_->write((uint8_t *)buf, BUFFER_SIZE, 0);
//     }
//     sleep(1);
//   }
// }
bool RawStream::isRunning() const { return is_running_; }
void RawStream::Stop() {
  if (isRunning()) {
    is_running_ = false;
    if (data_thread_ptr_ != nullptr && data_thread_ptr_->joinable()) {
      data_thread_ptr_->join();
    }
    data_thread_ptr_.reset();
    // heart_thread_ptr_.reset();
  } else {
    AINFO << "Raw stream is not running.";
  }
  AINFO << "Raw stream stopped [ok].";
}

}  // namespace cidiv2x
}  // namespace drivers
}  // namespace apollo
