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

#include <cmath>
#include <ctime>
#include <memory>
#include <thread>
#include <vector>

#include "ros/include/ros/ros.h"
#include "ros/include/std_msgs/String.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/drivers/gnss/gnss_gflags.h"
#include "modules/drivers/gnss/proto/config.pb.h"
#include "modules/drivers/gnss/stream/raw_stream.h"
#include "modules/drivers/gnss/stream/stream.h"
#include "modules/drivers/gnss/util/utils.h"

namespace apollo {
namespace drivers {
namespace gnss {

using apollo::common::adapter::AdapterManager;

void switch_stream_status(const apollo::drivers::gnss::Stream::Status &status,
                          StreamStatus_Type *report_status_type) {
  switch (status) {
    case apollo::drivers::gnss::Stream::Status::CONNECTED:
      *report_status_type = StreamStatus::CONNECTED;
      break;

    case apollo::drivers::gnss::Stream::Status::DISCONNECTED:
      *report_status_type = StreamStatus::DISCONNECTED;
      break;

    case apollo::drivers::gnss::Stream::Status::ERROR:
    default:
      *report_status_type = StreamStatus::DISCONNECTED;
      break;
  }
}
std::string getLocalTimeFileStr() {
  time_t it = std::time(0);
  char local_time_char[64];
  std::strftime(local_time_char, sizeof(local_time_char), "%Y%m%d_%H%M%S",
                localtime(&it));
  std::string local_time_str = local_time_char;
  CHECK(apollo::common::util::EnsureDirectory(FLAGS_gpsbin_folder))
      << "gbsbin folder : " << FLAGS_gpsbin_folder << " create fail";
  std::string local_time_file_str =
      FLAGS_gpsbin_folder + "/" + local_time_str + ".bin";
  return local_time_file_str;
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
      return Stream::create_tcp(sd.tcp().address().c_str(), sd.tcp().port());

    case config::Stream::kUdp:
      if (!sd.udp().has_address()) {
        AERROR << "tcp def has no address field.";
        return nullptr;
      }
      if (!sd.udp().has_port()) {
        AERROR << "tcp def has no port field.";
        return nullptr;
      }
      return Stream::create_udp(sd.udp().address().c_str(), sd.udp().port());

    case config::Stream::kNtrip:
      if (!sd.ntrip().has_address()) {
        AERROR << "ntrip def has no address field.";
        return nullptr;
      }
      if (!sd.ntrip().has_port()) {
        AERROR << "ntrip def has no port field.";
        return nullptr;
      }
      if (!sd.ntrip().has_mount_point()) {
        AERROR << "ntrip def has no mount point field.";
        return nullptr;
      }
      if (!sd.ntrip().has_user()) {
        AERROR << "ntrip def has no user field.";
        return nullptr;
      }
      if (!sd.ntrip().has_password()) {
        AERROR << "ntrip def has no passwd field.";
        return nullptr;
      }
      return Stream::create_ntrip(
          sd.ntrip().address(), sd.ntrip().port(), sd.ntrip().mount_point(),
          sd.ntrip().user(), sd.ntrip().password(), sd.ntrip().timeout_s());
    default:
      return nullptr;
  }
}

RawStream::RawStream(const config::Config &config) : config_(config) {
  data_parser_ptr_.reset(new DataParser(config_));
  rtcm_parser_ptr_.reset(new RtcmParser());
}

RawStream::~RawStream() {
  this->Logout();
  this->Disconnect();
  gpsbin_stream_->close();
}

bool RawStream::Init() {
  CHECK_NOTNULL(data_parser_ptr_);
  CHECK_NOTNULL(rtcm_parser_ptr_);
  if (!data_parser_ptr_->Init()) {
    AERROR << "Init data parser failed.";
    return false;
  }
  if (!rtcm_parser_ptr_->Init()) {
    AERROR << "Init rtcm parser failed.";
    return false;
  }
  stream_status_.set_ins_stream_type(StreamStatus::DISCONNECTED);
  stream_status_.set_rtk_stream_in_type(StreamStatus::DISCONNECTED);
  stream_status_.set_rtk_stream_out_type(StreamStatus::DISCONNECTED);
  AdapterManager::FillStreamStatusHeader(FLAGS_sensor_node_name,
                                         &stream_status_);
  AdapterManager::PublishStreamStatus(stream_status_);

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

  if (config_.has_rtk_from()) {
    s = create_stream(config_.rtk_from());
    if (s == nullptr) {
      AERROR << "Failed to create rtk_from stream.";
      return false;
    }
    in_rtk_stream_.reset(s);

    if (config_.rtk_from().has_push_location()) {
      push_location_ = config_.rtk_from().push_location();
    }

    status = new Status();
    if (!status) {
      AERROR << "Failed to create rtk_from stream status.";
      return false;
    }
    in_rtk_stream_status_.reset(status);

    if (config_.has_rtk_to()) {
      s = create_stream(config_.rtk_to());
      if (s == nullptr) {
        AERROR << "Failed to create rtk_to stream.";
        return false;
      }
      out_rtk_stream_.reset(s);

      status = new Status();
      if (!status) {
        AERROR << "Failed to create rtk_to stream status.";
        return false;
      }
      out_rtk_stream_status_.reset(status);
    } else {
      out_rtk_stream_ = data_stream_;
      out_rtk_stream_status_ = data_stream_status_;
    }

    if (config_.has_rtk_solution_type()) {
      if (config_.rtk_solution_type() ==
          config::Config::RTK_SOFTWARE_SOLUTION) {
        rtk_software_solution_ = true;
      }
    }
  }

  if (config_.login_commands_size() == 0) {
    AWARN << "No login_commands in config file.";
  }

  if (config_.logout_commands_size() == 0) {
    AWARN << "No logout_commands in config file.";
  }

  // connect and login
  if (!Connect()) {
    AERROR << "gnss driver connect failed.";
    return false;
  }

  if (!Login()) {
    AERROR << "gnss driver login failed.";
    return false;
  }

  const std::string gpsbin_file = getLocalTimeFileStr();
  gpsbin_stream_.reset(new std::ofstream(
      gpsbin_file, std::ios::app | std::ios::out | std::ios::binary));
  AdapterManager::AddGnssRawDataCallback(&RawStream::GpsbinCallback, this);
  return true;
}

void RawStream::Start() {
  data_thread_ptr_.reset(new std::thread(&RawStream::DataSpin, this));
  rtk_thread_ptr_.reset(new std::thread(&RawStream::RtkSpin, this));
  if (config_.has_wheel_parameters()) {
    wheel_velocity_timer_ = AdapterManager::CreateTimer(
        ros::Duration(1), &RawStream::OnWheelVelocityTimer, this);
  }
}

void RawStream::OnWheelVelocityTimer(const ros::TimerEvent &) {
  AdapterManager::Observe();
  if (AdapterManager::GetChassis()->Empty()) {
    AINFO << "No chassis message received";
    return;
  }
  auto chassis = AdapterManager::GetChassis()->GetLatestObservedPtr();
  auto latency_sec =
      ros::Time::now().toSec() - chassis->header().timestamp_sec();
  auto latency_ms = std::to_string(std::lround(latency_sec * 1000));
  auto speed_cmps = std::to_string(std::lround(chassis->speed_mps() * 100));
  auto cmd_wheelvelocity =
      "WHEELVELOCITY " + latency_ms + " 100 0 0 0 0 0 " + speed_cmps + "\r\n";
  AINFO << "Write command: " << cmd_wheelvelocity;
  command_stream_->write(cmd_wheelvelocity);
}

bool RawStream::Connect() {
  if (data_stream_) {
    if (data_stream_->get_status() != Stream::Status::CONNECTED) {
      if (!data_stream_->Connect()) {
        AERROR << "data stream connect failed.";
        return false;
      }
      data_stream_status_->status = Stream::Status::CONNECTED;
      stream_status_.set_ins_stream_type(StreamStatus::CONNECTED);
    }
  }

  if (command_stream_) {
    if (command_stream_->get_status() != Stream::Status::CONNECTED) {
      if (!data_stream_->Connect()) {
        AERROR << "command stream connect failed.";
        return false;
      }
      command_stream_status_->status = Stream::Status::CONNECTED;
    }
  }

  if (in_rtk_stream_) {
    if (in_rtk_stream_->get_status() != Stream::Status::CONNECTED) {
      if (!in_rtk_stream_->Connect()) {
        AERROR << "in rtk stream connect failed.";
      } else {
        in_rtk_stream_status_->status = Stream::Status::CONNECTED;
        stream_status_.set_rtk_stream_in_type(StreamStatus::CONNECTED);
      }
    }
  } else {
    stream_status_.set_rtk_stream_in_type(StreamStatus::CONNECTED);
  }

  if (out_rtk_stream_) {
    if (out_rtk_stream_->get_status() != Stream::Status::CONNECTED) {
      if (!out_rtk_stream_->Connect()) {
        AERROR << "out rtk stream connect failed.";
      } else {
        out_rtk_stream_status_->status = Stream::Status::CONNECTED;
        stream_status_.set_rtk_stream_out_type(StreamStatus::CONNECTED);
      }
    }
  } else {
    stream_status_.set_rtk_stream_out_type(StreamStatus::CONNECTED);
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
  if (in_rtk_stream_) {
    if (in_rtk_stream_->get_status() == Stream::Status::CONNECTED) {
      if (!in_rtk_stream_->Disconnect()) {
        AERROR << "in rtk stream disconnect failed.";
        return false;
      }
    }
  }
  if (out_rtk_stream_) {
    if (out_rtk_stream_->get_status() == Stream::Status::CONNECTED) {
      if (!out_rtk_stream_->Disconnect()) {
        AERROR << "out rtk stream disconnect failed.";
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
    ros::Duration(0.5).sleep();
  }
  data_stream_->RegisterLoginData(login_data);

  if (config_.has_wheel_parameters()) {
    AINFO << "Write command: " << config_.wheel_parameters();
    command_stream_->write(config_.wheel_parameters());
  }

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
  bool status_report = false;
  StreamStatus_Type report_stream_status;

  if (data_stream_ &&
      (data_stream_->get_status() != data_stream_status_->status)) {
    data_stream_status_->status = data_stream_->get_status();
    status_report = true;
    switch_stream_status(data_stream_status_->status, &report_stream_status);
    stream_status_.set_ins_stream_type(report_stream_status);
  }

  if (in_rtk_stream_ &&
      (in_rtk_stream_->get_status() != in_rtk_stream_status_->status)) {
    in_rtk_stream_status_->status = in_rtk_stream_->get_status();
    status_report = true;
    switch_stream_status(in_rtk_stream_status_->status, &report_stream_status);
    stream_status_.set_rtk_stream_in_type(report_stream_status);
  }

  if (out_rtk_stream_ &&
      (out_rtk_stream_->get_status() != out_rtk_stream_status_->status)) {
    out_rtk_stream_status_->status = out_rtk_stream_->get_status();
    status_report = true;
    switch_stream_status(out_rtk_stream_status_->status, &report_stream_status);
    stream_status_.set_rtk_stream_out_type(report_stream_status);
  }

  if (status_report) {
    AdapterManager::FillStreamStatusHeader(FLAGS_sensor_node_name,
                                           &stream_status_);
    AdapterManager::PublishStreamStatus(stream_status_);
  }
}

void RawStream::DataSpin() {
  AdapterManager::FillStreamStatusHeader(FLAGS_sensor_node_name,
                                         &stream_status_);
  AdapterManager::PublishStreamStatus(stream_status_);
  while (ros::ok()) {
    size_t length = data_stream_->read(buffer_, BUFFER_SIZE);
    if (length > 0) {
      std_msgs::StringPtr msg_pub(new std_msgs::String);
      if (!msg_pub) {
        AERROR << "New data sting msg failed.";
        continue;
      }
      msg_pub->data.assign(reinterpret_cast<const char *>(buffer_), length);
      AdapterManager::PublishGnssRawData(*msg_pub);  // for data recorder
      data_parser_ptr_->ParseRawData(msg_pub);
      if (push_location_) {
        PushGpgga(length);
      }
    }
    StreamStatusCheck();
  }
}

void RawStream::RtkSpin() {
  if (in_rtk_stream_ == nullptr) {
    return;
  }
  while (ros::ok()) {
    size_t length = in_rtk_stream_->read(buffer_rtk_, BUFFER_SIZE);
    if (length > 0) {
      if (rtk_software_solution_) {
        PublishRtkData(length);
      } else {
        PublishRtkData(length);
        if (out_rtk_stream_ == nullptr) {
          continue;
        }
        size_t ret = out_rtk_stream_->write(buffer_rtk_, length);
        if (ret != length) {
          AERROR << "Expect write out rtk stream bytes " << length
                 << " but got " << ret;
        }
      }
    }
  }
}

void RawStream::PublishRtkData(size_t length) {
  std_msgs::StringPtr rtkmsg(new std_msgs::String);
  CHECK_NOTNULL(rtkmsg);
  rtkmsg->data.assign(reinterpret_cast<const char *>(buffer_rtk_), length);
  AdapterManager::PublishRtcmData(*rtkmsg);
  rtcm_parser_ptr_->ParseRtcmData(rtkmsg);
}

void RawStream::PushGpgga(size_t length) {
  if (!in_rtk_stream_) {
    return;
  }

  char *gpgga = strstr(reinterpret_cast<char *>(buffer_), "$GPGGA");
  if (gpgga) {
    char *p = strchr(gpgga, '*');
    if (p) {
      p += 5;
      if (size_t(p - reinterpret_cast<char *>(buffer_)) <= length) {
        AINFO_EVERY(5) << "Push gpgga.";
        in_rtk_stream_->write(reinterpret_cast<uint8_t *>(gpgga),
                              reinterpret_cast<uint8_t *>(p) - buffer_);
      }
    }
  }
}

void RawStream::GpsbinSpin() {
  if (gpsbin_stream_ == nullptr) {
    return;
  }
  while (ros::ok()) {
    std_msgs::String raw_data;
    gpsbin_stream_->write(raw_data.data.c_str(), raw_data.data.size());
  }
}
void RawStream::GpsbinCallback(const std_msgs::String &raw_data) {
  if (gpsbin_stream_ == nullptr) {
    return;
  }
  gpsbin_stream_->write(raw_data.data.c_str(), raw_data.data.size());
}

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
