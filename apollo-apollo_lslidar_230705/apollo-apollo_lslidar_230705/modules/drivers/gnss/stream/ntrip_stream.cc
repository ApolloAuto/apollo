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

#include <unistd.h>
#include <iostream>
#include <mutex>

#include "cyber/cyber.h"

#include "modules/common/util/string_util.h"
#include "modules/common/util/util.h"
#include "modules/drivers/gnss/stream/stream.h"
#include "modules/drivers/gnss/stream/tcp_stream.h"

namespace {

template <typename T>
constexpr bool is_zero(T value) {
  return value == static_cast<T>(0);
}
}  // namespace

namespace apollo {
namespace drivers {
namespace gnss {

class NtripStream : public Stream {
 public:
  NtripStream(const std::string& address, uint16_t port,
              const std::string& mountpoint, const std::string& user,
              const std::string& passwd, uint32_t timeout_s);
  ~NtripStream();

  virtual size_t read(uint8_t* buffer, size_t max_length);
  virtual size_t write(const uint8_t* data, size_t length);
  virtual bool Connect();
  virtual bool Disconnect();

 private:
  void Reconnect();
  bool is_login_ = false;
  const std::string mountpoint_;
  const std::string write_data_prefix_;
  const std::string login_data_;
  double timeout_s_ = 60.0;
  double data_active_s_ = 0.0;
  std::unique_ptr<TcpStream> tcp_stream_;
  std::mutex internal_mutex_;
};

NtripStream::NtripStream(const std::string& address, uint16_t port,
                         const std::string& mountpoint, const std::string& user,
                         const std::string& passwd, uint32_t timeout_s)
    : mountpoint_(mountpoint),
      write_data_prefix_("GET /" + mountpoint +
                         " HTTP/1.0\r\n"
                         "User-Agent: NTRIP gnss_driver/0.0\r\n"
                         "accept: */* \r\n\r\n"),

      login_data_("GET /" + mountpoint +
                  " HTTP/1.0\r\n"
                  "User-Agent: NTRIP gnss_driver/0.0\r\n"
                  "accept: */* \r\n"
                  "Authorization: Basic " +
                  common::util::EncodeBase64(user + ":" + passwd) + "\r\n\r\n"),
      timeout_s_(timeout_s),
      tcp_stream_(new TcpStream(address.c_str(), port, 0, false)) {}

NtripStream::~NtripStream() { this->Disconnect(); }

bool NtripStream::Connect() {
  if (is_login_) {
    return true;
  }
  if (!tcp_stream_) {
    AERROR << "New tcp stream failed.";
    return true;
  }

  if (!tcp_stream_->Connect()) {
    status_ = Stream::Status::DISCONNECTED;
    AERROR << "Tcp connect failed.";
    return false;
  }

  uint8_t buffer[2048];
  size_t size = 0;
  size_t try_times = 0;

  size = tcp_stream_->write(
      reinterpret_cast<const uint8_t*>(login_data_.data()), login_data_.size());
  if (size != login_data_.size()) {
    tcp_stream_->Disconnect();
    status_ = Stream::Status::ERROR;
    AERROR << "Send ntrip request failed.";
    return false;
  }

  bzero(buffer, sizeof(buffer));
  AINFO << "Read ntrip response.";
  size = tcp_stream_->read(buffer, sizeof(buffer) - 1);
  while ((size == 0) && (try_times < 3)) {
    sleep(1);
    size = tcp_stream_->read(buffer, sizeof(buffer) - 1);
    ++try_times;
  }

  if (!size) {
    tcp_stream_->Disconnect();
    status_ = Stream::Status::DISCONNECTED;
    AERROR << "No response from ntripcaster.";
    return false;
  }

  if (std::strstr(reinterpret_cast<char*>(buffer), "ICY 200 OK\r\n")) {
    status_ = Stream::Status::CONNECTED;
    is_login_ = true;
    AINFO << "Ntrip login successfully.";
    return true;
  }

  if (std::strstr(reinterpret_cast<char*>(buffer), "SOURCETABLE 200 OK\r\n")) {
    AERROR << "Mountpoint " << mountpoint_ << " not exist.";
  }

  if (std::strstr(reinterpret_cast<char*>(buffer), "HTTP/")) {
    AERROR << "Authentication failed.";
  }

  AINFO << "No expect data.";
  AINFO << "Recv data length: " << size;
  // AINFO << "Data from server: " << reinterpret_cast<char*>(buffer);

  tcp_stream_->Disconnect();
  status_ = Stream::Status::ERROR;
  return false;
}

bool NtripStream::Disconnect() {
  if (is_login_) {
    bool ret = tcp_stream_->Disconnect();
    if (!ret) {
      return false;
    }
    status_ = Stream::Status::DISCONNECTED;
    is_login_ = false;
  }

  return true;
}

void NtripStream::Reconnect() {
  AINFO << "Reconnect ntrip caster.";
  std::unique_lock<std::mutex> lock(internal_mutex_);
  Disconnect();
  Connect();
  if (status_ != Stream::Status::CONNECTED) {
    AINFO << "Reconnect ntrip caster failed.";
    return;
  }

  data_active_s_ = cyber::Time::Now().ToSecond();
  AINFO << "Reconnect ntrip caster success.";
}

size_t NtripStream::read(uint8_t* buffer, size_t max_length) {
  if (!tcp_stream_) {
    return 0;
  }

  size_t ret = 0;

  if (tcp_stream_->get_status() != Stream::Status::CONNECTED) {
    Reconnect();
    if (status_ != Stream::Status::CONNECTED) {
      return 0;
    }
  }

  if (is_zero(data_active_s_)) {
    data_active_s_ = cyber::Time::Now().ToSecond();
  }

  ret = tcp_stream_->read(buffer, max_length);
  if (ret) {
    data_active_s_ = cyber::Time::Now().ToSecond();
  }

  // timeout detect
  if ((cyber::Time::Now().ToSecond() - data_active_s_) > timeout_s_) {
    AINFO << "Ntrip timeout.";
    Reconnect();
  }

  return ret;
}

size_t NtripStream::write(const uint8_t* buffer, size_t length) {
  if (!tcp_stream_) {
    return 0;
  }
  std::unique_lock<std::mutex> lock(internal_mutex_, std::defer_lock);
  if (!lock.try_lock()) {
    AINFO << "Try lock failed.";
    return 0;
  }

  if (tcp_stream_->get_status() != Stream::Status::CONNECTED) {
    return 0;
  }

  std::string data(reinterpret_cast<const char*>(buffer), length);
  data = write_data_prefix_ + data;
  size_t ret = tcp_stream_->write(reinterpret_cast<const uint8_t*>(data.data()),
                                  data.size());
  if (ret != data.size()) {
    AERROR << "Send ntrip data size " << data.size() << ", return " << ret;
    status_ = Stream::Status::ERROR;
    return 0;
  }

  return length;
}

Stream* Stream::create_ntrip(const std::string& address, uint16_t port,
                             const std::string& mountpoint,
                             const std::string& user, const std::string& passwd,
                             uint32_t timeout_s) {
  return new NtripStream(address, port, mountpoint, user, passwd, timeout_s);
}

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
