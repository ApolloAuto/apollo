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

#include <ros/ros.h>
#include <std_msgs/String.h>

#include "gnss/stream.h"
#include "util/utils.h"
#include "tcp_stream.h"

namespace {

template <typename T>
constexpr bool is_zero(T value) {
  return value == static_cast<T>(0);
}
}

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
  virtual bool connect();
  virtual bool disconnect();

 private:
  void reconnect();
  bool _is_login = false;
  const std::string _mountpoint;
  const std::string _write_data_prefix;
  const std::string _login_data;
  double _timeout_s = 60.0;
  double _data_active_s = 0.0;
  std::unique_ptr<TcpStream> _tcp_stream;
  std::mutex _internal_mutex;
};

NtripStream::NtripStream(const std::string& address, uint16_t port,
                         const std::string& mountpoint, const std::string& user,
                         const std::string& passwd, uint32_t timeout_s)
    : _mountpoint(mountpoint),
      _write_data_prefix("GET /" + mountpoint + " HTTP/1.0\r\n"
                    "User-Agent: NTRIP gnss_driver/0.0\r\n"
                    "accept: */* \r\n\r\n"),

      _login_data("GET /" + mountpoint +
                  " HTTP/1.0\r\n"
                  "User-Agent: NTRIP gnss_driver/0.0\r\n"
                  "accept: */* \r\n"
                  "Authorization: Basic " +
                  encode_base64(user + ":" + passwd) + "\r\n\r\n"),
      _timeout_s(timeout_s),
      _tcp_stream(new TcpStream(address.c_str(), port, 0, false)) {
}

NtripStream::~NtripStream() { this->disconnect(); }

bool NtripStream::connect() {
  if (_is_login) {
    return true;
  }
  if (!_tcp_stream) {
    ROS_ERROR("New tcp stream failed.");
    return true;
  }

  if (!_tcp_stream->connect()) {
    _status = Stream::Status::DISCONNECTED;
    ROS_ERROR("Tcp connect failed.");
    return false;
  }

  uint8_t buffer[2048];
  size_t size = 0;
  size_t try_times = 0;

  size = _tcp_stream->write(reinterpret_cast<const uint8_t*>(_login_data.data()),
               _login_data.size());
  if (size != _login_data.size()) {
    _tcp_stream->disconnect();
    _status = Stream::Status::ERROR;
    ROS_ERROR("Send ntrip request failed.");
    return false;
  }

  bzero(buffer, sizeof(buffer));
  ROS_INFO("Read ntrip response.");
  size = _tcp_stream->read(buffer, sizeof(buffer) - 1);
  while ((size == 0) && (try_times < 3)) {
    sleep(1);
    size = _tcp_stream->read(buffer, sizeof(buffer) - 1);
    ++try_times;
  }

  if (!size) {
    _tcp_stream->disconnect();
    _status = Stream::Status::DISCONNECTED;
    ROS_ERROR("No response from ntripcaster.");
    return false;
  }

  if (std::strstr(reinterpret_cast<char*>(buffer), "ICY 200 OK\r\n")) {
    _status = Stream::Status::CONNECTED;
    _is_login = true;
    ROS_INFO("Ntrip login successfully.");
    return true;
  }

  if (std::strstr(reinterpret_cast<char*>(buffer), "SOURCETABLE 200 OK\r\n")) {
    ROS_ERROR_STREAM("Mountpoint " << _mountpoint << " not exist.");
  }

  if (std::strstr(reinterpret_cast<char*>(buffer), "HTTP/")) {
    ROS_ERROR("Authentication failed.");
  }

  ROS_INFO_STREAM("No expect data.");
  ROS_INFO_STREAM("Recv data length: " << size);
  // ROS_INFO_STREAM("Data from server: " << reinterpret_cast<char*>(buffer));

  _tcp_stream->disconnect();
  _status = Stream::Status::ERROR;
  return false;
}

bool NtripStream::disconnect() {
  if (_is_login) {
    bool ret = _tcp_stream->disconnect();
    if (!ret) {
      return false;
    }
    _status = Stream::Status::DISCONNECTED;
    _is_login = false;
  }

  return true;
}

void NtripStream::reconnect() {
  ROS_INFO("Reconnect ntrip caster.");
  std::unique_lock<std::mutex> lock(_internal_mutex);
  disconnect();
  connect();
  if (_status != Stream::Status::CONNECTED) {
    ROS_INFO("Reconnect ntrip caster failed.");
    return;
  }

  _data_active_s = ros::Time::now().toSec();
  ROS_INFO("Reconnect ntrip caster success.");
}

size_t NtripStream::read(uint8_t* buffer, size_t max_length) {
  if (!_tcp_stream) {
    return 0;
  }

  size_t ret = 0;

  if (_tcp_stream->get_status() != Stream::Status::CONNECTED) {
    reconnect();
    if (_status != Stream::Status::CONNECTED) {
      return 0;
    }
  }

  if (is_zero(_data_active_s)) {
    _data_active_s = ros::Time::now().toSec();
  }

  ret = _tcp_stream->read(buffer, max_length);
  if (ret) {
    _data_active_s = ros::Time::now().toSec();
  }

  // timeout detect
  if ((ros::Time::now().toSec() - _data_active_s) > _timeout_s) {
    ROS_INFO("Ntrip timeout.");
    reconnect();
  }

  return ret;
}

size_t NtripStream::write(const uint8_t* buffer, size_t length) {
  if (!_tcp_stream) {
    return 0;
  }
  std::unique_lock<std::mutex> lock(_internal_mutex, std::defer_lock);
  if (!lock.try_lock()) {
    ROS_INFO("Try lock failed.");
    return 0;
  }

  if (_tcp_stream->get_status() != Stream::Status::CONNECTED) {
    return 0;
  }

  std::string data(reinterpret_cast<const char*>(buffer), length);
  data = _write_data_prefix + data;
  size_t ret = _tcp_stream->write(reinterpret_cast<const uint8_t*>(data.data()), data.size());
  if (ret != data.size()) {
    ROS_ERROR_STREAM("Send ntrip data size " << data.size()
                  << ", return " << ret);
    _status = Stream::Status::ERROR;
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
}
}
}
