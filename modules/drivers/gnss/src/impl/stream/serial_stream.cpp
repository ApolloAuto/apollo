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

#include <errno.h>
#include <fcntl.h>
#include <linux/netlink.h>
#include <linux/serial.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include <thread>

#include <ros/ros.h>

#include "gnss/stream.h"

namespace apollo {
namespace drivers {
namespace gnss {

speed_t get_serial_baudrate(uint32_t rate) {
  switch (rate) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    case 921600:
      return B921600;
    default:
      return 0;
  }
}

class SerialStream : public Stream {
 public:
  SerialStream(const char* device_name, speed_t baud_rate,
               uint32_t timeout_usec);
  ~SerialStream();

  virtual bool connect();
  virtual bool disconnect();
  virtual size_t read(uint8_t* buffer, size_t max_length);
  virtual size_t write(const uint8_t* data, size_t length);

 private:
  SerialStream() {}
  void open();
  void close();
  bool configure_port(int fd);
  bool wait_readable(uint32_t timeout_us);
  bool wait_writable(uint32_t timeout_us);
  void check_remove();

  std::string _device_name;
  speed_t _baud_rate;
  uint32_t _bytesize;
  uint32_t _parity;
  uint32_t _stopbits;
  uint32_t _flowcontrol;
  uint32_t _byte_time_us;

  uint32_t _timeout_usec;
  int _fd;
  int _errno;
  bool _is_open;
};

SerialStream::SerialStream(const char* device_name, speed_t baud_rate,
                           uint32_t timeout_usec)
    : _device_name(device_name),
      _baud_rate(baud_rate),
      _bytesize(8),
      _parity(0),
      _stopbits(1),
      _flowcontrol(0),
      _timeout_usec(timeout_usec),
      _fd(-1),
      _errno(0),
      _is_open(false) {
  if (_device_name.empty()) {
    _status = Stream::Status::ERROR;
  }
}

SerialStream::~SerialStream() { this->close(); }

void SerialStream::open(void) {
  int fd = 0;
  fd = ::open(_device_name.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd == -1) {
    switch (errno) {
      case EINTR:
        // Recurse because this is a recoverable error.
        return open();

      case ENFILE:
      case EMFILE:
      default:
        ROS_ERROR_STREAM_THROTTLE(
            2, "Open device " << _device_name
                              << " failed, error: " << strerror(errno));
        return;
    }
  }

  if (!configure_port(fd)) {
    return;
  }

  _fd = fd;
  _is_open = true;
}

bool SerialStream::configure_port(int fd) {
  if (fd < 0) {
    return false;
  }

  struct termios options;  // The options for the file descriptor
  if (tcgetattr(fd, &options) == -1) {
    ROS_ERROR("tcgetattr failed.");
    return false;
  }

  // set up raw mode / no echo / binary
  options.c_cflag |= (tcflag_t)(CLOCAL | CREAD);
  options.c_lflag &= (tcflag_t) ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL |
                                  ISIG | IEXTEN);  //|ECHOPRT

  options.c_oflag &= (tcflag_t) ~(OPOST);
  options.c_iflag &= (tcflag_t) ~(INLCR | IGNCR | ICRNL | IGNBRK);

#ifdef IUCLC
  options.c_iflag &= (tcflag_t)~IUCLC;
#endif

#ifdef PARMRK
  options.c_iflag &= (tcflag_t)~PARMRK;
#endif

#ifdef _BSD_SOURCE  // depend glibc
  ::cfsetspeed(&options, _baud_rate);
#else
  ::cfsetispeed(&options, _baud_rate);
  ::cfsetospeed(&options, _baud_rate);
#endif

  // setup char len
  options.c_cflag &= (tcflag_t)~CSIZE;

  // eightbits
  options.c_cflag |= CS8;

  // setup stopbits:stopbits_one
  options.c_cflag &= (tcflag_t) ~(CSTOPB);

  // setup parity: parity_none
  options.c_iflag &= (tcflag_t) ~(INPCK | ISTRIP);
  options.c_cflag &= (tcflag_t) ~(PARENB | PARODD);

// setup flow control : flowcontrol_none
// xonxoff
#ifdef IXANY
  options.c_iflag &= (tcflag_t) ~(IXON | IXOFF | IXANY);
#else
  options.c_iflag &= (tcflag_t) ~(IXON | IXOFF);
#endif

// rtscts
#ifdef CRTSCTS
  options.c_cflag &= (unsigned long)~(CRTSCTS);
#elif defined CNEW_RTSCTS
  options.c_cflag &= (unsigned long)~(CNEW_RTSCTS);
#else
#error "OS Support seems wrong."
#endif

  // http://www.unixwiz.net/techtips/termios-vmin-vtime.html
  // this basically sets the read call up to be a polling read,
  // but we are using select to ensure there is data available
  // to read before each call, so we should never needlessly poll
  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 0;

  // activate settings
  ::tcsetattr(fd, TCSANOW, &options);

  // Update byte_time_ based on the new settings.
  uint32_t bit_time_us = 1e6 / 115200;
  _byte_time_us = bit_time_us * (1 + _bytesize + _parity + _stopbits);
  return true;
}

bool SerialStream::connect() {
  if (!_is_open) {
    this->open();
    if (!_is_open) {
      _status = Stream::Status::ERROR;
      _errno = errno;
      return false;
    }
  }

  if (_status == Stream::Status::CONNECTED) {
    return true;
  }

  _status = Stream::Status::CONNECTED;
  login();
  return true;
}

void SerialStream::close(void) {
  if (_is_open) {
    ::close(_fd);
    _fd = -1;
    _is_open = false;
    _status = Stream::Status::DISCONNECTED;
  }
}

bool SerialStream::disconnect() {
  if (_is_open == false) {
    // not open
    return false;
  }

  this->close();
  return true;
}

void SerialStream::check_remove() {
  char data = 0;
  ssize_t nsent = ::write(_fd, &data, 0);
  if (nsent < 0) {
    ROS_ERROR_STREAM(
        "Serial stream detect write failed, error: " << strerror(errno));
    switch (errno) {
      case EBADF:
      case EIO:
        _status = Stream::Status::DISCONNECTED;
        ROS_ERROR_STREAM("Device " << _device_name << " removed.");
        disconnect();
        break;
    }
  }
}

size_t SerialStream::read(uint8_t* buffer, size_t max_length) {
  if (!_is_open) {
    if (!connect()) {
      return 0;
    }
    ROS_INFO_STREAM("Connect " << _device_name << " success.");
  }

  ssize_t bytes_read = 0;
  ssize_t bytes_current_read = 0;

  wait_readable(10000);  // wait 10ms

  while (max_length > 0) {
    bytes_current_read = ::read(_fd, buffer, max_length);
    if (bytes_current_read < 0) {
      switch (errno) {
        case EAGAIN:
        case EINVAL:
          bytes_current_read = 0;
          break;

        case EBADF:
        case EIO:
          ROS_ERROR_STREAM(
              "Serial stream read data failed, error: " << strerror(errno));
          disconnect();
          if (connect()) {
            ROS_INFO_STREAM("Reconnect " << _device_name << " success.");
            bytes_current_read = 0;
            break;  // has recoverable
          }

        default:
          ROS_ERROR_STREAM_THROTTLE(1, "Serial stream read data failed, error: "
                                           << strerror(errno)
                                           << ", errno: " << errno);
          _status = Stream::Status::ERROR;
          _errno = errno;
          return bytes_read;
      }
    }

    if (bytes_current_read == 0) {
      if (!bytes_read) {
        check_remove();
        return 0;
      }
      return bytes_read;
    }
    max_length -= bytes_current_read;
    buffer += bytes_current_read;
    bytes_read += bytes_current_read;
  }

  return bytes_read;
}

size_t SerialStream::write(const uint8_t* data, size_t length) {
  if (!_is_open) {
    if (!connect()) {
      return 0;
    }
    ROS_INFO_STREAM("Connect " << _device_name << " success.");
  }

  size_t total_nsent = 0;
  size_t delay_times = 0;

  while ((length > 0) && (delay_times < 5)) {
    ssize_t nsent = ::write(_fd, data, length);
    if (nsent < 0) {
      ROS_ERROR_STREAM(
          "Serial stream write data failed, error: " << strerror(errno));
      switch (errno) {
        case EAGAIN:
        case EINVAL:
          nsent = 0;
          break;

        case EBADF:
        case EIO:
          disconnect();
          if (connect()) {
            ROS_INFO_STREAM("Reconnect " << _device_name << "success.");
            nsent = 0;
            break;  // has recoverable
          }

        default:
          _status = Stream::Status::ERROR;
          _errno = errno;
          return total_nsent;
      }
    }

    if (nsent == 0) {
      if (!wait_writable(_byte_time_us)) {
        break;
      }
      ++delay_times;
      continue;
    }

    total_nsent += nsent;
    length -= nsent;
    data += nsent;
  }

  return total_nsent;
}

bool SerialStream::wait_readable(uint32_t timeout_us) {
  // Setup a select call to block for serial data or a timeout
  timespec timeout_ts;
  fd_set readfds;
  FD_ZERO(&readfds);
  FD_SET(_fd, &readfds);

  timeout_ts.tv_sec = timeout_us / 1000000;
  timeout_ts.tv_nsec = (timeout_us % 1000000) * 1000;
  int r = pselect(_fd + 1, &readfds, NULL, NULL, &timeout_ts, NULL);
  if (r <= 0) {
    return false;
  }

  // This shouldn't happen, if r > 0 our fd has to be in the list!
  if (!FD_ISSET(_fd, &readfds)) {
    return false;
  }
  // Data available to read.
  return true;
}

bool SerialStream::wait_writable(uint32_t timeout_us) {
  // Setup a select call to block for serial data or a timeout
  timespec timeout_ts;
  fd_set writefds;
  FD_ZERO(&writefds);
  FD_SET(_fd, &writefds);

  timeout_ts.tv_sec = timeout_us / 1000000;
  timeout_ts.tv_nsec = (timeout_us % 1000000) * 1000;
  int r = pselect(_fd + 1, NULL, &writefds, NULL, &timeout_ts, NULL);
  if (r <= 0) {
    return false;
  }

  // This shouldn't happen, if r > 0 our fd has to be in the list!
  if (!FD_ISSET(_fd, &writefds)) {
    return false;
  }
  // Data available to write.
  return true;
}

Stream* Stream::create_serial(const char* device_name, uint32_t baud_rate,
                              uint32_t timeout_usec) {
  speed_t baud = get_serial_baudrate(baud_rate);
  return baud == 0 ? nullptr
                   : new SerialStream(device_name, baud, timeout_usec);
}

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
