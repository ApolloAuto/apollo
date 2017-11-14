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

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <iostream>

#include <ros/ros.h>

#include "gnss/stream.h"
#include "tcp_stream.h"

namespace apollo {
namespace drivers {
namespace gnss {

TcpStream::TcpStream(const char* address, uint16_t port,
                     uint32_t timeout_usec, bool auto_reconnect)
    : _sockfd(-1),
      _errno(0),
      _auto_reconnect(auto_reconnect) {
  _peer_addr = inet_addr(address);
  _peer_port = htons(port);
  _timeout_usec = timeout_usec;
}

TcpStream::~TcpStream() { this->close(); }

void TcpStream::open() {
  int fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (fd < 0) {
    // error
    ROS_ERROR_STREAM("create socket failed, errno: " << errno << ", "
                                                     << strerror(errno));
    return;
  }

  _sockfd = fd;
}

bool TcpStream::init_socket() {
  if (_sockfd < 0) {
    return false;
  }

  // block or not block
  if (_timeout_usec != 0) {
    int flags = fcntl(_sockfd, F_GETFL, 0);
    if (flags == -1) {
      ::close(_sockfd);
      ROS_ERROR_STREAM("fcntl get flag failed, error: " << strerror(errno)
                                                        << ".");
      return false;
    }

    if (fcntl(_sockfd, F_SETFL, flags & ~O_NONBLOCK) == -1) {
      ::close(_sockfd);
      ROS_ERROR_STREAM("fcntl set block failed, error: " << strerror(errno)
                                                         << ".");
      return false;
    }

    timeval block_to = {_timeout_usec / 1000000, _timeout_usec % 1000000};
    if (setsockopt(_sockfd, SOL_SOCKET, SO_RCVTIMEO, &block_to,
                   sizeof(block_to)) < 0) {
      ::close(_sockfd);
      ROS_ERROR_STREAM("setsockopt set rcv timeout failed, error: "
                       << strerror(errno) << ".");
      return false;
    }

    if (setsockopt(_sockfd, SOL_SOCKET, SO_SNDTIMEO, &block_to,
                   sizeof(block_to)) < 0) {
      ::close(_sockfd);
      ROS_ERROR_STREAM("setsockopt set snd timeout failed, error: "
                       << strerror(errno) << ".");
      return false;
    }
  } else {
    int flags = fcntl(_sockfd, F_GETFL, 0);
    if (flags == -1) {
      ::close(_sockfd);
      ROS_ERROR_STREAM("fcntl get flag failed, error: " << strerror(errno)
                                                        << ".");
      return false;
    }

    if (fcntl(_sockfd, F_SETFL, flags | O_NONBLOCK) == -1) {
      ::close(_sockfd);
      ROS_ERROR_STREAM("fcntl set non block failed, error: " << strerror(errno)
                                                             << ".");
      return false;
    }
  }

  // disable Nagle
  int ret = 0;
  int enable = 1;
  ret = setsockopt(_sockfd, IPPROTO_TCP, TCP_NODELAY, (void*)&enable,
                   sizeof(enable));
  if (ret == -1) {
    ::close(_sockfd);
    ROS_ERROR_STREAM("setsockopt disable Nagle failed, errno: "
                     << errno << ", " << strerror(errno));
    return false;
  }

  return true;
}

void TcpStream::close() {
  if (_sockfd > 0) {
    ::close(_sockfd);
    _sockfd = -1;
    _status = Stream::Status::DISCONNECTED;
  }
}

bool TcpStream::reconnect() {
  if (_auto_reconnect) {
    disconnect();
    if (connect()) {
      return true;
    }
  }
  return false;
}

bool TcpStream::connect() {
  if (_sockfd < 0) {
    this->open();
    if (_sockfd < 0) {
      // error
      return false;
    }
  }

  if (_status == Stream::Status::CONNECTED) {
    return true;
  }

  fd_set fds;
  timeval timeo = {10, 0};
  int ret = 0;
  sockaddr_in peer_addr;

  bzero(&peer_addr, sizeof(peer_addr));
  peer_addr.sin_family = AF_INET;
  peer_addr.sin_port = _peer_port;
  peer_addr.sin_addr.s_addr = _peer_addr;

  int fd_flags = fcntl(_sockfd, F_GETFL);
  if (fd_flags < 0 || fcntl(_sockfd, F_SETFL, fd_flags | O_NONBLOCK) < 0) {
    ROS_ERROR_STREAM("Failed to set noblock, error: " << strerror(errno));
    return false;
  }

  while ((ret = ::connect(_sockfd, reinterpret_cast<sockaddr*>(&peer_addr),
                          sizeof(peer_addr))) < 0) {
    if (errno == EINTR) {
      ROS_INFO("Tcp connect return EINTR, continue.");
      continue;
    } else {
      if ((errno != EISCONN) && (errno != EINPROGRESS) && (errno != EALREADY)) {
        _status = Stream::Status::ERROR;
        _errno = errno;
        ROS_ERROR_STREAM("Connect failed, error: " << strerror(errno));
        return false;
      }

      FD_ZERO(&fds);
      FD_SET(_sockfd, &fds);
      ret = select(_sockfd + 1, NULL, &fds, NULL, &timeo);
      if (ret < 0) {
        _status = Stream::Status::ERROR;
        _errno = errno;
        ROS_ERROR_STREAM("Wait connect failed, error: " << strerror(errno));
        return false;
      } else if (ret == 0) {
        ROS_INFO("Tcp connect timeout.");
        return false;
      } else if (FD_ISSET(_sockfd, &fds)) {
        int error = 0;
        socklen_t len = sizeof(int);

        if (getsockopt(_sockfd, SOL_SOCKET, SO_ERROR, &error, &len) < 0) {
          _status = Stream::Status::ERROR;
          _errno = errno;
          ROS_ERROR_STREAM("Getsockopt failed, error: " << strerror(errno));
          return false;
        }
        if (error != 0) {
          _status = Stream::Status::ERROR;
          _errno = errno;
          ROS_ERROR_STREAM("Socket error: " << strerror(errno));
          return false;
        }

        // connect successfully
        break;
      } else {
        _status = Stream::Status::ERROR;
        _errno = errno;
        ROS_ERROR("Should not be here.");
        return false;
      }
    }
  }

  if (!init_socket()) {
    close();
    _status = Stream::Status::ERROR;
    _errno = errno;
    ROS_ERROR("Failed to init socket.");
    return false;
  }
  ROS_INFO("Tcp connect success.");
  _status = Stream::Status::CONNECTED;
  login();
  return true;
}

bool TcpStream::disconnect() {
  if (_sockfd < 0) {
    // not open
    return false;
  }

  this->close();
  return true;
}

size_t TcpStream::read(uint8_t* buffer, size_t max_length) {
  ssize_t ret = 0;

  if (_status != Stream::Status::CONNECTED) {
    reconnect();
    if (_status != Stream::Status::CONNECTED) {
      return 0;
    }
  }

  if (!readable(10000)) {
    return 0;
  }

  while ((ret = ::recv(_sockfd, buffer, max_length, 0)) < 0) {
    if (errno == EINTR) {
      continue;
    } else {
      // error
      if (errno != EAGAIN) {
        _status = Stream::Status::ERROR;
        _errno = errno;
        ROS_ERROR("Read errno %d, error %s.", errno, strerror(errno));
      }
    }

    return 0;
  }

  if (ret == 0) {
    _status = Stream::Status::ERROR;
    _errno = errno;
    ROS_ERROR("Remote closed.");
    if (reconnect()) {
      ROS_INFO("Reconnect tcp success.");
    }
  }

  return ret;
}

size_t TcpStream::write(const uint8_t* buffer, size_t length) {
  size_t total_nsent = 0;

  if (_status != Stream::Status::CONNECTED) {
    reconnect();
    if (_status != Stream::Status::CONNECTED) {
      return 0;
    }
  }

  while (length > 0) {
    ssize_t nsent = ::send(_sockfd, buffer, length, 0);
    if (nsent < 0) {
      if (errno == EINTR) {
        continue;
      } else {
        // error
        if (errno == EPIPE || errno == ECONNRESET) {
          _status = Stream::Status::DISCONNECTED;
          _errno = errno;
        } else if (errno != EAGAIN) {
          _status = Stream::Status::ERROR;
          _errno = errno;
        }
        return total_nsent;
      }
    }

    total_nsent += nsent;
    length -= nsent;
    buffer += nsent;
  }

  return total_nsent;
}

bool TcpStream::readable(uint32_t timeout_us) {
  // Setup a select call to block for serial data or a timeout
  timespec timeout_ts;
  fd_set readfds;
  FD_ZERO(&readfds);
  FD_SET(_sockfd, &readfds);

  timeout_ts.tv_sec = timeout_us / 1000000;
  timeout_ts.tv_nsec = (timeout_us % 1000000) * 1000;
  int r = pselect(_sockfd + 1, &readfds, NULL, NULL, &timeout_ts, NULL);
  if (r < 0) {
    _status = Stream::Status::ERROR;
    _errno = errno;
    ROS_ERROR("Failed to wait tcp data: %d, %s", errno, strerror(errno));
    return false;
  } else if (r == 0 || !FD_ISSET(_sockfd, &readfds)) {
    return false;
  }
  // Data available to read.
  return true;
}

Stream* Stream::create_tcp(const char* address, uint16_t port,
                           uint32_t timeout_usec) {
  return new TcpStream(address, port, timeout_usec);
}

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
