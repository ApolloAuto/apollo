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
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
#include <cinttypes>
#include <iostream>

#include "cyber/cyber.h"
#include "modules/drivers/gnss/stream/stream.h"
#include "modules/drivers/gnss/stream/tcp_stream.h"

namespace apollo {
namespace drivers {
namespace gnss {

TcpStream::TcpStream(const char* address, uint16_t port, uint32_t timeout_usec,
                     bool auto_reconnect)
    : sockfd_(-1), errno_(0), auto_reconnect_(auto_reconnect) {
  peer_addr_ = inet_addr(address);
  peer_port_ = htons(port);
  timeout_usec_ = timeout_usec;
}

TcpStream::~TcpStream() { this->close(); }

void TcpStream::open() {
  int fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (fd < 0) {
    // error
    AERROR << "create socket failed, errno: " << errno << ", "
           << strerror(errno);
    return;
  }

  sockfd_ = fd;
}

bool TcpStream::InitSocket() {
  if (sockfd_ < 0) {
    return false;
  }

  // block or not block
  if (timeout_usec_ != 0) {
    int flags = fcntl(sockfd_, F_GETFL, 0);
    if (flags == -1) {
      ::close(sockfd_);
      AERROR << "fcntl get flag failed, error: " << strerror(errno);
      return false;
    }

    if (fcntl(sockfd_, F_SETFL, flags & ~O_NONBLOCK) == -1) {
      ::close(sockfd_);
      AERROR << "fcntl set block failed, error: " << strerror(errno);
      return false;
    }

    timeval block_to = {timeout_usec_ / 1000000, timeout_usec_ % 1000000};
    if (setsockopt(sockfd_, SOL_SOCKET, SO_RCVTIMEO, &block_to,
                   sizeof(block_to)) < 0) {
      ::close(sockfd_);
      AERROR << "setsockopt set rcv timeout failed, error: " << strerror(errno);
      return false;
    }

    if (setsockopt(sockfd_, SOL_SOCKET, SO_SNDTIMEO, &block_to,
                   sizeof(block_to)) < 0) {
      ::close(sockfd_);
      AERROR << "setsockopt set snd timeout failed, error: " << strerror(errno);
      return false;
    }
  } else {
    int flags = fcntl(sockfd_, F_GETFL, 0);
    if (flags == -1) {
      ::close(sockfd_);
      AERROR << "fcntl get flag failed, error: " << strerror(errno);
      return false;
    }

    if (fcntl(sockfd_, F_SETFL, flags | O_NONBLOCK) == -1) {
      ::close(sockfd_);
      AERROR << "fcntl set non block failed, error: " << strerror(errno);
      return false;
    }
  }

  // disable Nagle
  int ret = 0;
  int enable = 1;
  ret = setsockopt(sockfd_, IPPROTO_TCP, TCP_NODELAY,
                   reinterpret_cast<void*>(&enable), sizeof(enable));
  if (ret == -1) {
    ::close(sockfd_);
    AERROR << "setsockopt disable Nagle failed, errno: " << errno << ", "
           << strerror(errno);
    return false;
  }

  return true;
}

void TcpStream::close() {
  if (sockfd_ > 0) {
    ::close(sockfd_);
    sockfd_ = -1;
    status_ = Stream::Status::DISCONNECTED;
  }
}

bool TcpStream::Reconnect() {
  if (auto_reconnect_) {
    Disconnect();
    if (Connect()) {
      return true;
    }
  }
  return false;
}

bool TcpStream::Connect() {
  if (sockfd_ < 0) {
    this->open();
    if (sockfd_ < 0) {
      // error
      return false;
    }
  }

  if (status_ == Stream::Status::CONNECTED) {
    return true;
  }

  fd_set fds;
  timeval timeo = {10, 0};
  int ret = 0;
  sockaddr_in peer_addr;

  bzero(&peer_addr, sizeof(peer_addr));
  peer_addr.sin_family = AF_INET;
  peer_addr.sin_port = peer_port_;
  peer_addr.sin_addr.s_addr = peer_addr_;

  int fd_flags = fcntl(sockfd_, F_GETFL);
  if (fd_flags < 0 || fcntl(sockfd_, F_SETFL, fd_flags | O_NONBLOCK) < 0) {
    AERROR << "Failed to set noblock, error: " << strerror(errno);
    return false;
  }

  while ((ret = ::connect(sockfd_, reinterpret_cast<sockaddr*>(&peer_addr),
                          sizeof(peer_addr))) < 0) {
    if (errno == EINTR) {
      AINFO << "Tcp connect return EINTR, continue.";
      continue;
    } else {
      if ((errno != EISCONN) && (errno != EINPROGRESS) && (errno != EALREADY)) {
        status_ = Stream::Status::ERROR;
        errno_ = errno;
        AERROR << "Connect failed, error: " << strerror(errno);
        return false;
      }

      FD_ZERO(&fds);
      FD_SET(sockfd_, &fds);
      ret = select(sockfd_ + 1, NULL, &fds, NULL, &timeo);
      if (ret < 0) {
        status_ = Stream::Status::ERROR;
        errno_ = errno;
        AERROR << "Wait connect failed, error: " << strerror(errno);
        return false;
      } else if (ret == 0) {
        AINFO << "Tcp connect timeout.";
        return false;
      } else if (FD_ISSET(sockfd_, &fds)) {
        int error = 0;
        socklen_t len = sizeof(int);

        if (getsockopt(sockfd_, SOL_SOCKET, SO_ERROR, &error, &len) < 0) {
          status_ = Stream::Status::ERROR;
          errno_ = errno;
          AERROR << "Getsockopt failed, error: " << strerror(errno);
          return false;
        }
        if (error != 0) {
          status_ = Stream::Status::ERROR;
          errno_ = errno;
          AERROR << "Socket error: " << strerror(errno);
          return false;
        }

        // connect successfully
        break;
      } else {
        status_ = Stream::Status::ERROR;
        errno_ = errno;
        AERROR << "Should not be here.";
        return false;
      }
    }
  }

  if (!InitSocket()) {
    close();
    status_ = Stream::Status::ERROR;
    errno_ = errno;
    AERROR << "Failed to init socket.";
    return false;
  }
  AINFO << "Tcp connect success.";
  status_ = Stream::Status::CONNECTED;
  Login();
  return true;
}

bool TcpStream::Disconnect() {
  if (sockfd_ < 0) {
    // not open
    return false;
  }

  this->close();
  return true;
}

size_t TcpStream::read(uint8_t* buffer, size_t max_length) {
  ssize_t ret = 0;

  if (status_ != Stream::Status::CONNECTED) {
    Reconnect();
    if (status_ != Stream::Status::CONNECTED) {
      return 0;
    }
  }

  if (!Readable(10000)) {
    return 0;
  }

  while ((ret = ::recv(sockfd_, buffer, max_length, 0)) < 0) {
    if (errno == EINTR) {
      continue;
    } else {
      // error
      if (errno != EAGAIN) {
        status_ = Stream::Status::ERROR;
        errno_ = errno;
        AERROR << "Read errno " << errno << ", error " << strerror(errno);
      }
    }

    return 0;
  }

  if (ret == 0) {
    status_ = Stream::Status::ERROR;
    errno_ = errno;
    AERROR << "Remote closed.";
    if (Reconnect()) {
      AINFO << "Reconnect tcp success.";
    }
  }

  return ret;
}

size_t TcpStream::write(const uint8_t* buffer, size_t length) {
  size_t total_nsent = 0;

  if (status_ != Stream::Status::CONNECTED) {
    Reconnect();
    if (status_ != Stream::Status::CONNECTED) {
      return 0;
    }
  }

  while (length > 0) {
    ssize_t nsent = ::send(sockfd_, buffer, length, 0);
    if (nsent < 0) {
      if (errno == EINTR) {
        continue;
      } else {
        // error
        if (errno == EPIPE || errno == ECONNRESET) {
          status_ = Stream::Status::DISCONNECTED;
          errno_ = errno;
        } else if (errno != EAGAIN) {
          status_ = Stream::Status::ERROR;
          errno_ = errno;
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

bool TcpStream::Readable(uint32_t timeout_us) {
  // Setup a select call to block for serial data or a timeout
  timespec timeout_ts;
  fd_set readfds;
  FD_ZERO(&readfds);
  FD_SET(sockfd_, &readfds);

  timeout_ts.tv_sec = timeout_us / 1000000;
  timeout_ts.tv_nsec = (timeout_us % 1000000) * 1000;
  int r = pselect(sockfd_ + 1, &readfds, NULL, NULL, &timeout_ts, NULL);
  if (r < 0) {
    status_ = Stream::Status::ERROR;
    errno_ = errno;
    AERROR << "Failed to wait tcp data: " << errno << ", " << strerror(errno);
    return false;
  } else if (r == 0 || !FD_ISSET(sockfd_, &readfds)) {
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
