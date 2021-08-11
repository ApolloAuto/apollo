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
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <cerrno>

#include "cyber/cyber.h"

#include "modules/drivers/gnss/stream/stream.h"

namespace apollo {
namespace drivers {
namespace gnss {

class UdpStream : public Stream {
  typedef uint16_t be16_t;
  typedef uint32_t be32_t;

 public:
  UdpStream(const char* address, uint16_t port, uint32_t timeout_usec);
  ~UdpStream();

  virtual bool Connect();
  virtual bool Disconnect();
  virtual size_t read(uint8_t* buffer, size_t max_length);
  virtual size_t write(const uint8_t* data, size_t length);

 private:
  UdpStream() {}
  void open();
  void close();
  be16_t peer_port_ = 0;
  be32_t peer_addr_ = 0;
  uint32_t timeout_usec_ = 0;
  int sockfd_ = -1;
  int errno_ = 0;
};

Stream* Stream::create_udp(const char* address, uint16_t port,
                           uint32_t timeout_usec) {
  return new UdpStream(address, port, timeout_usec);
}

UdpStream::UdpStream(const char* address, uint16_t port, uint32_t timeout_usec)
    : sockfd_(-1), errno_(0) {
  peer_addr_ = inet_addr(address);
  peer_port_ = htons(port);
  timeout_usec_ = timeout_usec;
  // call open or call open in connect later
}

UdpStream::~UdpStream() { this->close(); }

void UdpStream::open() {
  int fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (fd < 0) {
    // error
    AERROR << "Create socket failed, errno: " << errno << ", "
           << strerror(errno);
    return;
  }

  // block or not block
  if (timeout_usec_ != 0) {
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags == -1) {
      ::close(fd);
      AERROR << "fcntl get flag failed, errno: " << errno << ", "
             << strerror(errno);
      return;
    }

    if (fcntl(fd, F_SETFL, flags & ~O_NONBLOCK) == -1) {
      ::close(fd);
      AERROR << "fcntl set block failed, errno: " << errno << ", "
             << strerror(errno);
      return;
    }

    struct timeval block_to = {timeout_usec_ / 1000000,
                               timeout_usec_ % 1000000};
    if (setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO,
                   reinterpret_cast<char*>(&block_to), sizeof(block_to)) < 0) {
      ::close(fd);
      AERROR << "setsockopt set rcv timeout failed, errno: " << errno << ", "
             << strerror(errno);
      return;
    }

    if (setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO,
                   reinterpret_cast<char*>(&block_to), sizeof(block_to)) < 0) {
      ::close(fd);
      AERROR << "setsockopt set snd timeout failed, errno: " << errno << ", "
             << strerror(errno);
      return;
    }
  } else {
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags == -1) {
      ::close(fd);
      AERROR << "fcntl get flag failed, errno: " << errno << ", "
             << strerror(errno);
      return;
    }

    if (fcntl(fd, F_SETFL, flags | O_NONBLOCK) == -1) {
      ::close(fd);
      AERROR << "fcntl set non block failed, errno: " << errno << ", "
             << strerror(errno);
      return;
    }
  }

  sockfd_ = fd;
}

void UdpStream::close() {
  if (sockfd_ > 0) {
    ::close(sockfd_);
    sockfd_ = -1;
    status_ = Stream::Status::DISCONNECTED;
  }
}

bool UdpStream::Connect() {
  if (sockfd_ < 0) {
    this->open();
    if (sockfd_ < 0) {
      return false;
    }
  }

  if (status_ == Stream::Status::CONNECTED) {
    return true;
  }

  // upper layer support ping method ??
  Login();
  status_ = Stream::Status::CONNECTED;
  return true;
}

bool UdpStream::Disconnect() {
  if (sockfd_ < 0) {
    // not open
    return false;
  }

  this->close();
  return true;
}

size_t UdpStream::read(uint8_t* buffer, size_t max_length) {
  ssize_t ret = 0;
  struct sockaddr_in peer_sockaddr;
  socklen_t socklenth = sizeof(peer_sockaddr);
  bzero(&peer_sockaddr, sizeof(peer_sockaddr));
  peer_sockaddr.sin_family = AF_INET;
  peer_sockaddr.sin_port = peer_port_;
  peer_sockaddr.sin_addr.s_addr = peer_addr_;

  while ((ret = ::recvfrom(sockfd_, buffer, max_length, 0,
                           (struct sockaddr*)&peer_sockaddr,
                           reinterpret_cast<socklen_t*>(&socklenth))) < 0) {
    if (errno == EINTR) {
      continue;
    } else {
      // error
      if (errno != EAGAIN) {
        status_ = Stream::Status::ERROR;
        errno_ = errno;
      }
    }

    return 0;
  }

  return ret;
}

size_t UdpStream::write(const uint8_t* data, size_t length) {
  size_t total_nsent = 0;
  struct sockaddr_in peer_sockaddr;
  bzero(&peer_sockaddr, sizeof(peer_sockaddr));
  peer_sockaddr.sin_family = AF_INET;
  peer_sockaddr.sin_port = peer_port_;
  peer_sockaddr.sin_addr.s_addr = peer_addr_;

  while (length > 0) {
    ssize_t nsent =
        ::sendto(sockfd_, data, length, 0, (struct sockaddr*)&peer_sockaddr,
                 (socklen_t)sizeof(peer_sockaddr));
    if (nsent < 0) {  // error
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
    data += nsent;
  }

  return total_nsent;
}

}  // namespace gnss
}  // namespace drivers
}  // namespace apollo
