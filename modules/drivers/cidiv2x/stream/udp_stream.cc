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

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include "cyber/cyber.h"

#include "modules/drivers/cidiv2x/stream/stream.h"

namespace apollo {
namespace drivers {
namespace cidiv2x {

class UdpStream : public Stream {
  typedef uint16_t be16_t;
  typedef uint32_t be32_t;

 public:
  UdpStream(const char* address, uint16_t port, uint16_t broad_port,
            uint32_t timeout_usec);
  ~UdpStream();

  virtual bool Connect();
  virtual bool Disconnect();
  virtual size_t read(uint8_t* buffer, size_t max_length, uint8_t flag = 0);
  virtual size_t write(const uint8_t* data, size_t length, uint8_t flag = 0);

 private:
  UdpStream() {}
  void open();
  void close();
  be16_t peer_port_ = 0;
  be16_t peer_broad_port_ = 0;
  be32_t peer_addr_ = 0;
  uint32_t timeout_usec_ = 0;
  int sockfd_ = -1;
  int unicast_sockfd_ = -1;
  int errno_ = 0;
  struct sockaddr_in peer_sockaddr_;
  socklen_t socklenth_ = sizeof(peer_sockaddr_);
};

Stream* Stream::create_udp(const char* address, uint16_t port,
                           uint16_t broad_port, uint32_t timeout_usec) {
  return new UdpStream(address, port, broad_port, timeout_usec);
}

UdpStream::UdpStream(const char* address, uint16_t port, uint16_t broad_port,
                     uint32_t timeout_usec)
    : sockfd_(-1), errno_(0) {
  peer_addr_ = inet_addr(address);
  peer_port_ = htons(port);
  peer_broad_port_ = htons(broad_port);
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
  // Add unicast socket
  int unicastfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (unicastfd < 0) {
    // error
    AERROR << "Create unicast socket failed, errno: " << errno << ", "
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

    if (fcntl(unicastfd, F_SETFL, flags & ~O_NONBLOCK) == -1) {
      ::close(unicastfd);
      AERROR << "fcntl set unicast socket block failed, errno: " << errno
             << ", " << strerror(errno);
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

    if (setsockopt(unicastfd, SOL_SOCKET, SO_RCVTIMEO,
                   reinterpret_cast<char*>(&block_to), sizeof(block_to)) < 0) {
      ::close(unicastfd);
      AERROR << "setsockopt set unicastfd socket rcv timeout failed, errno: "
             << errno << ", " << strerror(errno);
      return;
    }

    if (setsockopt(unicastfd, SOL_SOCKET, SO_SNDTIMEO,
                   reinterpret_cast<char*>(&block_to), sizeof(block_to)) < 0) {
      ::close(unicastfd);
      AERROR << "setsockopt set unicastfd socket snd timeout failed, errno: "
             << errno << ", " << strerror(errno);
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
    if (fcntl(unicastfd, F_SETFL, flags | O_NONBLOCK) == -1) {
      ::close(unicastfd);
      AERROR << "fcntl set unicast socket non block failed, errno: " << errno
             << ", " << strerror(errno);
      return;
    }
  }
  //  const int opt = 1;
  // set socket to broadcast mode
  //  if(setsockopt(fd, SOL_SOCKET, SO_BROADCAST, (char *)&opt, sizeof(opt)) <
  //  0) {
  //    ::close(fd);
  //    AERROR << "setsockopt set broadcast mode failed, errno: " << errno << ",
  //    "
  //           << strerror(errno);
  //    return;
  //  }
  AINFO << "setsockopt set broadcast mode success.";
  struct sockaddr_in addrto;
  bzero(&addrto, sizeof(struct sockaddr_in));
  addrto.sin_family = AF_INET;
  addrto.sin_addr.s_addr = htonl(INADDR_ANY);
  addrto.sin_port = peer_broad_port_;
  if (bind(fd, (struct sockaddr*)&(addrto), sizeof(struct sockaddr_in)) == -1) {
    ::close(fd);
    AERROR << "bind to port failed, errno: " << errno << ", "
           << strerror(errno);
    return;
  }
  AINFO << "bind to port: " << ntohs(peer_broad_port_);
  sockfd_ = fd;

  unicast_sockfd_ = unicastfd;
  return;
}

void UdpStream::close() {
  if (sockfd_ > 0) {
    ::close(sockfd_);
    sockfd_ = -1;
    status_ = Stream::Status::DISCONNECTED;
  }
  if (unicast_sockfd_ > 0) {
    ::close(unicast_sockfd_);
    unicast_sockfd_ = -1;
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

size_t UdpStream::read(uint8_t* buffer, size_t max_length, uint8_t flag) {
  ssize_t ret = 0;
  // struct sockaddr_in peer_sockaddr;
  // socklen_t socklenth = sizeof(peer_sockaddr);
  // flag == 1, broadcast mode
  int sockfd = -1;
  if (flag) {
    bzero(&peer_sockaddr_, sizeof(peer_sockaddr_));
    peer_sockaddr_.sin_family = AF_INET;
    peer_sockaddr_.sin_port = peer_broad_port_;
    // peer_sockaddr.sin_addr.s_addr = peer_addr_;
    peer_sockaddr_.sin_addr.s_addr = htonl(INADDR_ANY);
    sockfd = sockfd_;
  } else {
    peer_sockaddr_.sin_port = peer_port_;
    sockfd = unicast_sockfd_;
  }
  while ((ret = ::recvfrom(sockfd, buffer, max_length, 0,
                           (struct sockaddr*)&peer_sockaddr_,
                           reinterpret_cast<socklen_t*>(&socklenth_))) < 0) {
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
  ADEBUG << "Receive addr: " << inet_ntoa(peer_sockaddr_.sin_addr)
         << ", port: " << ntohs(peer_sockaddr_.sin_port);

  return ret;
}

size_t UdpStream::write(const uint8_t* data, size_t length, uint8_t flag) {
  size_t total_nsent = 0;
  // struct sockaddr_in peer_sockaddr;
  // bzero(&peer_sockaddr, sizeof(peer_sockaddr));
  // peer_sockaddr_.sin_family = AF_INET;
  // peer_sockaddr_.sin_port = peer_port_;
  // peer_sockaddr_.sin_addr.s_addr = peer_addr_;
  int sockfd = -1;
  if (flag) {
    // peer_sockaddr_.sin_port = peer_broad_port_;
    sockfd = sockfd_;
  } else {
    peer_sockaddr_.sin_port = peer_port_;
    sockfd = unicast_sockfd_;
  }
  ADEBUG << "sendto addr: " << inet_ntoa(peer_sockaddr_.sin_addr)
         << ", port: " << ntohs(peer_sockaddr_.sin_port);
  while (length > 0) {
    ssize_t nsent =
        ::sendto(sockfd, data, length, 0, (struct sockaddr*)&peer_sockaddr_,
                 (socklen_t)sizeof(peer_sockaddr_));
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

// size_t UdpStream::broadcast_read(uint8_t* buffer, size_t max_length) {
//   ssize_t ret = 0;
//   // struct sockaddr_in peer_sockaddr;
//   // socklen_t socklenth = sizeof(peer_sockaddr);

//   while ((ret = ::recvfrom(sockfd_, buffer, max_length, 0,
//                            (struct sockaddr*)&peer_sockaddr_,
//                            reinterpret_cast<socklen_t*>(&socklenth_))) < 0) {
//     if (errno == EINTR) {
//       continue;
//     } else {
//       // error
//       if (errno != EAGAIN) {
//         status_ = Stream::Status::ERROR;
//         errno_ = errno;
//       }
//     }

//     return 0;
//   }

//   return ret;
// }

// size_t UdpStream::broadcast_write(const uint8_t* data, size_t length) {
//   size_t total_nsent = 0;
//   struct sockaddr_in peer_sockaddr;
//   bzero(&peer_sockaddr, sizeof(peer_sockaddr));
//   peer_sockaddr.sin_family = AF_INET;
//   peer_sockaddr.sin_port = peer_port_;
//           errno_ = errno;
//         } else if (errno != EAGAIN) {
//           status_ = Stream::Status::ERROR;
//           errno_ = errno;
//         }
//         return total_nsent;
//       }
//     }

//     total_nsent += nsent;
//     length -= nsent;
//     data += nsent;
//   }

//   return total_nsent;
// }        if (errno == EPIPE || errno == ECONNRESET) {
//           status_ = Stream::Status::DISCONNECTED;
//           errno_ = errno;
//         } else if (errno != EAGAIN) {
//           status_ = Stream::Status::ERROR;
//           errno_ = errno;
//         }
//         return total_nsent;
//       }
//     }

//     total_nsent += nsent;
//     length -= nsent;
//     data += nsent;
//   }

//   return total_nsent;
// }

}  // namespace cidiv2x
}  // namespace drivers
}  // namespace apollo
