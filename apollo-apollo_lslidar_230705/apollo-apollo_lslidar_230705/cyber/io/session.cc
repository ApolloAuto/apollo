/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "cyber/io/session.h"

#include "cyber/common/log.h"

namespace apollo {
namespace cyber {
namespace io {

Session::Session() : Session(-1) {}

Session::Session(int fd) : fd_(fd), poll_handler_(nullptr) {
  poll_handler_.reset(new PollHandler(fd_));
}

int Session::Socket(int domain, int type, int protocol) {
  if (fd_ != -1) {
    AINFO << "session has hold a valid fd[" << fd_ << "]";
    return -1;
  }
  int sock_fd = socket(domain, type | SOCK_NONBLOCK, protocol);
  if (sock_fd != -1) {
    set_fd(sock_fd);
  }
  return sock_fd;
}

int Session::Listen(int backlog) {
  ACHECK(fd_ != -1);
  return listen(fd_, backlog);
}

int Session::Bind(const struct sockaddr *addr, socklen_t addrlen) {
  ACHECK(fd_ != -1);
  ACHECK(addr != nullptr);
  return bind(fd_, addr, addrlen);
}

auto Session::Accept(struct sockaddr *addr, socklen_t *addrlen) -> SessionPtr {
  ACHECK(fd_ != -1);

  int sock_fd = accept4(fd_, addr, addrlen, SOCK_NONBLOCK);
  while (sock_fd == -1 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
    poll_handler_->Block(-1, true);
    sock_fd = accept4(fd_, addr, addrlen, SOCK_NONBLOCK);
  }

  if (sock_fd == -1) {
    return nullptr;
  }

  return std::make_shared<Session>(sock_fd);
}

int Session::Connect(const struct sockaddr *addr, socklen_t addrlen) {
  ACHECK(fd_ != -1);

  int optval;
  socklen_t optlen = sizeof(optval);
  int res = connect(fd_, addr, addrlen);
  if (res == -1 && errno == EINPROGRESS) {
    poll_handler_->Block(-1, false);
    getsockopt(fd_, SOL_SOCKET, SO_ERROR, reinterpret_cast<void *>(&optval),
               &optlen);
    if (optval == 0) {
      res = 0;
    } else {
      errno = optval;
    }
  }
  return res;
}

int Session::Close() {
  ACHECK(fd_ != -1);

  poll_handler_->Unblock();
  int res = close(fd_);
  fd_ = -1;
  return res;
}

ssize_t Session::Recv(void *buf, size_t len, int flags, int timeout_ms) {
  ACHECK(buf != nullptr);
  ACHECK(fd_ != -1);

  ssize_t nbytes = recv(fd_, buf, len, flags);
  if (timeout_ms == 0) {
    return nbytes;
  }

  while (nbytes == -1 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
    if (poll_handler_->Block(timeout_ms, true)) {
      nbytes = recv(fd_, buf, len, flags);
    }
    if (timeout_ms > 0) {
      break;
    }
  }
  return nbytes;
}

ssize_t Session::RecvFrom(void *buf, size_t len, int flags,
                          struct sockaddr *src_addr, socklen_t *addrlen,
                          int timeout_ms) {
  ACHECK(buf != nullptr);
  ACHECK(fd_ != -1);

  ssize_t nbytes = recvfrom(fd_, buf, len, flags, src_addr, addrlen);
  if (timeout_ms == 0) {
    return nbytes;
  }

  while (nbytes == -1 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
    if (poll_handler_->Block(timeout_ms, true)) {
      nbytes = recvfrom(fd_, buf, len, flags, src_addr, addrlen);
    }
    if (timeout_ms > 0) {
      break;
    }
  }
  return nbytes;
}

ssize_t Session::Send(const void *buf, size_t len, int flags, int timeout_ms) {
  ACHECK(buf != nullptr);
  ACHECK(fd_ != -1);

  ssize_t nbytes = send(fd_, buf, len, flags);
  if (timeout_ms == 0) {
    return nbytes;
  }

  while ((nbytes == -1) && (errno == EAGAIN || errno == EWOULDBLOCK)) {
    if (poll_handler_->Block(timeout_ms, false)) {
      nbytes = send(fd_, buf, len, flags);
    }
    if (timeout_ms > 0) {
      break;
    }
  }
  return nbytes;
}

ssize_t Session::SendTo(const void *buf, size_t len, int flags,
                        const struct sockaddr *dest_addr, socklen_t addrlen,
                        int timeout_ms) {
  ACHECK(buf != nullptr);
  ACHECK(dest_addr != nullptr);
  ACHECK(fd_ != -1);

  ssize_t nbytes = sendto(fd_, buf, len, flags, dest_addr, addrlen);
  if (timeout_ms == 0) {
    return nbytes;
  }

  while ((nbytes == -1) && (errno == EAGAIN || errno == EWOULDBLOCK)) {
    if (poll_handler_->Block(timeout_ms, false)) {
      nbytes = sendto(fd_, buf, len, flags, dest_addr, addrlen);
    }
    if (timeout_ms > 0) {
      break;
    }
  }
  return nbytes;
}

ssize_t Session::Read(void *buf, size_t count, int timeout_ms) {
  ACHECK(buf != nullptr);
  ACHECK(fd_ != -1);

  ssize_t nbytes = read(fd_, buf, count);
  if (timeout_ms == 0) {
    return nbytes;
  }

  while ((nbytes == -1) && (errno == EAGAIN || errno == EWOULDBLOCK)) {
    if (poll_handler_->Block(timeout_ms, true)) {
      nbytes = read(fd_, buf, count);
    }
    if (timeout_ms > 0) {
      break;
    }
  }
  return nbytes;
}

ssize_t Session::Write(const void *buf, size_t count, int timeout_ms) {
  ACHECK(buf != nullptr);
  ACHECK(fd_ != -1);

  ssize_t nbytes = write(fd_, buf, count);
  if (timeout_ms == 0) {
    return nbytes;
  }

  while ((nbytes == -1) && (errno == EAGAIN || errno == EWOULDBLOCK)) {
    if (poll_handler_->Block(timeout_ms, false)) {
      nbytes = write(fd_, buf, count);
    }
    if (timeout_ms > 0) {
      break;
    }
  }
  return nbytes;
}

}  // namespace io
}  // namespace cyber
}  // namespace apollo
