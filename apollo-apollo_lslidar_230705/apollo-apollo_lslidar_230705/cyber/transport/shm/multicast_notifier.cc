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

#include "cyber/transport/shm/multicast_notifier.h"

#include <arpa/inet.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <cstring>
#include <string>

#include "cyber/common/global_data.h"
#include "cyber/common/log.h"

namespace apollo {
namespace cyber {
namespace transport {

using common::GlobalData;

MulticastNotifier::MulticastNotifier() {
  if (!Init()) {
    Shutdown();
  }
}

MulticastNotifier::~MulticastNotifier() { Shutdown(); }

void MulticastNotifier::Shutdown() {
  if (is_shutdown_.exchange(true)) {
    return;
  }

  if (notify_fd_ != -1) {
    close(notify_fd_);
    notify_fd_ = -1;
  }
  memset(&notify_addr_, 0, sizeof(notify_addr_));

  if (listen_fd_ != -1) {
    close(listen_fd_);
    listen_fd_ = -1;
  }
  memset(&listen_addr_, 0, sizeof(listen_addr_));
}

bool MulticastNotifier::Notify(const ReadableInfo& info) {
  if (is_shutdown_.load()) {
    return false;
  }

  std::string info_str;
  info.SerializeTo(&info_str);
  ssize_t nbytes =
      sendto(notify_fd_, info_str.c_str(), info_str.size(), 0,
             (struct sockaddr*)&notify_addr_, sizeof(notify_addr_));
  return nbytes > 0;
}

bool MulticastNotifier::Listen(int timeout_ms, ReadableInfo* info) {
  if (is_shutdown_.load()) {
    return false;
  }

  if (info == nullptr) {
    AERROR << "info nullptr.";
    return false;
  }

  struct pollfd fds;
  fds.fd = listen_fd_;
  fds.events = POLLIN;
  int ready_num = poll(&fds, 1, timeout_ms);
  if (ready_num > 0) {
    char buf[32] = {0};  // larger than ReadableInfo::kSize
    ssize_t nbytes = recvfrom(listen_fd_, buf, 32, 0, nullptr, nullptr);
    if (nbytes == -1) {
      AERROR << "fail to recvfrom, " << strerror(errno);
      return false;
    }
    return info->DeserializeFrom(buf, nbytes);
  } else if (ready_num == 0) {
    ADEBUG << "timeout, no readableinfo.";
  } else {
    if (errno == EINTR) {
      AINFO << "poll was interrupted.";
    } else {
      AERROR << "fail to poll, " << strerror(errno);
    }
  }
  return false;
}

bool MulticastNotifier::Init() {
  std::string mcast_ip("239.255.0.100");
  uint16_t mcast_port = 8888;

  auto& g_conf = GlobalData::Instance()->Config();
  if (g_conf.has_transport_conf() && g_conf.transport_conf().has_shm_conf() &&
      g_conf.transport_conf().shm_conf().has_shm_locator()) {
    auto& locator = g_conf.transport_conf().shm_conf().shm_locator();
    mcast_ip = locator.ip();
    mcast_port = static_cast<uint16_t>(locator.port());
  }

  ADEBUG << "multicast notifier ip: " << mcast_ip;
  ADEBUG << "multicast notifier port: " << mcast_port;

  notify_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (notify_fd_ == -1) {
    AERROR << "fail to create notify fd, " << strerror(errno);
    return false;
  }

  memset(&notify_addr_, 0, sizeof(notify_addr_));
  notify_addr_.sin_family = AF_INET;
  notify_addr_.sin_addr.s_addr = inet_addr(mcast_ip.c_str());
  notify_addr_.sin_port = htons(mcast_port);

  listen_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
  if (listen_fd_ == -1) {
    AERROR << "fail to create listen fd, " << strerror(errno);
    return false;
  }

  if (fcntl(listen_fd_, F_SETFL, O_NONBLOCK) == -1) {
    AERROR << "fail to set listen fd nonblock, " << strerror(errno);
    return false;
  }

  memset(&listen_addr_, 0, sizeof(listen_addr_));
  listen_addr_.sin_family = AF_INET;
  listen_addr_.sin_addr.s_addr = htonl(INADDR_ANY);
  listen_addr_.sin_port = htons(mcast_port);

  int yes = 1;
  if (setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes)) < 0) {
    AERROR << "fail to setsockopt SO_REUSEADDR, " << strerror(errno);
    return false;
  }

  if (bind(listen_fd_, (struct sockaddr*)&listen_addr_, sizeof(listen_addr_)) <
      0) {
    AERROR << "fail to bind addr, " << strerror(errno);
    return false;
  }

  int loop = 1;
  if (setsockopt(listen_fd_, IPPROTO_IP, IP_MULTICAST_LOOP, &loop,
                 sizeof(loop)) < 0) {
    AERROR << "fail to setsockopt IP_MULTICAST_LOOP, " << strerror(errno);
    return false;
  }

  struct ip_mreq mreq;
  mreq.imr_multiaddr.s_addr = inet_addr(mcast_ip.c_str());
  mreq.imr_interface.s_addr = htonl(INADDR_ANY);
  if (setsockopt(listen_fd_, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq,
                 sizeof(mreq)) < 0) {
    AERROR << "fail to setsockopt IP_ADD_MEMBERSHIP, " << strerror(errno);
    return false;
  }

  return true;
}

}  // namespace transport
}  // namespace cyber
}  // namespace apollo
