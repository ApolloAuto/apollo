/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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
#include <pthread.h>
#include <sys/epoll.h>
#include <sys/resource.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cstdlib>

#include "modules/common_msgs/chassis_msgs/chassis.pb.h"

#include "cyber/common/log.h"
#include "modules/bridge/common/bridge_proto_diserialized_buf.h"
#include "modules/bridge/common/macro.h"
#include "modules/bridge/common/udp_listener.h"
#include "modules/bridge/common/util.h"

using apollo::bridge::BRIDGE_HEADER_FLAG;
using apollo::bridge::BridgeHeader;
using apollo::bridge::FRAME_SIZE;
using apollo::bridge::HEADER_FLAG_SIZE;
using apollo::bridge::hsize;
using apollo::bridge::MAXEPOLLSIZE;
using apollo::canbus::Chassis;
using BPDBChassis = apollo::bridge::BridgeProtoDiserializedBuf<Chassis>;

void *pthread_handle_message(void *pfd) {
  struct sockaddr_in client_addr;
  socklen_t sock_len = static_cast<socklen_t>(sizeof(client_addr));
  int bytes = 0;
  int total_recv = 2 * FRAME_SIZE;
  char total_buf[2 * FRAME_SIZE] = {0};
  bytes =
      static_cast<int>(recvfrom(*static_cast<int *>(pfd), total_buf, total_recv,
                                0, (struct sockaddr *)&client_addr, &sock_len));
  ADEBUG << "total recv " << bytes;
  if (bytes <= 0 || bytes > total_recv) {
    pthread_exit(nullptr);
  }
  char header_flag[sizeof(BRIDGE_HEADER_FLAG) + 1] = {0};
  size_t offset = 0;
  memcpy(header_flag, total_buf, HEADER_FLAG_SIZE);
  if (strcmp(header_flag, BRIDGE_HEADER_FLAG) != 0) {
    ADEBUG << "header flag not match!";
    pthread_exit(nullptr);
  }
  offset += sizeof(BRIDGE_HEADER_FLAG) + 1;

  char header_size_buf[sizeof(hsize) + 1] = {0};
  const char *cursor = total_buf + offset;
  memcpy(header_size_buf, cursor, sizeof(hsize));
  hsize header_size = *(reinterpret_cast<hsize *>(header_size_buf));
  if (header_size > FRAME_SIZE) {
    ADEBUG << "header size is more than FRAME_SIZE!";
    pthread_exit(nullptr);
  }
  offset += sizeof(hsize) + 1;

  BridgeHeader header;
  size_t buf_size = header_size - offset;
  cursor = total_buf + offset;
  if (!header.Diserialize(cursor, buf_size)) {
    ADEBUG << "header diserialize failed!";
    pthread_exit(nullptr);
  }

  ADEBUG << "proto name : " << header.GetMsgName().c_str();
  ADEBUG << "proto sequence num: " << header.GetMsgID();
  ADEBUG << "proto total frames: " << header.GetTotalFrames();
  ADEBUG << "proto frame index: " << header.GetIndex();

  BPDBChassis *proto_buf = new BPDBChassis();
  proto_buf->Initialize(header);
  if (!proto_buf) {
    pthread_exit(nullptr);
  }

  cursor = total_buf + header_size;
  char *buf = proto_buf->GetBuf(header.GetFramePos());
  memcpy(buf, cursor, header.GetFrameSize());
  proto_buf->UpdateStatus(header.GetIndex());
  if (proto_buf->IsReadyDiserialize()) {
    auto pb_msg = std::make_shared<Chassis>();
    proto_buf->Diserialized(pb_msg);
    ADEBUG << "sequence num: " << pb_msg->header().sequence_num();
    ADEBUG << "timestamp sec: " << pb_msg->header().timestamp_sec();
    ADEBUG << "engine rpm: " << pb_msg->engine_rpm();
    ADEBUG << "odometer m: " << pb_msg->odometer_m();
    ADEBUG << "throttle percentage: " << pb_msg->throttle_percentage();
    ADEBUG << "brake percentage: " << pb_msg->brake_percentage();
    ADEBUG << "steering percentage: " << pb_msg->steering_percentage();
    ADEBUG << "steering torque nm: " << pb_msg->steering_torque_nm();
    ADEBUG << "parking brake: " << pb_msg->parking_brake();
  }
  pthread_exit(nullptr);
}

bool receive(uint16_t port) {
  struct rlimit rt;
  rt.rlim_max = rt.rlim_cur = MAXEPOLLSIZE;
  if (setrlimit(RLIMIT_NOFILE, &rt) == -1) {
    ADEBUG << "set resource limitation failed";
    return false;
  }

  int listener_sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (listener_sock == -1) {
    ADEBUG << "create socket failed";
    return false;
  }
  int opt = SO_REUSEADDR;
  setsockopt(listener_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
  if (fcntl(listener_sock, F_SETFL,
            fcntl(listener_sock, F_GETFD, 0) | O_NONBLOCK) == -1) {
    ADEBUG << "set nonblocking failed";
    return false;
  }

  struct sockaddr_in serv_addr;
  serv_addr.sin_family = PF_INET;
  serv_addr.sin_port = htons((uint16_t)port);
  serv_addr.sin_addr.s_addr = INADDR_ANY;
  if (bind(listener_sock, (struct sockaddr *)&serv_addr,
           sizeof(struct sockaddr)) == -1) {
    close(listener_sock);
    ADEBUG << "bind socket failed";
    return false;
  }
  int kdpfd = epoll_create(MAXEPOLLSIZE);
  struct epoll_event ev;
  ev.events = EPOLLIN | EPOLLET;
  ev.data.fd = listener_sock;
  if (epoll_ctl(kdpfd, EPOLL_CTL_ADD, listener_sock, &ev) < 0) {
    ADEBUG << "set control interface for an epoll descriptor failed";
    close(listener_sock);
    return false;
  }

  ADEBUG << "Ready!";

  int nfds = -1;
  bool res = true;
  struct epoll_event events[MAXEPOLLSIZE];
  while (true) {
    nfds = epoll_wait(kdpfd, events, 10000, -1);
    if (nfds == -1) {
      ADEBUG << "some error occurs while waiting for I/O event";
      res = false;
      break;
    }

    for (int i = 0; i < nfds; ++i) {
      if (events[i].data.fd == listener_sock) {
        pthread_t thread;
        pthread_attr_t attr;
        pthread_attr_init(&attr);
        pthread_attr_setscope(&attr, PTHREAD_SCOPE_SYSTEM);
        pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
        if (pthread_create(&thread, &attr, &pthread_handle_message,
                           reinterpret_cast<void *>(&events[i].data.fd))) {
          ADEBUG << "message handler creation failed";
          res = false;
          break;
        }
      }
    }
    if (!res) {
      break;
    }
  }
  close(listener_sock);
  return res;
}

int main(int argc, char *argv[]) {
  receive(8900);
  return 0;
}
