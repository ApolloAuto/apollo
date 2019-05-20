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

#include "modules/bridge/udp_bridge_component.h"

namespace apollo {
namespace bridge {

#define BRIDGE_IMPL(pb_msg) \
  template class UDPBridgeComponent<pb_msg>

#define _1K 1024

using apollo::cyber::io::Session;
using apollo::localization::LocalizationEstimate;
using apollo::bridge::UDPBridgeRemoteInfo;

template<typename T>
bool UDPBridgeComponent<T>::Init() {
  AINFO << "UDP bridge init, startin..";
  buf_.reset(_1K);
  apollo::bridge::UDPBridgeRemoteInfo udp_bridge_remote;
  if (!this->GetProtoConfig(&udp_bridge_remote)) {
    AINFO << "load udp bridge component proto param failed";
    return false;
  }
  remote_ip_ = udp_bridge_remote.remote_ip();
  remote_port_ = udp_bridge_remote.remote_port();
  proto_name_ = udp_bridge_remote.proto_name();
  AINFO << "UDP Bridge remote ip is: "<< remote_ip_;
  AINFO << "UDP Bridge remote port is: "<< remote_port_;
  AINFO << "UDP Bridge for Proto is: "<< proto_name_;
  return true;
}

template<typename T>
bool UDPBridgeComponent<T>::Proc(const std::shared_ptr<T> &pb_msg) {
  if (remote_port_ == 0 || remote_ip_.empty()) {
    AERROR << "remote info is invalid!";
    return false;
  }

  if (pb_msg== nullptr) {
    AERROR << "proto msg is not ready!";
    return false;
  }

  apollo::cyber::scheduler::Instance()->CreateTask(
      [this, &pb_msg]() {
        struct sockaddr_in server_addr;
        server_addr.sin_addr.s_addr = inet_addr(remote_ip_.c_str());
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons((uint16_t)remote_port_);

        Session session;
        session.Socket(AF_INET, SOCK_DGRAM, 0);
        if (session.Connect((struct sockaddr*)&server_addr,
                            sizeof(server_addr)) < 0) {
          std::cout << "connect to server failed, " << strerror(errno)
                    << std::endl;
          return;
        }

        unsigned int msg_len = pb_msg->ByteSize();
        buf_.reset(msg_len);
        std::lock_guard<std::mutex> lg(mutex_);
        char *buf = buf_;
        pb_msg->SerializeToArray(buf, msg_len);
        if (session.Send(buf, msg_len, 0) < 0) {
          std::cout << "send message failed." << std::endl;
          return;
        }
      },
      "bridge_client");

  return true;
}

BRIDGE_IMPL(LocalizationEstimate);
BRIDGE_IMPL(planning::ADCTrajectory);

}  // namespace bridge
}  // namespace apollo
