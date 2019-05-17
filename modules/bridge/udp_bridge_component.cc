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

#include "modules/bridge/udp_bridge_component.h"

namespace apollo {
namespace bridge {

#define BRIDGE_IMPL(pb_msg)    \
  template class UDPBridgeComponent<pb_msg>

#define FREE_ARRY(arry) \
  if (arry) {           \
    delete [] arry;     \
    arry = nullptr;     \
  }

template<typename T>
bool UDPBridgeComponent<T>::Init() {
  AINFO << "UDP bridge init, starting ...";
  return true;
}

template<typename T>
bool UDPBridgeComponent<T>::Proc(
    const std::shared_ptr<T> &pb_msg) {

  if (pb_msg== nullptr) {
    AERROR << "proto msg is not ready!";
    return false;
  }

  apollo::cyber::scheduler::Instance()->CreateTask(
      [&pb_msg]() {
        struct sockaddr_in server_addr;
        server_addr.sin_addr.s_addr = inet_addr(FLAGS_remote_ip.c_str());
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons((uint16_t)FLAGS_remote_port);

        Session session;
        session.Socket(AF_INET, SOCK_DGRAM, 0);
        if (session.Connect((struct sockaddr*)&server_addr,
                            sizeof(server_addr)) < 0) {
          std::cout << "connect to server failed, " << strerror(errno)
                    << std::endl;
          return;
        }

        unsigned int msg_len = pb_msg->ByteSize();
        char *buf = new char[msg_len];
        memset(buf, 0, msg_len);
        pb_msg->SerializeToArray(buf, msg_len);
        if (session.Send(buf, msg_len, 0) < 0) {
          std::cout << "send message failed." << std::endl;
          FREE_ARRY(buf)
          return;
        }
        FREE_ARRY(buf)
      },
      "bridge_client");

  return true;
}

BRIDGE_IMPL(LocalizationEstimate);
BRIDGE_IMPL(planning::ADCTrajectory);

}  // namespace bridge
}  // namespace apollo
