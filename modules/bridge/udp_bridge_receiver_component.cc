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

#include "modules/bridge/udp_bridge_receiver_component.h"
#include "modules/bridge/common/util.h"
#include "modules/bridge/common/macro.h"

namespace apollo {
namespace bridge {

template<typename T>
UDPBridgeReceiverComponent<T>::UDPBridgeReceiverComponent()
  : monitor_logger_buffer_(common::monitor::MonitorMessageItem::CONTROL) {}

template <typename T>
UDPBridgeReceiverComponent<T>::~UDPBridgeReceiverComponent() {
  if (session_) {
    session_->Close();
  }
}

template<typename T>
bool UDPBridgeReceiverComponent<T>::Init() {
  AINFO << "UDP bridge receiver init, startin..";
  buf_.reset(_1K);
  apollo::bridge::UDPBridgeReceiverRemoteInfo udp_bridge_remote;
  if (!this->GetProtoConfig(&udp_bridge_remote)) {
    AINFO << "load udp bridge component proto param failed";
    return false;
  }
  bind_port_ = udp_bridge_remote.bind_port();
  proto_name_ = udp_bridge_remote.proto_name();
  topic_name_ = udp_bridge_remote.topic_name();
  AINFO << "UDP Bridge remote port is: "<< bind_port_;
  AINFO << "UDP Bridge for Proto is: "<< proto_name_;
  writer_ = node_->CreateWriter<T>(topic_name_.c_str());

  if (!InitSession((uint16_t)bind_port_)) {
    return false;
  }
  MsgDispatcher();
  return true;
}

template <typename T>
bool UDPBridgeReceiverComponent<T>::InitSession(uint16_t port) {
  struct sockaddr_in addr;
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htons(INADDR_ANY);
  addr.sin_port = htons(port);

  session_->Socket(AF_INET, SOCK_DGRAM, 0);
  if (session_->Bind((struct sockaddr*)&addr, sizeof(addr)) < 0) {
    AINFO << "bind prot ["  << port << "] failed";
    session_->Close();
    return false;
  }
  return true;
}

template <typename T>
bool UDPBridgeReceiverComponent<T>::MsgHandle() {
  struct sockaddr_in client_addr;
  socklen_t sock_len = static_cast<socklen_t>(sizeof(client_addr));
  int bytes = 0;
  buf_.reset(_1K);

  char header_size[sizeof(size_t)] = {0};
  bytes = static_cast<int>(
      session_->RecvFrom(header_size, sizeof(header_size), 0,
        (struct sockaddr*)&client_addr, &sock_len));
  if (bytes <= 0) {
    return false;
  }
  int msg_len = GetProtoSize(header_size, sizeof(size_t));
  if (msg_len <= 0) {
    return false;
  }

  buf_.reset(msg_len);
  bytes = static_cast<int>(
      session_->RecvFrom(buf_, buf_.capacity(), 0,
        (struct sockaddr*)&client_addr, &sock_len));
  if (bytes <= 0 || bytes != msg_len) {
    return false;
  }

  auto pb_msg = std::make_shared<T>();
  pb_msg->ParseFromArray(buf_, msg_len);
  writer_->Write(pb_msg);
  return true;
}

template <typename T>
void UDPBridgeReceiverComponent<T>::MsgDispatcher() {
  apollo::cyber::scheduler::Instance()->CreateTask(
      [this]() {
        while (true) {
          MsgHandle();
        }
        session_->Close();
      },
      "bridge_server");
}
}  // namespace bridge
}  // namespace apollo
