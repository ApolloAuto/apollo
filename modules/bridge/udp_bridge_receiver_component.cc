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
#include "modules/bridge/common/macro.h"
#include "modules/bridge/common/util.h"

namespace apollo {
namespace bridge {

#define BRIDGE_RECV_IMPL(pb_msg) \
  template class UDPBridgeReceiverComponent<pb_msg>

template <typename T>
UDPBridgeReceiverComponent<T>::UDPBridgeReceiverComponent()
    : monitor_logger_buffer_(common::monitor::MonitorMessageItem::CONTROL) {}

template <typename T>
UDPBridgeReceiverComponent<T>::~UDPBridgeReceiverComponent() {
  if (session_) {
    session_->Close();
  }
  for (auto proto : proto_list_) {
    FREE_POINTER(proto);
  }
}

template <typename T>
bool UDPBridgeReceiverComponent<T>::Init() {
  AINFO << "UDP bridge receiver init, startin..";
  apollo::bridge::UDPBridgeReceiverRemoteInfo udp_bridge_remote;
  if (!this->GetProtoConfig(&udp_bridge_remote)) {
    AINFO << "load udp bridge component proto param failed";
    return false;
  }
  bind_port_ = udp_bridge_remote.bind_port();
  proto_name_ = udp_bridge_remote.proto_name();
  topic_name_ = udp_bridge_remote.topic_name();
  AINFO << "UDP Bridge remote port is: " << bind_port_;
  AINFO << "UDP Bridge for Proto is: " << proto_name_;
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
    AINFO << "bind prot [" << port << "] failed";
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

  char header_flag[sizeof(BRIDGE_HEADER_FLAG) + 1] = {0};
  bytes = static_cast<int>(session_->RecvFrom(header_flag,
    sizeof(header_flag) + 1, 0, (struct sockaddr*)&client_addr,
    &sock_len));
  if (bytes != sizeof(BRIDGE_HEADER_FLAG) + 1 ||
    strcmp(header_flag, BRIDGE_HEADER_FLAG) != 0) {
    return false;
  }
  char header_size_buf[sizeof(size_t) + 1] = {0};
  bytes = static_cast<int>(session_->RecvFrom(header_size_buf,
    sizeof(size_t) + 1, 0, (struct sockaddr*)&client_addr, &sock_len));
  if (bytes != sizeof(size_t) + 1) {
    return false;
  }
  size_t header_size = *reinterpret_cast<size_t *>(header_size_buf);
  if (header_size == 0) {
    return false;
  }
  char *header_buf = new char[header_size];
  bytes = static_cast<int>(session_->RecvFrom(header_buf, header_size,
    0, (struct sockaddr*)&client_addr, &sock_len));
  if (bytes !=  header_size) {
    return false;
  }

  BridgeHeader header;
  if (!header.Diserialize(header_buf)) {
    FREE_ARRY(header_buf);
    return false;
  }
  FREE_ARRY(header_buf);
  BridgeProtoBuf<T> *proto_buf = CreateBridgeProtoBuf(header);

  if (!proto_buf) {
    return false;
  }

  char *buf = proto_buf->GetBuf(header.GetFramePos());
  bytes = static_cast<int>(session_->RecvFrom(buf, header.GetFrameSize(),
    0, (struct sockaddr*)&client_addr, &sock_len));
  proto_buf->UpdateStatus(header.GetIndex());
  if (proto_buf->IsReadyDiserialize()) {
    auto pb_msg = std::make_shared<T>();
    proto_buf->Diserialized(pb_msg);
    writer_->Write(pb_msg);
    RemoveItem(&proto_list_, proto_buf);
  }
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

template <typename T>
BridgeProtoBuf<T> *UDPBridgeReceiverComponent<T>::CreateBridgeProtoBuf(
  const BridgeHeader &header) {
  for (auto proto : proto_list_) {
    if (proto->IsTheProto(header)) {
      return proto;
    }
  }
  BridgeProtoBuf<T> *proto_buf = new BridgeProtoBuf<T>;
  if (!proto_buf) {
    return nullptr;
  }
  proto_buf->Initialize(header);
  proto_list_.push_back(proto_buf);
  return proto_buf;
}

template <typename T>
bool UDPBridgeReceiverComponent<T>::IsProtoExist(const BridgeHeader &header) {
  for (auto proto : proto_list_) {
    if (proto->IsTheProto(header)) {
      return true;
    }
  }
  return false;
}

BRIDGE_RECV_IMPL(canbus::Chassis);
}  // namespace bridge
}  // namespace apollo
