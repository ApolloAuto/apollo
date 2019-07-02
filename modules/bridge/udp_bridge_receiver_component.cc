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
  enable_timeout_ = udp_bridge_remote.enable_timeout();
  AINFO << "UDP Bridge remote port is: " << bind_port_;
  AINFO << "UDP Bridge for Proto is: " << proto_name_;
  writer_ = node_->CreateWriter<T>(topic_name_.c_str());

  if (!InitSession((uint16_t)bind_port_)) {
    return false;
  }
  AINFO << "initialize session successful.";
  MsgDispatcher();
  return true;
}

template <typename T>
bool UDPBridgeReceiverComponent<T>::InitSession(uint16_t port) {
  return listener_->Initialize(this, &UDPBridgeReceiverComponent<T>::MsgHandle,
                               port);
}

template <typename T>
void UDPBridgeReceiverComponent<T>::MsgDispatcher() {
  AINFO << "msg dispatcher start successful.";
  listener_->Listen();
}

template <typename T>
BridgeProtoDiserializedBuf<T>
    *UDPBridgeReceiverComponent<T>::CreateBridgeProtoBuf(
        const BridgeHeader &header) {
  if (IsTimeout(header.GetTimeStamp())) {
    typename std::vector<BridgeProtoDiserializedBuf<T> *>::iterator itor =
        proto_list_.begin();
    for (; itor != proto_list_.end();) {
      if ((*itor)->IsTheProto(header)) {
        BridgeProtoDiserializedBuf<T> *tmp = *itor;
        FREE_POINTER(tmp);
        itor = proto_list_.erase(itor);
        break;
      }
      ++itor;
    }
    return nullptr;
  }

  for (auto proto : proto_list_) {
    if (proto->IsTheProto(header)) {
      return proto;
    }
  }
  BridgeProtoDiserializedBuf<T> *proto_buf = new BridgeProtoDiserializedBuf<T>;
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

template <typename T>
bool UDPBridgeReceiverComponent<T>::IsTimeout(double time_stamp) {
  if (enable_timeout_ == false) {
    return false;
  }
  double cur_time = apollo::common::time::Clock::NowInSeconds();
  if (cur_time < time_stamp) {
    return true;
  }
  if (FLAGS_timeout < cur_time - time_stamp) {
    return true;
  }
  return false;
}

template <typename T>
bool UDPBridgeReceiverComponent<T>::MsgHandle(int fd) {
  struct sockaddr_in client_addr;
  socklen_t sock_len = static_cast<socklen_t>(sizeof(client_addr));
  int bytes = 0;
  int total_recv = 2 * FRAME_SIZE;
  char total_buf[2 * FRAME_SIZE] = {0};
  bytes =
      static_cast<int>(recvfrom(fd, total_buf, total_recv, 0,
                                (struct sockaddr *)&client_addr, &sock_len));
  AINFO << "total recv " << bytes;
  if (bytes <= 0 || bytes > total_recv) {
    return false;
  }
  char header_flag[sizeof(BRIDGE_HEADER_FLAG) + 1] = {0};
  size_t offset = 0;
  memcpy(header_flag, total_buf, HEADER_FLAG_SIZE);
  if (strcmp(header_flag, BRIDGE_HEADER_FLAG) != 0) {
    AINFO << "header flag not match!";
    return false;
  }
  offset += sizeof(BRIDGE_HEADER_FLAG) + 1;

  char header_size_buf[sizeof(hsize) + 1] = {0};
  const char *cursor = total_buf + offset;
  memcpy(header_size_buf, cursor, sizeof(hsize));
  hsize header_size = *(reinterpret_cast<hsize *>(header_size_buf));
  if (header_size > FRAME_SIZE) {
    AINFO << "header size is more than FRAME_SIZE!";
    return false;
  }
  offset += sizeof(hsize) + 1;

  BridgeHeader header;
  size_t buf_size = header_size - offset;
  cursor = total_buf + offset;
  if (!header.Diserialize(cursor, buf_size)) {
    AINFO << "header diserialize failed!";
    return false;
  }

  AINFO << "proto name : " << header.GetMsgName().c_str();
  AINFO << "proto sequence num: " << header.GetMsgID();
  AINFO << "proto total frames: " << header.GetTotalFrames();
  AINFO << "proto frame index: " << header.GetIndex();

  std::lock_guard<std::mutex> lock(mutex_);
  BridgeProtoDiserializedBuf<T> *proto_buf = CreateBridgeProtoBuf(header);
  if (!proto_buf) {
    return false;
  }

  cursor = total_buf + header_size;
  char *buf = proto_buf->GetBuf(header.GetFramePos());
  memcpy(buf, cursor, header.GetFrameSize());
  proto_buf->UpdateStatus(header.GetIndex());
  if (proto_buf->IsReadyDiserialize()) {
    auto pb_msg = std::make_shared<T>();
    proto_buf->Diserialized(pb_msg);
    writer_->Write(pb_msg);
    RemoveInvalidBuf(proto_buf->GetMsgID());
    RemoveItem(&proto_list_, proto_buf);
  }
  return true;
}

template <typename T>
bool UDPBridgeReceiverComponent<T>::RemoveInvalidBuf(uint32_t msg_id) {
  if (msg_id == 0) {
    return false;
  }
  typename std::vector<BridgeProtoDiserializedBuf<T> *>::iterator itor =
      proto_list_.begin();
  for (; itor != proto_list_.end();) {
    if ((*itor)->GetMsgID() < msg_id) {
      BridgeProtoDiserializedBuf<T> *tmp = *itor;
      FREE_POINTER(tmp);
      itor = proto_list_.erase(itor);
      continue;
    }
    ++itor;
  }
  return true;
}

BRIDGE_RECV_IMPL(canbus::Chassis);
}  // namespace bridge
}  // namespace apollo
