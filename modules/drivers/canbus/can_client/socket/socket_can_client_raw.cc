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

/**
 * @file socket_can_client.cc
 * @brief the encapsulate call the api of socket can card according to
 *can_client.h interface
 **/

#include "modules/drivers/canbus/can_client/socket/socket_can_client_raw.h"

#include "absl/strings/str_cat.h"

namespace apollo {
namespace drivers {
namespace canbus {
namespace can {

using apollo::common::ErrorCode;

bool SocketCanClientRaw::Init(const CANCardParameter &parameter) {
  if (!parameter.has_channel_id()) {
    AERROR << "Init CAN failed: parameter does not have channel id. The "
              "parameter is "
           << parameter.DebugString();
    return false;
  }

  port_ = parameter.channel_id();
  interface_ = parameter.interface();
  auto num_ports = parameter.num_ports();
  if (port_ > static_cast<int32_t>(num_ports) || port_ < 0) {
    AERROR << "Can port number [" << port_ << "] is out of range [0, "
           << num_ports << ") !";
    return false;
  }
  return true;
}

SocketCanClientRaw::~SocketCanClientRaw() {
  if (dev_handler_) {
    Stop();
  }
}

ErrorCode SocketCanClientRaw::Start() {
  if (is_started_) {
    return ErrorCode::OK;
  }
  struct sockaddr_can addr;
  struct ifreq ifr;

  // open device
  // guss net is the device minor number, if one card is 0,1
  // if more than one card, when install driver u can specify the minior id
  // int32_t ret = canOpen(net, pCtx->mode, txbufsize, rxbufsize, 0, 0,
  // &dev_handler_);
  dev_handler_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (dev_handler_ < 0) {
    AERROR << "open device error code [" << dev_handler_ << "]: ";
    return ErrorCode::CAN_CLIENT_ERROR_BASE;
  }

  // init config and state
  int ret;

  // 1. for non virtual busses, set receive message_id filter, ie white list
  if (interface_ != CANCardParameter::VIRTUAL) {
    struct can_filter filter[2048];
    for (int i = 0; i < 2048; ++i) {
      filter[i].can_id = 0x000 + i;
      filter[i].can_mask = CAN_SFF_MASK;
    }

    ret = setsockopt(dev_handler_, SOL_CAN_RAW, CAN_RAW_FILTER, &filter,
                     sizeof(filter));
    if (ret < 0) {
      AERROR << "add receive msg id filter error code: " << ret;
      return ErrorCode::CAN_CLIENT_ERROR_BASE;
    }
  }

  // 2. enable reception of can frames.
  int enable = 1;
  ret = ::setsockopt(dev_handler_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable,
                     sizeof(enable));
  if (ret < 0) {
    AERROR << "enable reception of can frame error code: " << ret;
    return ErrorCode::CAN_CLIENT_ERROR_BASE;
  }

  std::string interface_prefix;
  if (interface_ == CANCardParameter::VIRTUAL) {
    interface_prefix = "vcan";
  } else if (interface_ == CANCardParameter::SLCAN) {
    interface_prefix = "slcan";
  } else {  // default: CANCardParameter::NATIVE
    interface_prefix = "can";
  }

  const std::string can_name = absl::StrCat(interface_prefix, port_);
  std::strncpy(ifr.ifr_name, can_name.c_str(), IFNAMSIZ);
  if (ioctl(dev_handler_, SIOCGIFINDEX, &ifr) < 0) {
    AERROR << "ioctl error";
    return ErrorCode::CAN_CLIENT_ERROR_BASE;
  }

  // bind socket to network interface

  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  ret = ::bind(dev_handler_, reinterpret_cast<struct sockaddr *>(&addr),
               sizeof(addr));

  if (ret < 0) {
    AERROR << "bind socket to network interface error code: " << ret;
    return ErrorCode::CAN_CLIENT_ERROR_BASE;
  }

  is_started_ = true;
  return ErrorCode::OK;
}

void SocketCanClientRaw::Stop() {
  if (is_started_) {
    is_started_ = false;

    int ret = close(dev_handler_);
    if (ret < 0) {
      AERROR << "close error code:" << ret << ", " << GetErrorString(ret);
    } else {
      AINFO << "close socket can ok. port:" << port_;
    }
  }
}

// Synchronous transmission of CAN messages
ErrorCode SocketCanClientRaw::Send(const std::vector<CanFrame> &frames,
                                   int32_t *const frame_num) {
  CHECK_NOTNULL(frame_num);
  CHECK_EQ(frames.size(), static_cast<size_t>(*frame_num));

  if (!is_started_) {
    AERROR << "Nvidia can client has not been initiated! Please init first!";
    return ErrorCode::CAN_CLIENT_ERROR_SEND_FAILED;
  }
  for (size_t i = 0; i < frames.size() && i < MAX_CAN_SEND_FRAME_LEN; ++i) {
    if (frames[i].len > CANBUS_MESSAGE_LENGTH || frames[i].len < 0) {
      AERROR << "frames[" << i << "].len = " << frames[i].len
             << ", which is not equal to can message data length ("
             << CANBUS_MESSAGE_LENGTH << ").";
      return ErrorCode::CAN_CLIENT_ERROR_SEND_FAILED;
    }
    send_frames_[i].can_id = frames[i].id;
    send_frames_[i].can_dlc = frames[i].len;
    std::memcpy(send_frames_[i].data, frames[i].data, frames[i].len);

    // Synchronous transmission of CAN messages
    int ret = static_cast<int>(
        write(dev_handler_, &send_frames_[i], sizeof(send_frames_[i])));
    if (ret <= 0) {
      AERROR << "send message failed, error code: " << ret;
      return ErrorCode::CAN_CLIENT_ERROR_BASE;
    }
  }

  return ErrorCode::OK;
}

// buf size must be 8 bytes, every time, we receive only one frame
ErrorCode SocketCanClientRaw::Receive(std::vector<CanFrame> *const frames,
                                      int32_t *const frame_num) {
  if (!is_started_) {
    AERROR << "Nvidia can client is not init! Please init first!";
    return ErrorCode::CAN_CLIENT_ERROR_RECV_FAILED;
  }

  if (*frame_num > MAX_CAN_RECV_FRAME_LEN || *frame_num < 0) {
    AERROR << "recv can frame num not in range[0, " << MAX_CAN_RECV_FRAME_LEN
           << "], frame_num:" << *frame_num;
    // TODO(Authors): check the difference of returning frame_num/error_code
    return ErrorCode::CAN_CLIENT_ERROR_FRAME_NUM;
  }

  for (int32_t i = 0; i < *frame_num && i < MAX_CAN_RECV_FRAME_LEN; ++i) {
    CanFrame cf;
    auto ret = read(dev_handler_, &recv_frames_[i], sizeof(recv_frames_[i]));

    if (ret < 0) {
      AERROR << "receive message failed, error code: " << ret;
      return ErrorCode::CAN_CLIENT_ERROR_BASE;
    }
    if (recv_frames_[i].can_dlc > CANBUS_MESSAGE_LENGTH ||
        recv_frames_[i].can_dlc < 0) {
      AERROR << "recv_frames_[" << i
             << "].can_dlc = " << recv_frames_[i].can_dlc
             << ", which is not equal to can message data length ("
             << CANBUS_MESSAGE_LENGTH << ").";
      return ErrorCode::CAN_CLIENT_ERROR_RECV_FAILED;
    }
    cf.id = recv_frames_[i].can_id;
    cf.len = recv_frames_[i].can_dlc;
    std::memcpy(cf.data, recv_frames_[i].data, recv_frames_[i].can_dlc);
    frames->push_back(cf);
  }
  return ErrorCode::OK;
}

std::string SocketCanClientRaw::GetErrorString(const int32_t /*status*/) {
  return "";
}

}  // namespace can
}  // namespace canbus
}  // namespace drivers
}  // namespace apollo
