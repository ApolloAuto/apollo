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

#include "modules/drivers/canbus/can_client/hermes_can/hermes_can_client.h"

#include <cstdio>
#include <cstring>
#include <iostream>
#include <vector>

namespace apollo {
namespace drivers {
namespace canbus {
namespace can {

using apollo::common::ErrorCode;

HermesCanClient::~HermesCanClient() {
  if (_dev_handler) {
    Stop();
  }
}

bool HermesCanClient::Init(const CANCardParameter &parameter) {
  if (!parameter.has_channel_id()) {
    AERROR << "Init CAN failed: parameter does not have channel id. The "
              "parameter is "
           << parameter.DebugString();
    return false;
  } else {
    _card_port = parameter.channel_id();
    return true;
  }
}

ErrorCode HermesCanClient::Start() {
  if (_is_init) {
    return ErrorCode::OK;
  }

  if (_card_port > MAX_CAN_PORT || _card_port < 0) {
    AERROR << "can port number [" << _card_port << "] is out of the range [0,"
           << MAX_CAN_PORT << "]";
    return ErrorCode::CAN_CLIENT_ERROR_BASE;
  }
  // open device
  int32_t ret = bcan_open(_card_port, 0,
                          5,  // 5ms for rx timeout
                          5,  // 5ms for tx timeout
                          &_dev_handler);

  if (ret != ErrorCode::OK) {
    AERROR << "Open device error code: " << ret
           << ", channel id: " << _card_port;
    return ErrorCode::CAN_CLIENT_ERROR_BASE;
  }
  AINFO << "Open device succ code: " << ret << ", channel id: " << _card_port;

  // 1. set baudrate to 500k
  ret = bcan_set_baudrate(_dev_handler, BCAN_BAUDRATE_500K);
  if (ret != ErrorCode::OK) {
    AERROR << "Set baudrate error Code: " << ret;
    return ErrorCode::CAN_CLIENT_ERROR_BASE;
  }

  // 2. start receive
  ret = bcan_start(_dev_handler);
  if (ret != ErrorCode::OK) {
    AERROR << "Start hermes can card failed: " << ret;
    return ErrorCode::CAN_CLIENT_ERROR_BASE;
  }

  _is_init = true;
  return ErrorCode::OK;
}

void HermesCanClient::Stop() {
  if (_is_init) {
    _is_init = false;
    int32_t ret = bcan_close(_dev_handler);
    if (ret != ErrorCode::OK) {
      AERROR << "close error code: " << ret;
    }
  }
}

// Synchronous transmission of CAN messages
apollo::common::ErrorCode HermesCanClient::Send(
    const std::vector<CanFrame> &frames, int32_t *const frame_num) {
  /*
  typedef struct bcan_msg {
      uint32_t bcan_msg_id;        // source CAN node id
      uint8_t  bcan_msg_datalen;   // message data length
      uint8_t  bcan_msg_rsv[3];    // reserved
      uint8_t  bcan_msg_data[8];   // message data
      uint64_t bcan_msg_timestamp; // TBD
  } bcan_msg_t;
  */
  CHECK_NOTNULL(frame_num);
  CHECK_EQ(frames.size(), static_cast<size_t>(*frame_num));

  if (!_is_init) {
    AERROR << "Hermes can client is not init! Please init first!";
    return ErrorCode::CAN_CLIENT_ERROR_SEND_FAILED;
  }
  //    if (*frame_num > MAX_CAN_SEND_FRAME_LEN || *frame_num < 0) {
  //       AERROR << "send can frame num not in range[0, "
  //         << MAX_CAN_SEND_FRAME_LEN << "], frame_num:" << *frame_num;
  //       return ErrorCode::CAN_CLIENT_ERROR_FRAME_NUM;
  //    }
  for (int i = 0; i < *frame_num; ++i) {
    _send_frames[i].bcan_msg_id = frames[i].id;
    _send_frames[i].bcan_msg_datalen = frames[i].len;
    memcpy(_send_frames[i].bcan_msg_data, frames[i].data, frames[i].len);
  }

  // Synchronous transmission of CAN messages
  int32_t send_num = *frame_num;
  int32_t ret = bcan_send(_dev_handler, _send_frames, send_num);
  if (ret < 0) {
    int ret_send_error = bcan_get_status(_dev_handler);
    AERROR << "send message failed, error code: " << ret
           << ", send error: " << ret_send_error;
    return ErrorCode::CAN_CLIENT_ERROR_SEND_FAILED;
  }
  *frame_num = ret;
  return ErrorCode::OK;
}

// buf size must be 8 bytes, every time, we receive only one frame
const int RX_TIMEOUT = -7;
apollo::common::ErrorCode HermesCanClient::Receive(
    std::vector<CanFrame> *const frames, int32_t *const frame_num) {
  if (!_is_init) {
    AERROR << "Hermes can client is not init! Please init first!";
    return ErrorCode::CAN_CLIENT_ERROR_RECV_FAILED;
  }
  if (*frame_num > MAX_CAN_RECV_FRAME_LEN || *frame_num < 0) {
    AERROR << "recv can frame num not in range[0, " << MAX_CAN_RECV_FRAME_LEN
           << "], frame_num:" << *frame_num;
    return ErrorCode::CAN_CLIENT_ERROR_FRAME_NUM;
  }

  int32_t ret = bcan_recv(_dev_handler, _recv_frames, *frame_num);
  // don't log timeout
  if (ret == RX_TIMEOUT) {
    *frame_num = 0;
    return ErrorCode::OK;
  }
  if (ret < 0) {
    int ret_rece_error = bcan_get_status(_dev_handler);
    AERROR << "receive message failed, error code:" << ret
           << "receive error:" << ret_rece_error;
    return ErrorCode::CAN_CLIENT_ERROR_RECV_FAILED;
  }
  *frame_num = ret;

  // is ret num is equal *frame_num?
  for (int i = 0; i < *frame_num; ++i) {
    CanFrame cf;
    cf.id = _recv_frames[i].bcan_msg_id;
    cf.len = _recv_frames[i].bcan_msg_datalen;
    cf.timestamp.tv_sec = _recv_frames[i].bcan_msg_timestamp.tv_sec;
    cf.timestamp.tv_usec = _recv_frames[i].bcan_msg_timestamp.tv_usec;
    memcpy(cf.data, _recv_frames[i].bcan_msg_data, cf.len);
    frames->push_back(cf);
  }

  return ErrorCode::OK;
}

std::string HermesCanClient::GetErrorString(int32_t ntstatus) { return ""; }

void HermesCanClient::SetInited(bool init) { _is_init = init; }

}  // namespace can
}  // namespace canbus
}  // namespace drivers
}  // namespace apollo

/* vim: set expandtab ts=4 sw=4 sts=4 tw=100: */
