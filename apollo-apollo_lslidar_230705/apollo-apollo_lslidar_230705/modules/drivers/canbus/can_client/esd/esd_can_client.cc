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
 * @file esd_can_client.cc
 * @brief the encapsulate call the api of esd can card according to can_client.h
 *interface
 **/

#include "modules/drivers/canbus/can_client/esd/esd_can_client.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/sensor_gflags.h"

namespace apollo {
namespace drivers {
namespace canbus {
namespace can {

using apollo::common::ErrorCode;

bool EsdCanClient::Init(const CANCardParameter &parameter) {
  if (!parameter.has_channel_id()) {
    AERROR << "Init CAN failed: parameter does not have channel id. The "
              "parameter is "
           << parameter.DebugString();
    return false;
  }
  port_ = parameter.channel_id();

  int num_ports = parameter.num_ports();

  if (port_ > num_ports || port_ < 0) {
    AERROR << "Can port number [" << port_ << "] is out of range [0, "
           << num_ports << ") !";
    return false;
  }

  return true;
}

EsdCanClient::~EsdCanClient() {
  if (dev_handler_) {
    Stop();
  }
}

ErrorCode EsdCanClient::Start() {
  if (is_started_) {
    return ErrorCode::OK;
  }

  // open device
  // guss net is the device minor number, if one card is 0,1
  // if more than one card, when install driver u can specify the minior id
  // int32_t ret = canOpen(net, pCtx->mode, txbufsize, rxbufsize, 0, 0,
  // &dev_handler_);
  uint32_t mode = 0;

  if (FLAGS_esd_can_extended_frame) {
    mode = NTCAN_MODE_NO_RTR;
  }

  // mode |= NTCAN_MODE_NO_RTR;
  int32_t ret = canOpen(port_, mode, NTCAN_MAX_TX_QUEUESIZE,
                        NTCAN_MAX_RX_QUEUESIZE, 5, 5, &dev_handler_);
  if (ret != NTCAN_SUCCESS) {
    AERROR << "open device error code [" << ret << "]: " << GetErrorString(ret);
    return ErrorCode::CAN_CLIENT_ERROR_BASE;
  }

  // init config and state
  // After a CAN handle is created with canOpen() the CAN-ID filter is
  // cleared
  // (no CAN messages
  // will pass the filter). To receive a CAN message with a certain CAN-ID
  // or an
  // NTCAN-Event with
  // a certain Event-ID it is required to enable this ID in the handle
  // filter as
  // otherwise a
  // received  message or event is discarded by the driver for this handle.
  // 1. set receive message_id filter, ie white list

  int32_t id_count = 0x800;
  ret = canIdRegionAdd(dev_handler_, 0, &id_count);

  if (FLAGS_esd_can_extended_frame) {
    id_count = 0x1FFFFFFE;
    ret = canIdRegionAdd(dev_handler_, 0x20000000, &id_count);
  }

  if (ret != NTCAN_SUCCESS) {
    AERROR << "add receive msg id filter error code: " << ret << ", "
           << GetErrorString(ret);
    return ErrorCode::CAN_CLIENT_ERROR_BASE;
  }

  // 2. set baudrate to 500k
  ret = canSetBaudrate(dev_handler_, NTCAN_BAUD_500);
  if (ret != NTCAN_SUCCESS) {
    AERROR << "set baudrate error code: " << ret << ", " << GetErrorString(ret);
    return ErrorCode::CAN_CLIENT_ERROR_BASE;
  }

  is_started_ = true;
  return ErrorCode::OK;
}

void EsdCanClient::Stop() {
  if (is_started_) {
    is_started_ = false;
    int32_t ret = canClose(dev_handler_);
    if (ret != NTCAN_SUCCESS) {
      AERROR << "close error code:" << ret << ", " << GetErrorString(ret);
    } else {
      AINFO << "close esd can ok. port:" << port_;
    }
  }
}

// Synchronous transmission of CAN messages
ErrorCode EsdCanClient::Send(const std::vector<CanFrame> &frames,
                             int32_t *const frame_num) {
  CHECK_NOTNULL(frame_num);
  CHECK_EQ(frames.size(), static_cast<size_t>(*frame_num));

  if (!is_started_) {
    AERROR << "Esd can client has not been initiated! Please init first!";
    return ErrorCode::CAN_CLIENT_ERROR_SEND_FAILED;
  }
  for (size_t i = 0; i < frames.size() && i < MAX_CAN_SEND_FRAME_LEN; ++i) {
    send_frames_[i].id = frames[i].id;
    send_frames_[i].len = frames[i].len;
    std::memcpy(send_frames_[i].data, frames[i].data, frames[i].len);
  }

  // Synchronous transmission of CAN messages
  int32_t ret = canWrite(dev_handler_, send_frames_, frame_num, nullptr);
  if (ret != NTCAN_SUCCESS) {
    AERROR << "send message failed, error code: " << ret << ", "
           << GetErrorString(ret);
    return ErrorCode::CAN_CLIENT_ERROR_BASE;
  }
  return ErrorCode::OK;
}

// buf size must be 8 bytes, every time, we receive only one frame
ErrorCode EsdCanClient::Receive(std::vector<CanFrame> *const frames,
                                int32_t *const frame_num) {
  if (!is_started_) {
    AERROR << "Esd can client is not init! Please init first!";
    return ErrorCode::CAN_CLIENT_ERROR_RECV_FAILED;
  }

  if (*frame_num > MAX_CAN_RECV_FRAME_LEN || *frame_num < 0) {
    AERROR << "recv can frame num not in range[0, " << MAX_CAN_RECV_FRAME_LEN
           << "], frame_num:" << *frame_num;
    // TODO(Authors): check the difference of returning frame_num/error_code
    return ErrorCode::CAN_CLIENT_ERROR_FRAME_NUM;
  }

  const int32_t ret = canRead(dev_handler_, recv_frames_, frame_num, nullptr);
  // rx timeout not log
  if (ret == NTCAN_RX_TIMEOUT) {
    return ErrorCode::OK;
  }
  if (ret != NTCAN_SUCCESS) {
    AERROR << "receive message failed, error code: " << ret << ", "
           << GetErrorString(ret);
    return ErrorCode::CAN_CLIENT_ERROR_BASE;
  }

  for (int32_t i = 0; i < *frame_num && i < MAX_CAN_RECV_FRAME_LEN; ++i) {
    CanFrame cf;
    cf.id = recv_frames_[i].id;
    cf.len = recv_frames_[i].len;
    std::memcpy(cf.data, recv_frames_[i].data, recv_frames_[i].len);
    frames->push_back(cf);
  }

  return ErrorCode::OK;
}

/************************************************************************/
/************************************************************************/
/* Function: GetErrorString()                                            */
/* Return ASCII representation of NTCAN return code                     */
/************************************************************************/
/************************************************************************/
const int32_t ERROR_BUF_SIZE = 200;
std::string EsdCanClient::GetErrorString(const NTCAN_RESULT ntstatus) {
  struct ERR2STR {
    NTCAN_RESULT ntstatus;
    const char *str;
  };

  int8_t str_buf[ERROR_BUF_SIZE];

  static const struct ERR2STR err2str[] = {
      {NTCAN_SUCCESS, "NTCAN_SUCCESS"},
      {NTCAN_RX_TIMEOUT, "NTCAN_RX_TIMEOUT"},
      {NTCAN_TX_TIMEOUT, "NTCAN_TX_TIMEOUT"},
      {NTCAN_TX_ERROR, "NTCAN_TX_ERROR"},
      {NTCAN_CONTR_OFF_BUS, "NTCAN_CONTR_OFF_BUS"},
      {NTCAN_CONTR_BUSY, "NTCAN_CONTR_BUSY"},
      {NTCAN_CONTR_WARN, "NTCAN_CONTR_WARN"},
      {NTCAN_NO_ID_ENABLED, "NTCAN_NO_ID_ENABLED"},
      {NTCAN_ID_ALREADY_ENABLED, "NTCAN_ID_ALREADY_ENABLED"},
      {NTCAN_ID_NOT_ENABLED, "NTCAN_ID_NOT_ENABLED"},
      {NTCAN_INVALID_FIRMWARE, "NTCAN_INVALID_FIRMWARE"},
      {NTCAN_MESSAGE_LOST, "NTCAN_MESSAGE_LOST"},
      {NTCAN_INVALID_PARAMETER, "NTCAN_INVALID_PARAMETER"},
      {NTCAN_INVALID_HANDLE, "NTCAN_INVALID_HANDLE"},
      {NTCAN_NET_NOT_FOUND, "NTCAN_NET_NOT_FOUND"},
#ifdef NTCAN_IO_INCOMPLETE
      {NTCAN_IO_INCOMPLETE, "NTCAN_IO_INCOMPLETE"},
#endif
#ifdef NTCAN_IO_PENDING
      {NTCAN_IO_PENDING, "NTCAN_IO_PENDING"},
#endif
#ifdef NTCAN_INVALID_HARDWARE
      {NTCAN_INVALID_HARDWARE, "NTCAN_INVALID_HARDWARE"},
#endif
#ifdef NTCAN_PENDING_WRITE
      {NTCAN_PENDING_WRITE, "NTCAN_PENDING_WRITE"},
#endif
#ifdef NTCAN_PENDING_READ
      {NTCAN_PENDING_READ, "NTCAN_PENDING_READ"},
#endif
#ifdef NTCAN_INVALID_DRIVER
      {NTCAN_INVALID_DRIVER, "NTCAN_INVALID_DRIVER"},
#endif
#ifdef NTCAN_OPERATION_ABORTED
      {NTCAN_OPERATION_ABORTED, "NTCAN_OPERATION_ABORTED"},
#endif
#ifdef NTCAN_WRONG_DEVICE_STATE
      {NTCAN_WRONG_DEVICE_STATE, "NTCAN_WRONG_DEVICE_STATE"},
#endif
      {NTCAN_INSUFFICIENT_RESOURCES, "NTCAN_INSUFFICIENT_RESOURCES"},
#ifdef NTCAN_HANDLE_FORCED_CLOSE
      {NTCAN_HANDLE_FORCED_CLOSE, "NTCAN_HANDLE_FORCED_CLOSE"},
#endif
#ifdef NTCAN_NOT_IMPLEMENTED
      {NTCAN_NOT_IMPLEMENTED, "NTCAN_NOT_IMPLEMENTED"},
#endif
#ifdef NTCAN_NOT_SUPPORTED
      {NTCAN_NOT_SUPPORTED, "NTCAN_NOT_SUPPORTED"},
#endif
#ifdef NTCAN_SOCK_CONN_TIMEOUT
      {NTCAN_SOCK_CONN_TIMEOUT, "NTCAN_SOCK_CONN_TIMEOUT"},
#endif
#ifdef NTCAN_SOCK_CMD_TIMEOUT
      {NTCAN_SOCK_CMD_TIMEOUT, "NTCAN_SOCK_CMD_TIMEOUT"},
#endif
#ifdef NTCAN_SOCK_HOST_NOT_FOUND
      {NTCAN_SOCK_HOST_NOT_FOUND, "NTCAN_SOCK_HOST_NOT_FOUND"},
#endif
#ifdef NTCAN_CONTR_ERR_PASSIVE
      {NTCAN_CONTR_ERR_PASSIVE, "NTCAN_CONTR_ERR_PASSIVE"},
#endif
#ifdef NTCAN_ERROR_NO_BAUDRATE
      {NTCAN_ERROR_NO_BAUDRATE, "NTCAN_ERROR_NO_BAUDRATE"},
#endif
#ifdef NTCAN_ERROR_LOM
      {NTCAN_ERROR_LOM, "NTCAN_ERROR_LOM"},
#endif
      {(NTCAN_RESULT)0xffffffff, "NTCAN_UNKNOWN"} /* stop-mark */
  };

  const struct ERR2STR *es = err2str;

  do {
    if (es->ntstatus == ntstatus) {
      break;
    }
    es++;
  } while ((uint32_t)es->ntstatus != 0xffffffff);

#ifdef NTCAN_ERROR_FORMAT_LONG
  {
    NTCAN_RESULT res;
    char sz_error_text[60];

    res = canFormatError(ntstatus, NTCAN_ERROR_FORMAT_LONG, sz_error_text,
                         static_cast<uint32_t>(sizeof(sz_error_text) - 1));
    if (NTCAN_SUCCESS == res) {
      snprintf(reinterpret_cast<char *>(str_buf), ERROR_BUF_SIZE, "%s - %s",
               es->str, sz_error_text);
    } else {
      snprintf(reinterpret_cast<char *>(str_buf), ERROR_BUF_SIZE, "%s(0x%08x)",
               es->str, ntstatus);
    }
  }
#else
  snprintf(reinterpret_cast<char *>(str_buf), ERROR_BUF_SIZE, "%s(0x%08x)",
           es->str, ntstatus);
#endif /* of NTCAN_ERROR_FORMAT_LONG */

  return std::string((const char *)(str_buf));
}

}  // namespace can
}  // namespace canbus
}  // namespace drivers
}  // namespace apollo
