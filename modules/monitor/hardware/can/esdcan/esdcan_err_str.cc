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

#include "modules/monitor/hardware/can/esdcan/esdcan_err_str.h"

namespace apollo {
namespace monitor {
namespace hw {

const char *esdcan_err_to_str(NTCAN_RESULT ntstatus) {
  switch (ntstatus) {
    case NTCAN_SUCCESS:
      return "NTCAN_SUCCESS";
    case NTCAN_RX_TIMEOUT:
      return "NTCAN_RX_TIMEOUT";
    case NTCAN_TX_TIMEOUT:
      return "NTCAN_TX_TIMEOUT";
    case NTCAN_TX_ERROR:
      return "NTCAN_TX_ERROR";
    case NTCAN_CONTR_OFF_BUS:
      return "NTCAN_CONTR_OFF_BUS";
    case NTCAN_CONTR_BUSY:
      return "NTCAN_CONTR_BUSY";
    case NTCAN_CONTR_WARN:
      return "NTCAN_CONTR_WARN";
    case NTCAN_NO_ID_ENABLED:
      return "NTCAN_NO_ID_ENABLED";
    case NTCAN_ID_ALREADY_ENABLED:
      return "NTCAN_ID_ALREADY_ENABLED";
    case NTCAN_ID_NOT_ENABLED:
      return "NTCAN_ID_NOT_ENABLED";
    case NTCAN_INVALID_FIRMWARE:
      return "NTCAN_INVALID_FIRMWARE";
    case NTCAN_MESSAGE_LOST:
      return "NTCAN_MESSAGE_LOST";
    case NTCAN_INVALID_PARAMETER:
      return "NTCAN_INVALID_PARAMETER";
    case NTCAN_INVALID_HANDLE:
      return "NTCAN_INVALID_HANDLE";
    case NTCAN_NET_NOT_FOUND:
      return "NTCAN_NET_NOT_FOUND";
#ifdef NTCAN_IO_INCOMPLETE
    case NTCAN_IO_INCOMPLETE:
      return "NTCAN_IO_INCOMPLETE";
#endif
#ifdef NTCAN_IO_PENDING
    case NTCAN_IO_PENDING:
      return "NTCAN_IO_PENDING";
#endif
#ifdef NTCAN_INVALID_HARDWARE
    case NTCAN_INVALID_HARDWARE:
      return "NTCAN_INVALID_HARDWARE";
#endif
#ifdef NTCAN_PENDING_WRITE
    case NTCAN_PENDING_WRITE:
      return "NTCAN_PENDING_WRITE";
#endif
#ifdef NTCAN_PENDING_READ
    case NTCAN_PENDING_READ:
      return "NTCAN_PENDING_READ";
#endif
#ifdef NTCAN_INVALID_DRIVER
    case NTCAN_INVALID_DRIVER:
      return "NTCAN_INVALID_DRIVER";
#endif
#ifdef NTCAN_OPERATION_ABORTED
    case NTCAN_OPERATION_ABORTED:
      return "NTCAN_OPERATION_ABORTED";
#endif
#ifdef NTCAN_WRONG_DEVICE_STATE
    case NTCAN_WRONG_DEVICE_STATE:
      return "NTCAN_WRONG_DEVICE_STATE";
#endif
    case NTCAN_INSUFFICIENT_RESOURCES:
      return "NTCAN_INSUFFICIENT_RESOURCES";
#ifdef NTCAN_HANDLE_FORCED_CLOSE
    case NTCAN_HANDLE_FORCED_CLOSE:
      return "NTCAN_HANDLE_FORCED_CLOSE";
#endif
#ifdef NTCAN_NOT_IMPLEMENTED
    case NTCAN_NOT_IMPLEMENTED:
      return "NTCAN_NOT_IMPLEMENTED";
#endif
#ifdef NTCAN_NOT_SUPPORTED
    case NTCAN_NOT_SUPPORTED:
      return "NTCAN_NOT_SUPPORTED";
#endif
#ifdef NTCAN_SOCK_CONN_TIMEOUT
    case NTCAN_SOCK_CONN_TIMEOUT:
      return "NTCAN_SOCK_CONN_TIMEOUT";
#endif
#ifdef NTCAN_SOCK_CMD_TIMEOUT
    case NTCAN_SOCK_CMD_TIMEOUT:
      return "NTCAN_SOCK_CMD_TIMEOUT";
#endif
#ifdef NTCAN_SOCK_HOST_NOT_FOUND
    case NTCAN_SOCK_HOST_NOT_FOUND:
      return "NTCAN_SOCK_HOST_NOT_FOUND";
#endif
#ifdef NTCAN_CONTR_ERR_PASSIVE
    case NTCAN_CONTR_ERR_PASSIVE:
      return "NTCAN_CONTR_ERR_PASSIVE";
#endif
#ifdef NTCAN_ERROR_NO_BAUDRATE
    case NTCAN_ERROR_NO_BAUDRATE:
      return "NTCAN_ERROR_NO_BAUDRATE";
#endif
#ifdef NTCAN_ERROR_LOM
    case NTCAN_ERROR_LOM:
      return "NTCAN_ERROR_LOM";
#endif
    default:
      return "NTCAN_UNKNOWN";
  }
}

}  // namespace hw
}  // namespace monitor
}  // namespace apollo
