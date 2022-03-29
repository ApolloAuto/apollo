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

#include "modules/monitor/hardware/esdcan_monitor.h"

#include <string>

#if USE_ESD_CAN
#include "esd_can/include/ntcan.h"
#endif

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/common/util/map_util.h"
#include "modules/monitor/common/monitor_manager.h"
#include "modules/monitor/software/summary_monitor.h"

DEFINE_int32(esdcan_id, 0, "ESD CAN id.");
DEFINE_string(esdcan_monitor_name, "EsdCanMonitor", "Name of the CAN monitor.");
DEFINE_double(esdcan_monitor_interval, 3, "CAN status checking interval (s).");
DEFINE_string(esdcan_component_name, "ESD-CAN",
              "Name of the ESD CAN component in SystemStatus.");

namespace apollo {
namespace monitor {
namespace {

#if USE_ESD_CAN
std::string StatusString(const NTCAN_RESULT ntstatus) {
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
    case NTCAN_INSUFFICIENT_RESOURCES:
      return "NTCAN_INSUFFICIENT_RESOURCES";
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
      break;
  }
  return "NTCAN_UNKNOWN";
}

NTCAN_RESULT EsdCanTest(const int can_id, NTCAN_HANDLE* handle) {
  NTCAN_RESULT ret = canOpen(can_id, 0, 1, 1, 0, 0, handle);
  if (ret == NTCAN_SUCCESS) {
    AINFO << "Successfully opened ESD-CAN device " << can_id;
  } else {
    AERROR << "Failed to open ESD-CAN device " << can_id << ", error: " << ret
           << " (" << StatusString(ret) << ")";
    return ret;
  }

  CAN_IF_STATUS if_status;
  ret = canStatus(*handle, &if_status);
  if (ret != NTCAN_SUCCESS) {
    AERROR << "Cannot get status of ESD-CAN, ret=" << ret << " ("
           << StatusString(ret) << ")";
    return ret;
  }

  NTCAN_BUS_STATISTIC stats;
  ret = canIoctl(*handle, NTCAN_IOCTL_GET_BUS_STATISTIC, &stats);
  if (ret != NTCAN_SUCCESS) {
    AERROR << "NTCAN_IOCTL_GET_BUS_STATISTIC failed for device with error: "
           << ret << " (" << StatusString(ret) << ")";
    return ret;
  }

  NTCAN_CTRL_STATE ctrl_state;
  ret = canIoctl(*handle, NTCAN_IOCTL_GET_CTRL_STATUS, &ctrl_state);
  if (ret != NTCAN_SUCCESS) {
    AERROR << "NTCAN_IOCTL_GET_CTRL_STATUS failed for device with error: "
           << ret << " (" << StatusString(ret) << ")";
    return ret;
  }

  NTCAN_BITRATE bitrate;
  ret = canIoctl(*handle, NTCAN_IOCTL_GET_BITRATE_DETAILS, &bitrate);
  if (ret != NTCAN_SUCCESS) {
    AERROR << "NTCAN_IOCTL_GET_BITRATE_ for device with error: " << ret << " ("
           << StatusString(ret) << ")";
    return ret;
  }
  return ret;
}

void EsdCanTest(const int can_id, ComponentStatus* status) {
  NTCAN_HANDLE handle;
  const NTCAN_RESULT ret = EsdCanTest(can_id, &handle);
  canClose(handle);

  SummaryMonitor::EscalateStatus(
      ret == NTCAN_SUCCESS ? ComponentStatus::OK : ComponentStatus::ERROR,
      StatusString(ret), status);
}
#else
// USE_ESD_CAN is not set, do dummy check.
void EsdCanTest(const int can_id, ComponentStatus* status) {
  SummaryMonitor::EscalateStatus(ComponentStatus::ERROR,
                                 "USE_ESD_CAN is not defined during compiling",
                                 status);
}
#endif

}  // namespace

EsdCanMonitor::EsdCanMonitor()
    : RecurrentRunner(FLAGS_esdcan_monitor_name,
                      FLAGS_esdcan_monitor_interval) {}

void EsdCanMonitor::RunOnce(const double current_time) {
  Component* component = apollo::common::util::FindOrNull(
      *MonitorManager::Instance()->GetStatus()->mutable_components(),
      FLAGS_esdcan_component_name);
  if (component == nullptr) {
    // Canbus is not monitored in current mode, skip.
    return;
  }

  auto* status = component->mutable_other_status();
  status->clear_status();
  EsdCanTest(FLAGS_esdcan_id, status);
}

}  // namespace monitor
}  // namespace apollo
