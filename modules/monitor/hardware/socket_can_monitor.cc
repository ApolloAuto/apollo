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

#include "modules/monitor/hardware/socket_can_monitor.h"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <string>

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/common/util/map_util.h"
#include "modules/monitor/common/monitor_manager.h"
#include "modules/monitor/software/summary_monitor.h"

DEFINE_string(socket_can_monitor_name, "SocketCanMonitor",
              "Name of the CAN monitor.");
DEFINE_double(socket_can_monitor_interval, 3,
              "Socket CAN status checking interval seconds.");
DEFINE_string(socket_can_component_name, "SocketCAN",
              "Name of the Socket CAN component in SystemStatus.");

namespace apollo {
namespace monitor {
namespace {

// Test Socket CAN on an open handler.
bool SocketCanHandlerTest(const int dev_handler, std::string* message) {
  // init config and state
  // 1. set receive message_id filter, ie white list
  struct can_filter filter[1];
  filter[0].can_id = 0x000;
  filter[0].can_mask = CAN_SFF_MASK;

  int ret = setsockopt(dev_handler, SOL_CAN_RAW, CAN_RAW_FILTER, &filter,
                       sizeof(filter));
  if (ret < 0) {
    *message = "set message filter failed";
    return false;
  }

  // 2. enable reception of can frames.
  const int enable = 1;
  ret = setsockopt(dev_handler, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable,
                   sizeof(enable));
  if (ret < 0) {
    *message = "Enable reception of can frames failed";
    return false;
  }

  struct ifreq ifr;
  std::strncpy(ifr.ifr_name, "can0", IFNAMSIZ);
  if (ioctl(dev_handler, SIOCGIFINDEX, &ifr) < 0) {
    *message = "ioctl failed";
    return false;
  }

  // bind socket to network interface
  struct sockaddr_can addr;
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  ret = bind(dev_handler, reinterpret_cast<struct sockaddr*>(&addr),
             sizeof(addr));

  if (ret < 0) {
    *message = "bind socket can failed";
    return false;
  }

  return true;
}

// Open a Socket CAN handler and test.
bool SocketCanTest(std::string* message) {
  const int dev_handler = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (dev_handler < 0) {
    *message = "Open can device failed";
    return false;
  }
  const bool ret = SocketCanHandlerTest(dev_handler, message);
  close(dev_handler);
  return ret;
}

}  // namespace

SocketCanMonitor::SocketCanMonitor()
    : RecurrentRunner(FLAGS_socket_can_monitor_name,
                      FLAGS_socket_can_monitor_interval) {}

void SocketCanMonitor::RunOnce(const double current_time) {
  auto manager = MonitorManager::Instance();
  Component* component = apollo::common::util::FindOrNull(
      *manager->GetStatus()->mutable_components(),
      FLAGS_socket_can_component_name);
  if (component == nullptr) {
    // Canbus is not monitored in current mode, skip.
    return;
  }
  auto* status = component->mutable_other_status();
  status->clear_status();

  std::string message;
  const bool ret = SocketCanTest(&message);
  SummaryMonitor::EscalateStatus(
      ret ? ComponentStatus::OK : ComponentStatus::ERROR, message, status);
}

}  // namespace monitor
}  // namespace apollo
