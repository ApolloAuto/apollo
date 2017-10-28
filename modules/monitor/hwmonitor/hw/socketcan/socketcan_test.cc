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

#include "modules/monitor/hwmonitor/hw/socketcan/socketcan_test.h"

#include "modules/common/log.h"

namespace apollo {
namespace monitor {
namespace hw {

int socketcan_do_test(int id) {
  struct sockaddr_can addr;
  struct ifreq ifr;

  // Check open device

  if (id < 0) {
    AERROR << "can port number " << id << " not valid";
    return -1;
  }

  int dev_handler = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (dev_handler < 0) {
    AERROR << "open can device failed";
    return -1;
  }

  // init config and state
  // 1. set receive message_id filter, ie white list
  struct can_filter filter[1];
  filter[0].can_id = 0x000;
  filter[0].can_mask = CAN_SFF_MASK;

  int ret = setsockopt(dev_handler, SOL_CAN_RAW, CAN_RAW_FILTER, &filter,
                       sizeof(filter));
  if (ret < 0) {
    AERROR << "set message filter failed";
    return -1;
  }

  // 2. enable reception of can frames.
  int enable = 1;
  ret = ::setsockopt(dev_handler, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable,
                     sizeof(enable));
  if (ret < 0) {
    AERROR << "enable reception of can frames failed";
    return -1;
  }

  // strcpy(ifr.ifr_name, "can0");
  std::strncpy(ifr.ifr_name, "can0", IFNAMSIZ);
  if (ioctl(dev_handler, SIOCGIFINDEX, &ifr) < 0) {
    AERROR << "ioctl failed";
    return -1;
  }

  // bind socket to network interface
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  ret = ::bind(dev_handler, reinterpret_cast<struct sockaddr *>(&addr),
               sizeof(addr));

  if (ret < 0) {
    AERROR << "bind socket can failed";
    return -1;
  }

  return 0;
}

}  // namespace hw
}  // namespace monitor
}  // namespace apollo
