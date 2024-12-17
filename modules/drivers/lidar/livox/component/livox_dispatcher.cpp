/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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

#include "modules/drivers/lidar/livox/component/livox_dispatcher.h"

#include <string>

namespace apollo {
namespace drivers {
namespace lidar {

bool LivoxDispatcher::GetHandleFromIP(const std::string& ip, uint32_t& handle) {
  const std::regex ip_regex(
      "(\\d{1,3})\\.(\\d{1,3})\\.(\\d{1,3})\\.(\\d{1,3})");
  std::smatch ip_match;
  if (!std::regex_match(ip, ip_match, ip_regex)) {
    AERROR << "ip = " << ip << "; is not a valid ip address";
    return false;
  }

  if (ip_match.size() != 5) {
    AERROR << "ip = " << ip << "; match error";
    return false;
  }
  handle = 0;
  for (int i = 1; i <= 4; ++i) {
    int dig = stoi(ip_match[i]);
    handle = handle + pow(256, i - 1) * dig;
  }
  return true;
}

void LivoxDispatcher::RegisterHandleDispatchCallback(
    uint32_t handle, PointCloudCallbackType cb) {
  AINFO << "register handle " << handle;
  std::lock_guard<std::mutex> lg(mtx);
  handle_callback_functions[handle] = cb;
}

void LivoxDispatcher::LivoxPointCloudCallback(uint32_t handle,
                                              const uint8_t dev_type,
                                              LivoxLidarEthernetPacket* data,
                                              void* client_data) {
  std::lock_guard<std::mutex> lg(mtx);
  if (handle_callback_functions.count(handle)) {
    PointCloudCallbackType cb = handle_callback_functions[handle];
    cb(handle, dev_type, data, client_data);
  }
}

LivoxDispatcher LivoxDispatcher::dispatcher_instance_;

}  // namespace lidar
}  // namespace drivers
}  // namespace apollo
