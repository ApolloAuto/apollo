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

#include "modules/monitor/hwmonitor/hw/socketcan/socketcan_checker.h"

#include <utility>
#include <vector>

#include "modules/common/util/string_util.h"
#include "modules/monitor/hwmonitor/hw/hw_log_module.h"

namespace apollo {
namespace monitor {
namespace hw {

const char SocketCanChecker::SOCKET_CAN_NAME[] = "SOCKET_CAN";

SocketCanChecker::SocketCanChecker() {
  name_ = apollo::common::util::StrCat(SOCKET_CAN_NAME, "-", can_id_);
}

HardwareStatus::Status SocketCanChecker::socketcan_result_to_hw_status(
    int socketcan_status) {
  // @todo: device not present detection in socket_can_test.
  return socketcan_status == 0 ? HardwareStatus::OK : HardwareStatus::ERR;
}

std::string SocketCanChecker::socketcan_result_to_message(
    int socketcan_status) {
  return socketcan_status == 0 ? std::string("OK") : std::string("Failed");
}

void SocketCanChecker::run_check(std::vector<HwCheckResult> *results) {
  PLATFORM_DBG(get_log_module(), log::LVL_INFO, "To check SOCKET-CAN-%d",
               can_id_);

  int result = socketcan_do_test(can_id_);

  HwCheckResult rslt(name_, socketcan_result_to_hw_status(result),
                     std::move(socketcan_result_to_message(result)));

  results->emplace_back(std::move(rslt));
}

}  // namespace hw
}  // namespace monitor
}  // namespace apollo
