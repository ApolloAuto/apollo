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

#include "esdcan_checker.h"

#include <sstream>

#include "modules/monitor/hwmonitor/hw/hw_log_module.h"
#include "esdcan_err_str.h"

namespace apollo {
namespace platform {
namespace hw {

const std::string EsdCanChecker::ESD_CAN_NAME = "ESD_CAN";

EsdCanChecker::EsdCanChecker(int id) : can_id_(id) {
  std::ostringstream os;
  os << ESD_CAN_NAME << "-" << id;
  name_ = os.str();
}

hw::Status EsdCanChecker::esdcan_result_to_hw_status(NTCAN_RESULT ntstatus) {
  // @todo: device not present detection in esd_can_test.
  return ntstatus == NTCAN_SUCCESS ? hw::Status::OK : hw::Status::ERR;
}

std::string EsdCanChecker::esdcan_result_to_message(NTCAN_RESULT ntstatus) {
  return ntstatus == NTCAN_SUCCESS ? std::string("OK")
                                   : std::string(esdcan_err_to_str(ntstatus));
}

void EsdCanChecker::run_check(std::vector<HwCheckResult> &results) {
  PLATFORM_DBG(get_log_module(), log::LVL_INFO, "To check ESD-CAN-%d", can_id_);

  EsdCanDetails *details = new EsdCanDetails();
  NTCAN_RESULT result = esdcan_do_test(can_id_, details);

  HwCheckResult rslt(name_, esdcan_result_to_hw_status(result), details,
                     std::move(esdcan_result_to_message(result)));

  results.emplace_back(std::move(rslt));
}

}  // namespace hw
}  // namespace platform
}  // namespace apollo
