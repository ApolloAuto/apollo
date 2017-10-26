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

#include "modules/monitor/hwmonitor/hw/esdcan/esdcan_checker.h"

#include <utility>
#include <vector>

#include "modules/common/util/string_util.h"
#include "modules/monitor/hwmonitor/hw/esdcan/esdcan_err_str.h"
#include "modules/monitor/hwmonitor/hw/hw_log_module.h"

namespace apollo {
namespace monitor {
namespace hw {

const char EsdCanChecker::ESD_CAN_NAME[] = "ESD_CAN";

EsdCanChecker::EsdCanChecker() {
  name_ = apollo::common::util::StrCat(ESD_CAN_NAME, "-", can_id_);
}

HardwareStatus::Status EsdCanChecker::esdcan_result_to_hw_status(
    NTCAN_RESULT ntstatus) {
  // @todo: device not present detection in esd_can_test.
  return ntstatus == NTCAN_SUCCESS ? HardwareStatus::OK : HardwareStatus::ERR;
}

std::string EsdCanChecker::esdcan_result_to_message(NTCAN_RESULT ntstatus) {
  return ntstatus == NTCAN_SUCCESS ? std::string("OK")
                                   : std::string(esdcan_err_to_str(ntstatus));
}

void EsdCanChecker::run_check(std::vector<HwCheckResult> *results) {
  PLATFORM_DBG(get_log_module(), log::LVL_INFO, "To check ESD-CAN-%d", can_id_);

  EsdCanDetails *details = new EsdCanDetails();
  NTCAN_RESULT result = details->esdcan_do_test(can_id_);

  HwCheckResult rslt(name_, esdcan_result_to_hw_status(result), details,
                     std::move(esdcan_result_to_message(result)));

  results->emplace_back(std::move(rslt));
}

}  // namespace hw
}  // namespace monitor
}  // namespace apollo
