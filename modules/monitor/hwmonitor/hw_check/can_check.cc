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

#include <assert.h>
#include <iostream>

#include "modules/canbus/common/canbus_gflags.h"
#include "modules/canbus/proto/canbus_conf.pb.h"
#include "modules/common/log.h"
#include "modules/common/util/file.h"
#include "modules/hmi/utils/hmi_status_helper.h"
#include "modules/monitor/common/annotations.h"
#include "modules/monitor/common/can_checker_factory.h"
#include "modules/monitor/hwmonitor/hw_check/hw_chk_utils.h"

using ::apollo::canbus::CanbusConf;
using ::apollo::hmi::HMIStatusHelper;
using ::apollo::hmi::HardwareStatus;
using ::apollo::monitor::CanCheckerFactory;
using ::apollo::monitor::HwCheckResult;

int main(int argc, const char *argv[]) {
  // For other modules that uses glog.
  google::InitGoogleLogging("platform");

  CanbusConf canbus_conf;

  if (!::apollo::common::util::GetProtoFromFile(FLAGS_canbus_conf_file,
                                                &canbus_conf)) {
    return -1;
  }

  auto *can_chk_factory = CanCheckerFactory::instance();
  can_chk_factory->RegisterCanCheckers();
  auto can_chk =
      can_chk_factory->CreateCanChecker(canbus_conf.can_card_parameter());
  if (!can_chk) {
    return -1;
  }

  std::vector<HwCheckResult> can_rslt;
  can_chk->run_check(&can_rslt);
  assert(can_rslt.size() == 1);

#ifdef DEBUG
  if (can_rslt[0].details != nullptr) {
    can_rslt[0].details->print_summary(std::cout);
  }
#else
  ADEBUG << "Done checking " << can_chk->get_name() << ", "
            "status: " << can_rslt[0].status;
#endif

  std::vector<HardwareStatus> hw_status;
  apollo::monitor::hw::hw_chk_result_to_hmi_status(can_rslt, &hw_status);
  HMIStatusHelper::ReportHardwareStatus(hw_status);

  return 0;
}
