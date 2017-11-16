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

#include "modules/monitor/hardware/can/can_monitor.h"

#include "modules/canbus/common/canbus_gflags.h"
#include "modules/canbus/proto/canbus_conf.pb.h"
#include "modules/common/log.h"
#include "modules/common/util/file.h"
#include "modules/monitor/hardware/can/can_checker_factory.h"

DEFINE_string(can_monitor_name, "CAN", "Name of the CAN monitor.");
DEFINE_double(can_monitor_interval, 3, "CAN status checking interval (s).");

namespace apollo {
namespace monitor {

using apollo::canbus::CanbusConf;

CanMonitor::CanMonitor(SystemStatus *system_status)
    : HardwareMonitor(FLAGS_can_monitor_name, FLAGS_can_monitor_interval,
                      system_status) {
}

void CanMonitor::RunOnce(const double current_time) {
  CanbusConf canbus_conf;

  CHECK(apollo::common::util::GetProtoFromFile(FLAGS_canbus_conf_file,
                                               &canbus_conf));

  auto *can_chk_factory = CanCheckerFactory::instance();
  can_chk_factory->RegisterCanCheckers();
  auto can_chk =
      can_chk_factory->CreateCanChecker(canbus_conf.can_card_parameter());

  if (can_chk == nullptr) {
    return;
  }

  std::vector<HwCheckResult> can_rslt;
  can_chk->run_check(&can_rslt);
  CHECK_EQ(can_rslt.size(), 1);

  status_->set_status(static_cast<HardwareStatus::Status>(can_rslt[0].status));
  status_->set_msg(can_rslt[0].mssg);

  ADEBUG << "Done checking " << name_ << ", status=" << status_->status();
}

}  // namespace monitor
}  // namespace apollo
