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

#include <iostream>

#include "gflags/gflags.h"

#include "modules/monitor/hwmonitor/hw/esdcan/esdcan_test.h"
#include "modules/monitor/hwmonitor/hw/esdcan/esdcan_utils.h"

DEFINE_bool(details, false, "prints detailed stats if true, default is false");
DEFINE_int32(can_id, 0, "can device id, default is 0");

using apollo::monitor::hw::EsdCanDetails;
namespace _hw = apollo::monitor::hw;

int main(int argc, char *argv[]) {
  google::SetUsageMessage(
      std::string(argv[0]) +
      std::string(" [--can_id=#id] [--details=true|false]\n"
                  "    can_id: CAN channel 0, 1, ...; default to use 0\n"
                  "    details: prints detailed stats if true, "
                  "default is false\n"));
  google::ParseCommandLineFlags(&argc, &argv, true);

  EsdCanDetails can_details;
  can_details.esdcan_do_test(FLAGS_can_id);
  if (FLAGS_details) {
    can_details.print_test_result(std::cout);

    if (can_details.valid_flag & EsdCanDetails::IF_STATUS) {
      _hw::esdcan_print_if_status(FLAGS_can_id, can_details.if_status);
    }
    if (can_details.valid_flag & EsdCanDetails::STATS) {
      printf("\n");
      _hw::esdcan_print_stats(can_details.stats);
    }
    if (can_details.valid_flag & EsdCanDetails::CTRL_STATE) {
      printf("\n");
      _hw::esdcan_print_ctrl_state(can_details.ctrl_state);
    }
    if (can_details.valid_flag & EsdCanDetails::BITRATE) {
      printf("\n");
      _hw::esdcan_print_bitrate(can_details.bitrate);
    }
  } else {
    can_details.print_summary(std::cout);
  }

  return 0;
}
