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

#include "modules/monitor/hardware/can/esdcan/esdcan_utils.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define __STDC_FORMAT_MACROS
#include <inttypes.h>

#include "modules/monitor/hardware/can/esdcan/esdcan_err_str.h"

namespace apollo {
namespace monitor {
namespace hw {

// @todo: clean up print functions

void esdcan_print_if_status(int id, const CAN_IF_STATUS &if_status) {
  printf(
      "Net %3d: ID=%s\n"
      "         Versions (hex): Dll=%1X.%1X.%02X "
      " Drv=%1X.%1X.%02X"
      " FW=%1X.%1X.%02X"
      " HW=%1X.%1X.%02X\n"
      "         Status=%08x\n",
      id, if_status.boardid, if_status.dll >> 12, (if_status.dll >> 8) & 0xf,
      if_status.dll & 0xff, if_status.driver >> 12,
      (if_status.driver >> 8) & 0xf, if_status.driver & 0xff,
      if_status.firmware >> 12, (if_status.firmware >> 8) & 0xf,
      if_status.firmware & 0xff, if_status.hardware >> 12,
      (if_status.hardware >> 8) & 0xf, if_status.hardware & 0xff,
      (unsigned int)if_status.boardstatus);
}

void esdcan_print_stats(const NTCAN_BUS_STATISTIC &stats) {
  printf("CAN bus statistics:\n");
  printf("Rcv frames      : Std(Data/RTR): %ld/%ld Ext(Data/RTR) %ld/%ld\n",
         static_cast<int64_t>(stats.rcv_count.std_data),
         static_cast<int64_t>(stats.rcv_count.std_rtr),
         static_cast<int64_t>(stats.rcv_count.ext_data),
         static_cast<int64_t>(stats.rcv_count.ext_rtr));
  printf("Xmit frames     : Std(Data/RTR): %ld/%ld Ext(Data/RTR) %ld/%ld\n",
         static_cast<int64_t>(stats.xmit_count.std_data),
         static_cast<int64_t>(stats.xmit_count.std_rtr),
         static_cast<int64_t>(stats.xmit_count.ext_data),
         static_cast<int64_t>(stats.xmit_count.ext_rtr));
  printf("Bytes           : (Rcv/Xmit): %ld/%ld\n",
         static_cast<int64_t>(stats.rcv_byte_count),
         static_cast<int64_t>(stats.xmit_byte_count));
  printf("Overruns        : (Controller/FIFO): %ld/%ld\n",
         static_cast<int64_t>(stats.ctrl_ovr),
         static_cast<int64_t>(stats.fifo_ovr));
  printf("Err frames      : %ld\n", static_cast<int64_t>(stats.err_frames));
  printf("Aborted frames  : %ld\n", static_cast<int64_t>(stats.aborted_frames));
  printf("Rcv bits        : %" PRIu64 "\n", stats.bit_count);
}

void esdcan_print_ctrl_state(const NTCAN_CTRL_STATE &c_state) {
  printf("Err counter     : (Rx/Tx): %d/%d Status: %02x\n",
         c_state.rcv_err_counter, c_state.xmit_err_counter, c_state.status);
}

void esdcan_print_bitrate(const NTCAN_BITRATE &bitrate) {
  printf("CAN bitrate:\n");
  printf("Value set by canSetBaudrate()  : 0x%08lX\n",
         static_cast<int64_t>(bitrate.baud));
  if (NTCAN_SUCCESS == bitrate.valid) {
    printf("Actual Bitrate                 : %ld Bits/s\n",
           static_cast<int64_t>(bitrate.rate));
    printf("Timequantas per Bit            : %ld\n",
           static_cast<int64_t>(bitrate.tq_pre_sp + bitrate.tq_post_sp));
    printf("Timequantas before samplepoint : %ld\n",
           static_cast<int64_t>(bitrate.tq_pre_sp));
    printf("Timequantas after samplepoint  : %ld\n",
           static_cast<int64_t>(bitrate.tq_post_sp));
    printf("Syncronization Jump Width      : %ld\n",
           static_cast<int64_t>(bitrate.sjw));
    printf("Additional flags               : 0x%08lX\n",
           static_cast<int64_t>(bitrate.flags));
    int64_t sp = static_cast<int64_t>(bitrate.tq_pre_sp * 10000) /
                 static_cast<int64_t>(bitrate.tq_pre_sp + bitrate.tq_post_sp);
    printf("Position samplepoint           : %ld.%ld%%\n", sp / 100, sp % 100);
    printf("Deviation from configured rate : %ld.%02ld%%\n",
           static_cast<int64_t>(bitrate.error / 100),
           static_cast<int64_t>(bitrate.error % 100));
    printf("Controller clockrate           : %ld.%ldMHz\n",
           static_cast<int64_t>(bitrate.clock / 1000000),
           static_cast<int64_t>(bitrate.clock % 1000000));
  }
}

}  // namespace hw
}  // namespace monitor
}  // namespace apollo
