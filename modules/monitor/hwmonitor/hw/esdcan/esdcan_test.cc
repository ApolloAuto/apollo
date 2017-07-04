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

#include "esdcan_test.h"

#include "modules/monitor/hwmonitor/hw/hw_log_module.h"
#include "esdcan_err_str.h"

namespace apollo {
namespace platform {
namespace hw {

NTCAN_RESULT esdcan_do_test(int id, EsdCanDetails *details) {
  NTCAN_HANDLE h0;
  NTCAN_RESULT ret;

  details->invalidate();

  ret = canOpen(id, 0, 1, 1, 0, 0, &h0);
  if (ret == NTCAN_SUCCESS) {
    PLATFORM_DBG(get_log_module(), log::LVL_INFO,
                 "Successfully opened ESD-CAN device %d", id);

    ret = canStatus(h0, &details->if_status);
    if (ret != NTCAN_SUCCESS) {
      PLATFORM_LOG(get_log_module(), log::LVL_ERR,
                   "Cannot get status of ESD-CAN device %d, ret=%d (%s)\n", id,
                   ret, esdcan_err_to_str(ret));
      goto err;
    }

    PLATFORM_DBG(get_log_module(), log::LVL_INFO,
                 "Got ESD-CAN-%d interface status", id);
    details->add_valid_field(EsdCanDetails::IF_STATUS);
    // else: fall-out to continue
  } else {
    PLATFORM_LOG(get_log_module(), log::LVL_ERR,
                 "Failed to open ESD-CAN device %d, error: %d (%s)\n", id, ret,
                 esdcan_err_to_str(ret));
    goto err;
  }

  ret = canIoctl(h0, NTCAN_IOCTL_GET_BUS_STATISTIC, &details->stats);
  if (ret != NTCAN_SUCCESS) {
    PLATFORM_LOG(get_log_module(), log::LVL_ERR,
                 "NTCAN_IOCTL_GET_BUS_STATISTIC failed for device %d with "
                 "error: %d (%s)\n",
                 id, ret, esdcan_err_to_str(ret));
    goto err;
  }
  details->add_valid_field(EsdCanDetails::STATS);
  PLATFORM_DBG(get_log_module(), log::LVL_INFO, "Got ESD-CAN-%d statistics",
               id);

  ret = canIoctl(h0, NTCAN_IOCTL_GET_CTRL_STATUS, &details->ctrl_state);
  if (ret != NTCAN_SUCCESS) {
    PLATFORM_LOG(get_log_module(), log::LVL_ERR,
                 "NTCAN_IOCTL_GET_CTRL_STATUS failed for device %d with error: "
                 "%d (%s)\n",
                 id, ret, esdcan_err_to_str(ret));
    goto err;
  }
  details->add_valid_field(EsdCanDetails::CTRL_STATE);
  PLATFORM_DBG(get_log_module(), log::LVL_INFO, "Got ESD-CAN-%d strl-state",
               id);

  ret = canIoctl(h0, NTCAN_IOCTL_GET_BITRATE_DETAILS, &details->bitrate);
  if (ret != NTCAN_SUCCESS) {
    PLATFORM_LOG(
        get_log_module(), log::LVL_ERR,
        "NTCAN_IOCTL_GET_BITRATE_DETAILS for device %d with error: %d (%s)\n",
        id, ret, esdcan_err_to_str(ret));
    goto err;
  }
  details->add_valid_field(EsdCanDetails::BITRATE);
  PLATFORM_DBG(get_log_module(), log::LVL_INFO, "Got ESD-CAN-%d bitrate", id);

err:
  canClose(h0);
  details->result = ret;
  return ret;
}

}  // namespace hw
}  // namespace platform
}  // namespace apollo
