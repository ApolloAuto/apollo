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
#include <sys/time.h>

#include "ros/include/ros/ros.h"

#include "modules/common/log.h"
#include "modules/common/proto/gnss_status.pb.h"
#include "modules/hmi/utils/hmi_status_helper.h"
#include "modules/monitor/hwmonitor/hw_check/hw_chk_utils.h"

namespace {
using apollo::hmi::HMIStatusHelper;
using apollo::hmi::HardwareStatus;
using apollo::monitor::HwCheckResult;
using apollo::monitor::hw::set_hmi_status;
using HWStatus = apollo::monitor::HardwareStatus;

volatile bool g_ins_detected = false;
volatile bool g_gnss_detected = false;
apollo::common::gnss_status::InsStatus g_ins_status;
apollo::common::gnss_status::GnssStatus g_gnss_status;

void ins_status_callback(
    const apollo::common::gnss_status::InsStatus &ins_status) {
  g_ins_status.CopyFrom(ins_status);
  g_ins_detected = true;
  std::cout << "INS status: " << ins_status.DebugString() << std::endl;
}

void gnss_status_callback(
    const apollo::common::gnss_status::GnssStatus &gnss_status) {
  g_gnss_status.CopyFrom(gnss_status);
  g_gnss_detected = true;
  std::cout << "GNSS status: " << gnss_status.DebugString() << std::endl;
}
}  // namespace

int main(int argc, char *argv[]) {
  ::google::InitGoogleLogging("platform");

  ros::init(argc, argv, std::string("gps_check"));
  ros::NodeHandle nh;
  ros::Subscriber ins_status_sub;
  ros::Subscriber gnss_status_sub;
  bool timeout = false;
  struct timeval start_time;
  struct timeval now;
  uint64_t timeout_sec = 5;

  ins_status_sub =
      nh.subscribe("/apollo/sensor/gnss/ins_status", 16, ins_status_callback);
  gnss_status_sub =
      nh.subscribe("/apollo/sensor/gnss/gnss_status", 16, gnss_status_callback);

  gettimeofday(&start_time, NULL);
  while (!(timeout) && ((!g_ins_detected) || (!g_gnss_detected))) {
    ros::spinOnce();

    gettimeofday(&now, NULL);
    if (static_cast<uint64_t>(now.tv_sec * 1000000 + now.tv_usec -
                              start_time.tv_sec * 1000000 -
                              start_time.tv_usec) >
        static_cast<uint64_t>(timeout_sec * 1000000)) {
      std::cout << "Detect timeout." << std::endl;
      timeout = true;
    }
  }

  HardwareStatus gps_st;
  if (timeout) {
    set_hmi_status(&gps_st, "GPS", HWStatus::ERR, "GPS CHECK TIMEOUT");
    std::cout << "gps check timeout." << std::endl;
  } else if (g_gnss_status.solution_completed() != true) {
    set_hmi_status(&gps_st, "GPS", HWStatus::ERR, "GPS SOLUTION UNCOMPUTED");
    std::cout << "gps solution not computed." << std::endl;
  } else {
    switch (g_ins_status.type()) {
      case apollo::common::gnss_status::InsStatus::CONVERGING:
        set_hmi_status(&gps_st, "GPS", HWStatus::NOT_READY, "INS ALIGNING");
        std::cout << "ins is aligning." << std::endl;
        break;

      case apollo::common::gnss_status::InsStatus::GOOD:
        set_hmi_status(&gps_st, "GPS", HWStatus::OK, "OK");
        std::cout << "ins is good." << std::endl;
        break;

      case apollo::common::gnss_status::InsStatus::INVALID:
      default:
        set_hmi_status(&gps_st, "GPS", HWStatus::ERR, "INS INACTIVE");
        std::cout << "ins is inactive." << std::endl;
        break;
    }
  }

  std::vector<HardwareStatus> hw_status;
  hw_status.emplace_back(gps_st);
  HMIStatusHelper::ReportHardwareStatus(hw_status);

  return 0;
}
