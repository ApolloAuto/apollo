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

#include "modules/monitor/hardware/gps/gps_monitor.h"

#include <algorithm>

#include "cybertron/common/log.h"
#include "modules/drivers/gnss/proto/gnss_best_pose.pb.h"
#include "modules/drivers/gnss/proto/gnss_status.pb.h"
#include "modules/drivers/gnss/proto/ins.pb.h"
#include "modules/monitor/common/message_observer.h"
#include "modules/monitor/common/monitor_manager.h"

DEFINE_string(gps_hardware_name, "GPS", "Name of the GPS hardware.");
DEFINE_string(gps_monitor_name, "GpsMonitor", "Name of the GPS monitor.");
DEFINE_double(gps_monitor_interval, 3, "GPS status checking interval (s).");

DEFINE_double(acceptable_gnss_best_pose_std_dev, 0.5,
              "Acceptable Gnss BestPose standard deviation on latitude, "
              "longitude and height.");

DEFINE_double(acceptable_gnss_best_pose_unstable_duration, 120,
              "Acceptable Gnss BestPose unstable duration in seconds.");

namespace apollo {
namespace monitor {

using apollo::drivers::gnss::GnssBestPose;
using apollo::drivers::gnss_status::GnssStatus;
using apollo::drivers::gnss_status::InsStatus;

GpsMonitor::GpsMonitor() : RecurrentRunner(FLAGS_gps_monitor_name,
                                           FLAGS_gps_monitor_interval) {
}

void GpsMonitor::RunOnce(const double current_time) {
  static auto *status = MonitorManager::GetHardwareStatus(
      FLAGS_gps_hardware_name);

  // Check Gnss status.
  static auto gnss_status_observer =
      MonitorManager::CreateObserver<GnssStatus>(FLAGS_gnss_status_topic);
  const auto gnss_status = gnss_status_observer->GetLatest();
  if (gnss_status == nullptr) {
    status->set_status(HardwareStatus::ERR);
    status->set_detailed_msg("No GNSS status message.");
    return;
  }
  if (!gnss_status->solution_completed()) {
    status->set_status(HardwareStatus::WARN);
    status->set_detailed_msg("GNSS solution uncompleted.");
    return;
  }

  // Check Ins status.
  static auto ins_status_observer =
      MonitorManager::CreateObserver<InsStatus>(FLAGS_ins_status_topic);
  const auto ins_status = ins_status_observer->GetLatest();
  if (ins_status == nullptr) {
    status->set_status(HardwareStatus::ERR);
    status->set_detailed_msg("No INS status message.");
    return;
  }
  switch (ins_status->type()) {
    case InsStatus::CONVERGING:
      status->set_status(HardwareStatus::NOT_READY);
      status->set_detailed_msg("INS ALIGNING");
      return;
    case InsStatus::GOOD:
      break;
    case InsStatus::INVALID:
    default:
      status->set_status(HardwareStatus::ERR);
      status->set_detailed_msg("INS status invalid.");
      return;
  }

  // Check Gnss BestPose.
  static auto best_pose_observer =
      MonitorManager::CreateObserver<GnssBestPose>(FLAGS_gnss_best_pose_topic);
  const auto best_pose = best_pose_observer->GetLatest();
  if (best_pose == nullptr) {
    status->set_status(HardwareStatus::ERR);
    status->set_detailed_msg("No Gnss BestPose message.");
    return;
  }
  const double largest_std_dev = std::max({best_pose->latitude_std_dev(),
                                           best_pose->longitude_std_dev(),
                                           best_pose->height_std_dev()});
  if (largest_std_dev > FLAGS_acceptable_gnss_best_pose_std_dev) {
    status->set_status(HardwareStatus::GPS_UNSTABLE_WARNING);
    status->set_detailed_msg("GPS BestPose is unstable.");
    if (status->has_gps_unstable_start_time()) {
      const double duration = current_time - status->gps_unstable_start_time();
      if (duration > FLAGS_acceptable_gnss_best_pose_unstable_duration) {
        status->set_status(HardwareStatus::GPS_UNSTABLE_ERROR);
        MonitorManager::LogBuffer().ERROR("GPS is very unstable!");
      }
    } else {
      status->set_gps_unstable_start_time(current_time);
      MonitorManager::LogBuffer().WARN("GPS becomes unstable!");
    }
    return;
  } else if (status->has_gps_unstable_start_time()) {
    status->clear_gps_unstable_start_time();
    MonitorManager::LogBuffer().INFO("GPS stability recovered.");
  }

  // All check passed.
  status->set_status(HardwareStatus::OK);
  status->set_detailed_msg("OK");
}

}  // namespace monitor
}  // namespace apollo
