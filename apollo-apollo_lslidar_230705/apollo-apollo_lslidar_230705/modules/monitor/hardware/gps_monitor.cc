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

#include "modules/monitor/hardware/gps_monitor.h"

#include "cyber/common/log.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/map_util.h"
#include "modules/common_msgs/sensor_msgs/gnss_best_pose.pb.h"
#include "modules/monitor/common/monitor_manager.h"
#include "modules/monitor/software/summary_monitor.h"

DEFINE_string(gps_monitor_name, "GpsMonitor", "Name of the GPS monitor.");
DEFINE_double(gps_monitor_interval, 3, "GPS status checking interval (s).");
DEFINE_string(gps_component_name, "GPS", "GPS component name.");

namespace apollo {
namespace monitor {

using apollo::drivers::gnss::GnssBestPose;
using apollo::drivers::gnss::SolutionType;

GpsMonitor::GpsMonitor()
    : RecurrentRunner(FLAGS_gps_monitor_name, FLAGS_gps_monitor_interval) {}

void GpsMonitor::RunOnce(const double current_time) {
  auto manager = MonitorManager::Instance();
  Component* component = apollo::common::util::FindOrNull(
      *manager->GetStatus()->mutable_components(), FLAGS_gps_component_name);
  if (component == nullptr) {
    // GPS is not monitored in current mode, skip.
    return;
  }
  ComponentStatus* component_status = component->mutable_other_status();
  component_status->clear_status();

  static auto gnss_best_pose_reader =
      manager->CreateReader<GnssBestPose>(FLAGS_gnss_best_pose_topic);
  gnss_best_pose_reader->Observe();
  const auto gnss_best_pose_status = gnss_best_pose_reader->GetLatestObserved();
  if (gnss_best_pose_status == nullptr) {
    SummaryMonitor::EscalateStatus(ComponentStatus::ERROR,
                                   "No GnssBestPose message", component_status);
    return;
  }
  switch (gnss_best_pose_status->sol_type()) {
    case SolutionType::NARROW_INT:
      SummaryMonitor::EscalateStatus(ComponentStatus::OK, "", component_status);
      break;
    case SolutionType::SINGLE:
      SummaryMonitor::EscalateStatus(
          ComponentStatus::WARN, "SolutionType is SINGLE", component_status);
      break;
    default:
      SummaryMonitor::EscalateStatus(ComponentStatus::ERROR,
                                     "SolutionType is wrong", component_status);
      break;
  }
}

}  // namespace monitor
}  // namespace apollo
