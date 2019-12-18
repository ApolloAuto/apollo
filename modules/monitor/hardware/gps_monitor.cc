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
#include "modules/drivers/gnss/proto/gnss_status.pb.h"
#include "modules/drivers/gnss/proto/ins.pb.h"
#include "modules/monitor/common/monitor_manager.h"
#include "modules/monitor/software/summary_monitor.h"

DEFINE_string(gps_monitor_name, "GpsMonitor", "Name of the GPS monitor.");
DEFINE_double(gps_monitor_interval, 3, "GPS status checking interval (s).");
DEFINE_string(gps_component_name, "GPS", "GPS component name.");

namespace apollo {
namespace monitor {

using apollo::drivers::gnss::GnssStatus;
using apollo::drivers::gnss::InsStatus;

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

  // Check Gnss status.
  static auto gnss_status_reader =
      manager->CreateReader<GnssStatus>(FLAGS_gnss_status_topic);
  gnss_status_reader->Observe();
  const auto gnss_status = gnss_status_reader->GetLatestObserved();
  if (gnss_status == nullptr) {
    SummaryMonitor::EscalateStatus(ComponentStatus::ERROR,
                                   "No GNSS status message", component_status);
    return;
  }
  if (!gnss_status->solution_completed()) {
    SummaryMonitor::EscalateStatus(
        ComponentStatus::WARN, "GNSS solution uncompleted", component_status);
    return;
  }

  // Check Ins status.
  static auto ins_status_reader =
      manager->CreateReader<InsStatus>(FLAGS_ins_status_topic);
  ins_status_reader->Observe();
  const auto ins_status = ins_status_reader->GetLatestObserved();
  if (ins_status == nullptr) {
    SummaryMonitor::EscalateStatus(ComponentStatus::ERROR,
                                   "No INS status message", component_status);
    return;
  }
  switch (ins_status->type()) {
    case InsStatus::CONVERGING:
      SummaryMonitor::EscalateStatus(
          ComponentStatus::WARN, "INS not ready, converging", component_status);
      break;
    case InsStatus::GOOD:
      SummaryMonitor::EscalateStatus(ComponentStatus::OK, "", component_status);
      break;
    case InsStatus::INVALID:
      SummaryMonitor::EscalateStatus(ComponentStatus::ERROR,
                                     "INS status invalid", component_status);
      break;
    default:
      SummaryMonitor::EscalateStatus(ComponentStatus::ERROR,
                                     "INS status unknown", component_status);
      break;
  }
}

}  // namespace monitor
}  // namespace apollo
