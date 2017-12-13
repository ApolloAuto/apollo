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

#include "modules/monitor/online_report/vehicle_state_reporter.h"

#include "gflags/gflags.h"
#include "modules/common/log.h"
#include "modules/common/util/http_client.h"
#include "modules/common/util/json_util.h"
#include "modules/common/vehicle_state/vehicle_state_provider.h"
#include "modules/data/util/info_collector.h"
#include "modules/monitor/proto/online_report.pb.h"
#include "modules/monitor/common/monitor_manager.h"

DEFINE_string(vehicle_state_reporter_name, "VehicleStateReporter",
              "Vehicle state reporter name.");

DEFINE_double(vehicle_state_report_interval, 5,
              "Vehicle state reporting interval (s).");

namespace apollo {
namespace monitor {

VehicleStateReporter::VehicleStateReporter()
    : RecurrentRunner(FLAGS_vehicle_state_reporter_name,
                      FLAGS_vehicle_state_report_interval) {
}

void VehicleStateReporter::RunOnce(const double current_time) {
  VehicleStateReport report;
  *report.mutable_vehicle_info() =
      apollo::data::InfoCollector::GetVehicleInfo();
  *report.mutable_vehicle_state() =
      apollo::common::VehicleStateProvider::instance()->vehicle_state();

  const nlohmann::json json = apollo::common::util::JsonUtil::ProtoToTypedJson(
      "VehicleStateReport", report);

  const auto &endpoint = MonitorManager::GetConfig().online_report_endpoint();
  CHECK(!endpoint.empty()) << "No remote endpoint to report to.";
  const auto status = apollo::common::util::HttpClient::Post(endpoint, json);
  if (status.ok()) {
    AINFO_EVERY(100) << "Reported vehicle state to " << endpoint;
  } else {
    AERROR_EVERY(100) << "Cannot report vehicle state to " << endpoint;
  }
}

}  // namespace monitor
}  // namespace apollo
