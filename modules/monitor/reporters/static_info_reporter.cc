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

#include "modules/monitor/reporters/static_info_reporter.h"

#include "gflags/gflags.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/data/util/info_collector.h"

DEFINE_string(static_info_reporter_name, "StaticInfoReporter",
              "Static info reporter name.");

DEFINE_double(static_info_report_interval, 40,
              "Static info reporting interval (s).");

namespace apollo {
namespace monitor {

StaticInfoReporter::StaticInfoReporter()
    : RecurrentRunner(FLAGS_static_info_reporter_name,
                      FLAGS_static_info_report_interval) {
}

void StaticInfoReporter::RunOnce(const double current_time) {
  AINFO << "Reported static info.";
  apollo::common::adapter::AdapterManager::PublishStaticInfo(
      apollo::data::InfoCollector::GetStaticInfo());
}

}  // namespace monitor
}  // namespace apollo
