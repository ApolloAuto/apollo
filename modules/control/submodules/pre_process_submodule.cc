/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

/**
 * @file pre_process_submodule.cc
 */

#include "modules/control/submodules/pre_process_submodule.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/time/time.h"
#include "modules/control/common/control_gflags.h"

namespace apollo {
namespace control {
using apollo::canbus::Chassis;
using apollo::common::Status;
using apollo::common::time::Clock;
using apollo::localization::LocalizationEstimate;
using apollo::planning::ADCTrajectory;

PreProcessSubmodule::PreProcessSubmodule()
    : monitor_logger_buffer_(common::monitor::MonitorMessageItem::CONTROL) {}

PreProcessSubmodule::~PreProcessSubmodule() {}

std::string PreProcessSubmodule::Name() const {
  return FLAGS_pre_process_submodule_name;
}

bool PreProcessSubmodule::Init() {
  if (!cyber::common::GetProtoFromFile(FLAGS_control_common_conf_file,
                                       &control_common_conf_)) {
    AERROR << "Unable to load control common conf file: " +
                  FLAGS_control_common_conf_file;
    return false;
  }
  return true;
}

void PreProcessSubmodule::OnChassis(const std::shared_ptr<Chassis> &chassis) {
  ADEBUG << "Received chassis data: run chassis callback.";
  std::lock_guard<std::mutex> lock(mutex_);
  latest_chassis_.CopyFrom(*chassis);
}

}  // namespace control
}  // namespace apollo
