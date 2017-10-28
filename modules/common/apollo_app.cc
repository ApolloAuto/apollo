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

#include "modules/common/apollo_app.h"

#include <csignal>
#include <string>

#include "gflags/gflags.h"
#include "modules/common/log.h"
#include "modules/common/status/status.h"
#include "modules/hmi/utils/hmi_status_helper.h"

#include "ros/include/ros/ros.h"

namespace apollo {
namespace common {

void ApolloApp::SetCallbackThreadNumber(uint32_t callback_thread_num) {
  CHECK_GE(callback_thread_num, 1);
  callback_thread_num_ = callback_thread_num;
}

void ApolloApp::ReportModuleStatus(
    const apollo::hmi::ModuleStatus::Status status) {
  status_.set_name(Name());
  status_.set_status(status);
  hmi::HMIStatusHelper::ReportModuleStatus(status_);
}

int ApolloApp::Spin() {
  ros::AsyncSpinner spinner(callback_thread_num_);
  auto status = Init();
  if (!status.ok()) {
    AERROR << Name() << " Init failed: " << status;
    ReportModuleStatus(apollo::hmi::ModuleStatus::UNINITIALIZED);
    return -1;
  }
  ReportModuleStatus(apollo::hmi::ModuleStatus::INITIALIZED);
  status = Start();
  if (!status.ok()) {
    AERROR << Name() << " Start failed: " << status;
    ReportModuleStatus(apollo::hmi::ModuleStatus::STOPPED);
    return -2;
  }
  ReportModuleStatus(apollo::hmi::ModuleStatus::STARTED);
  spinner.start();
  ros::waitForShutdown();
  Stop();
  ReportModuleStatus(apollo::hmi::ModuleStatus::STOPPED);
  AINFO << Name() << " exited.";
  return 0;
}

void apollo_app_sigint_handler(int signal_num) {
  AINFO << "Received signal: " << signal_num;
  if (signal_num != SIGINT) {
    return;
  }
  bool static is_stopping = false;
  if (is_stopping) {
    return;
  }
  is_stopping = true;
  ros::shutdown();
}

}  // namespace common
}  // namespace apollo
