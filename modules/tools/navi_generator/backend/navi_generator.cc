/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/tools/navi_generator/backend/navi_generator.h"

#include <vector>

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/adapters/proto/adapter_config.pb.h"
#include "modules/common/time/time.h"
#include "modules/common/util/file.h"
#include "modules/common/util/util.h"
#include "modules/tools/navi_generator/backend/common/navi_generator_gflags.h"

namespace apollo {
namespace navi_generator {

using apollo::common::Status;
using apollo::common::adapter::AdapterManager;
using apollo::common::time::Clock;
using apollo::common::util::PathExists;

std::string NaviGenerator::Name() const {
  return FLAGS_navi_generator_module_name;
}

void NaviGenerator::TerminateProfilingMode(const ros::TimerEvent &event) {
  Stop();
  ros::shutdown();
  AWARN << "Profiling timer called shutdown!";
}

void NaviGenerator::CheckAdapters() {
  // Check the expected adapters are initialized.
  CHECK(AdapterManager::GetChassis()) << "ChassisAdapter is not initialized.";
  CHECK(AdapterManager::GetGps()) << "GpsAdapter is not initialized.";
  CHECK(AdapterManager::GetLocalization())
      << "LocalizationAdapter is not initialized.";
}

Status NaviGenerator::Init() {
  AdapterManager::Init(FLAGS_navi_generator_adapter_config_filename);

  if (FLAGS_navi_generator_profiling_mode &&
      FLAGS_navi_generator_profiling_duration > 0.0) {
    exit_timer_ = AdapterManager::CreateTimer(
        ros::Duration(FLAGS_navi_generator_profiling_duration),
        &NaviGenerator::TerminateProfilingMode, this, true, true);
    AWARN << "============================================================";
    AWARN << "| navi_generator running in profiling mode, exit in "
          << FLAGS_navi_generator_profiling_duration << " seconds |";
    AWARN << "============================================================";
  }

  CheckAdapters();

  // Initialize and run the web server which serves the htmls and
  // javascripts and handles websocket requests.
  std::vector<std::string> options = {
      "document_root",      FLAGS_static_file_dir,   "listening_ports",
      FLAGS_server_ports,   "websocket_timeout_ms",  FLAGS_websocket_timeout_ms,
      "request_timeout_ms", FLAGS_request_timeout_ms};
  if (PathExists(FLAGS_ssl_certificate)) {
    options.push_back("ssl_certificate");
    options.push_back(FLAGS_ssl_certificate);
  } else if (FLAGS_ssl_certificate.size() > 0) {
    AERROR << "Certificate file " << FLAGS_ssl_certificate
           << " does not exist!";
  }

  server_ = std::make_unique<CivetServer>(options);
  websocket_ = std::make_unique<NaviGeneratorWebSocket>("NaviGenerator");
  topics_updater_ = std::make_unique<TopicsUpdater>(websocket_.get());

  server_->addWebSocketHandler("/naviGenerator", *websocket_);

  ApolloApp::SetCallbackThreadNumber(FLAGS_navi_generator_worker_num);

  return Status::OK();
}

Status NaviGenerator::Start() {
  topics_updater_->Start();
  return Status::OK();
}

void NaviGenerator::Stop() { server_->close(); }

}  // namespace navi_generator
}  // namespace apollo
