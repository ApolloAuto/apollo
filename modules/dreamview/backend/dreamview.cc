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

#include "modules/dreamview/backend/dreamview.h"

#include <vector>

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/adapters/proto/adapter_config.pb.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/time/time.h"
#include "modules/common/util/file.h"
#include "modules/map/hdmap/hdmap_util.h"

#include "modules/dreamview/backend/common/dreamview_gflags.h"

namespace apollo {
namespace dreamview {

using apollo::common::adapter::AdapterManager;
using apollo::common::VehicleConfigHelper;
using apollo::common::Status;
using apollo::common::time::Clock;
using apollo::common::util::PathExists;
using apollo::hdmap::BaseMapFile;

std::string Dreamview::Name() const {
  return FLAGS_dreamview_module_name;
}

Status Dreamview::Init() {
  AdapterManager::Init(FLAGS_dreamview_adapter_config_filename);
  VehicleConfigHelper::Init();

  // Check the expected adapters are initialized.
  CHECK(AdapterManager::GetChassis()) << "ChassisAdapter is not initialized.";
  CHECK(AdapterManager::GetPlanning()) << "PlanningAdapter is not initialized.";
  CHECK(AdapterManager::GetLocalization())
      << "LocalizationAdapter is not initialized.";
  CHECK(AdapterManager::GetMonitor()) << "MonitorAdapter is not initialized.";
  CHECK(AdapterManager::GetPrediction())
      << "PredictionAdapter is not initialized.";
  CHECK(AdapterManager::GetPerceptionObstacles())
      << "PerceptionObstaclesAdapter is not initialized.";
  CHECK(AdapterManager::GetRoutingRequest())
      << "RoutingRequestAdapter is not initialized.";
  CHECK(AdapterManager::GetRoutingResponse())
      << "RoutingResponseAdapter is not initialized.";
  CHECK(AdapterManager::GetCompressedImage())
      << "CompressedImageAdapter is not initialized.";

  // Initialize and run the web server which serves the dreamview htmls and
  // javascripts and handles websocket requests.
  std::vector<std::string> options = {
      "document_root",    FLAGS_static_file_dir,  "listening_ports",
      FLAGS_server_ports, "websocket_timeout_ms", FLAGS_websocket_timeout_ms};
  if (PathExists(FLAGS_ssl_certificate)) {
    options.push_back("ssl_certificate");
    options.push_back(FLAGS_ssl_certificate);
  } else if (FLAGS_ssl_certificate.size() > 0) {
    AERROR << "Certificate file " << FLAGS_ssl_certificate
           << " does not exist!";
  }
  server_.reset(new CivetServer(options));

  image_.reset(new ImageHandler());
  websocket_.reset(new WebSocketHandler());
  map_service_.reset(new MapService());
  sim_control_.reset(new SimControl(map_service_.get()));

  sim_world_updater_.reset(
      new SimulationWorldUpdater(websocket_.get(), sim_control_.get(),
                                 map_service_.get(), FLAGS_routing_from_file));
  hmi_.reset(new HMI(websocket_.get(), map_service_.get()));

  server_->addWebSocketHandler("/websocket", *websocket_);
  server_->addHandler("/image", *image_);

  return Status::OK();
}

Status Dreamview::Start() {
  sim_world_updater_->Start();
  if (FLAGS_enable_sim_control) {
    sim_control_->Init(true);
  }
  return Status::OK();
}

void Dreamview::Stop() {
  server_->close();
  sim_control_->Stop();
}

}  // namespace dreamview
}  // namespace apollo
