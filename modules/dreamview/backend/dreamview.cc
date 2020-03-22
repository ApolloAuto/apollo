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

#include "cyber/common/file.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/common/time/time.h"
#include "modules/dreamview/backend/common/dreamview_gflags.h"

namespace apollo {
namespace dreamview {

using apollo::common::Status;
using apollo::common::VehicleConfigHelper;
using cyber::common::PathExists;

Dreamview::~Dreamview() { Stop(); }

void Dreamview::TerminateProfilingMode() {
  Stop();
  AWARN << "Profiling timer called shutdown!";
}

Status Dreamview::Init() {
  VehicleConfigHelper::Init();

  if (FLAGS_dreamview_profiling_mode &&
      FLAGS_dreamview_profiling_duration > 0.0) {
    exit_timer_.reset(
        new cyber::Timer(FLAGS_dreamview_profiling_duration,
                         [this]() { this->TerminateProfilingMode(); }, false));

    exit_timer_->Start();
    AWARN << "============================================================";
    AWARN << "| Dreamview running in profiling mode, exit in "
          << FLAGS_dreamview_profiling_duration << " seconds |";
    AWARN << "============================================================";
  }

  // Initialize and run the web server which serves the dreamview htmls and
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
  server_.reset(new CivetServer(options));

  websocket_.reset(new WebSocketHandler("SimWorld"));
  map_ws_.reset(new WebSocketHandler("Map"));
  point_cloud_ws_.reset(new WebSocketHandler("PointCloud"));
  camera_ws_.reset(new WebSocketHandler("Camera"));

  map_service_.reset(new MapService());
  image_.reset(new ImageHandler());
  sim_control_.reset(new SimControl(map_service_.get()));
  data_collection_monitor_.reset(new DataCollectionMonitor());
  perception_camera_updater_.reset(
      new PerceptionCameraUpdater(camera_ws_.get()));

  sim_world_updater_.reset(new SimulationWorldUpdater(
      websocket_.get(), map_ws_.get(), camera_ws_.get(), sim_control_.get(),
      map_service_.get(), data_collection_monitor_.get(),
      perception_camera_updater_.get(), FLAGS_routing_from_file));
  point_cloud_updater_.reset(new PointCloudUpdater(point_cloud_ws_.get()));
  hmi_.reset(new HMI(websocket_.get(), map_service_.get(),
                     data_collection_monitor_.get()));

  server_->addWebSocketHandler("/websocket", *websocket_);
  server_->addWebSocketHandler("/map", *map_ws_);
  server_->addWebSocketHandler("/pointcloud", *point_cloud_ws_);
  server_->addWebSocketHandler("/camera", *camera_ws_);
  server_->addHandler("/image", *image_);
#ifdef TELEOP
  teleop_ws_.reset(new WebSocketHandler("Teleop"));
  teleop_.reset(new TeleopService(teleop_ws_.get()));
  server_->addWebSocketHandler("/teleop", *teleop_ws_);
#endif
  return Status::OK();
}

Status Dreamview::Start() {
  sim_world_updater_->Start();
  point_cloud_updater_->Start();
  hmi_->Start();
  perception_camera_updater_->Start();
#ifdef TELEOP
  teleop_->Start();
#endif
  return Status::OK();
}

void Dreamview::Stop() {
  server_->close();
  sim_control_->Stop();
  point_cloud_updater_->Stop();
  hmi_->Stop();
  perception_camera_updater_->Stop();
}

}  // namespace dreamview
}  // namespace apollo
