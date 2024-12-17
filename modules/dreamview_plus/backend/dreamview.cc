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

#include "modules/dreamview_plus/backend/dreamview.h"

#include <vector>

#include "cyber/common/file.h"
#include "cyber/time/clock.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/dreamview/backend/common/dreamview_gflags.h"
namespace {
std::map<std::string, int> plugin_function_map = {
    {"UpdateRecordToStatus", 1},
    {"UpdateDynamicModelToStatus", 2},
    {"UpdateVehicleToStatus", 3},
    {"UpdateMapToStatus", 4}};
std::map<std::string, int> hmi_function_map = {
    {"MapServiceReloadMap", 1},
    // {"RestartDynamicModel", 6},
    {"GetDataHandlerConf", 7},
    {"ClearDataHandlerConfChannelMsgs", 8},
};
std::map<std::string, int> socket_manager_function_map = {
    {"SimControlRestart", 0},
    {"MapServiceReloadMap", 1},
};
}  // namespace

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
    exit_timer_.reset(new cyber::Timer(
        FLAGS_dreamview_profiling_duration,
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
      "document_root",         FLAGS_static_file_dir,
      "listening_ports",       FLAGS_server_ports,
      "websocket_timeout_ms",  FLAGS_websocket_timeout_ms,
      "request_timeout_ms",    FLAGS_request_timeout_ms,
      "enable_keep_alive",     "yes",
      "tcp_nodelay",           "1",
      "keep_alive_timeout_ms", "500"};
  if (PathExists(FLAGS_ssl_certificate)) {
    options.push_back("ssl_certificate");
    options.push_back(FLAGS_ssl_certificate);
  } else if (FLAGS_ssl_certificate.size() > 0) {
    AERROR << "Certificate file " << FLAGS_ssl_certificate
           << " does not exist!";
  }
  server_.reset(new CivetServer(options));

  websocket_.reset(new WebSocketHandler("websocket"));
  map_ws_.reset(new WebSocketHandler("Map"));
  point_cloud_ws_.reset(new WebSocketHandler("PointCloud"));
  camera_ws_.reset(new WebSocketHandler("Camera"));
  plugin_ws_.reset(new WebSocketHandler("Plugin"));
  hmi_ws_.reset(new WebSocketHandler("HMI"));
  sim_world_ws_.reset(new WebSocketHandler("SimWorld"));
  obstacle_ws_.reset(new WebSocketHandler("Obstacle"));
  // auto registered when dreamview start to publish specified channels info
  channels_info_ws_.reset(new WebSocketHandler("ChannelsInfo"));
  map_service_.reset(new MapService());
  image_.reset(new ImageHandler());
  proto_handler_.reset(new ProtoHandler());
  perception_camera_updater_.reset(
      new PerceptionCameraUpdater(camera_ws_.get()));
  hmi_.reset(new HMI(websocket_.get(), map_service_.get(), hmi_ws_.get()));
  plugin_manager_.reset(new PluginManager(plugin_ws_.get()));
  sim_world_updater_.reset(new SimulationWorldUpdater(
      websocket_.get(), map_ws_.get(), plugin_ws_.get(), map_service_.get(),
      plugin_manager_.get(), sim_world_ws_.get(), hmi_.get(),
      FLAGS_routing_from_file));
  point_cloud_updater_.reset(new PointCloudUpdater(point_cloud_ws_.get()));
  map_updater_.reset(new MapUpdater(map_ws_.get(), map_service_.get()));
  obstacle_updater_.reset(new ObstacleUpdater(obstacle_ws_.get()));
  channels_info_updater_.reset(new ChannelsUpdater(channels_info_ws_.get()));
  updater_manager_.reset(new UpdaterManager());
  RegisterUpdaters();
  dv_plugin_manager_.reset(
      new DvPluginManager(server_.get(), updater_manager_.get()));
  server_->addWebSocketHandler("/websocket", *websocket_);
  server_->addWebSocketHandler("/map", *map_ws_);
  server_->addWebSocketHandler("/pointcloud", *point_cloud_ws_);
  server_->addWebSocketHandler("/camera", *camera_ws_);
  server_->addWebSocketHandler("/plugin", *plugin_ws_);
  server_->addWebSocketHandler("/simworld", *sim_world_ws_);
  server_->addWebSocketHandler("/hmi", *hmi_ws_);
  server_->addWebSocketHandler("/socketmanager", *socket_manager_ws_);
  server_->addWebSocketHandler("/obstacle", *obstacle_ws_);
  server_->addWebSocketHandler("/channelsinfo", *channels_info_ws_);
  server_->addHandler("/image", *image_);
  server_->addHandler("/proto", *proto_handler_);
  dv_plugin_manager_->Init();
  socket_manager_.reset(new SocketManager(
      websocket_.get(), updater_manager_.get(), dv_plugin_manager_.get()));
#if WITH_TELEOP == 1
  teleop_ws_.reset(new WebSocketHandler("Teleop"));
  teleop_.reset(new TeleopService(teleop_ws_.get()));
#endif
  return Status::OK();
}

Status Dreamview::Start() {
  hmi_->Start([this](const std::string& function_name,
                     const nlohmann::json& param_json) -> nlohmann::json {
    nlohmann::json ret = HMICallbackOtherService(function_name, param_json);
    ADEBUG << "ret: " << ret.dump();
    return ret;
  });
  plugin_manager_->Start([this](const std::string& function_name,
                                const nlohmann::json& param_json) -> bool {
    return PluginCallbackHMI(function_name, param_json);
  });
  dv_plugin_manager_->Start();
  // sim_world_updater_->StartStream(100);
  // perception_camera_updater_->Start();
  // ([this](const std::string& param_string) -> bool {
  //   return PerceptionCameraCallback(param_string);
  // });
#if WITH_TELEOP == 1
  teleop_->Start();
#endif
  return Status::OK();
}

void Dreamview::Stop() {
  dv_plugin_manager_->Stop();
  server_->close();
  SimControlManager::Instance()->Stop();
  point_cloud_updater_->Stop();
  hmi_->Stop();
  perception_camera_updater_->Stop();
  obstacle_updater_->Stop();
  plugin_manager_->Stop();
  // sim_world_updater_->StopStream();
}

nlohmann::json Dreamview::HMICallbackOtherService(
    const std::string& function_name, const nlohmann::json& param_json) {
  nlohmann::json callback_res = {};
  callback_res["result"] = false;
  if (hmi_function_map.find(function_name) == hmi_function_map.end()) {
    AERROR << "Donnot support this callback";
    return callback_res;
  }
  switch (hmi_function_map[function_name]) {
    case 1: {
      callback_res["result"] = map_service_->ReloadMap(true);
      break;
    }
      // todo(@lijin):reconstruct map and sim control set point logic
      //  case 6: {
      //       // restart dynamic model
      //       map_service_->ReloadMap(true);
      //       sim_control_manager_->Restart();
      //       callback_res["result"] = true;
      //       break;
      //     }
      // use socket manager to broadcast data handler conf after play record
    case 7: {
      socket_manager_->BrocastDataHandlerConf();
      callback_res["result"] = true;
      break;
    }
    case 8: {
      socket_manager_->BrocastDataHandlerConf(true);
      callback_res["result"] = true;
      break;
    }
    default:
      break;
  }
  return callback_res;
}

bool Dreamview::PluginCallbackHMI(const std::string& function_name,
                                  const nlohmann::json& param_json) {
  bool callback_res;
  if (plugin_function_map.find(function_name) == plugin_function_map.end()) {
    AERROR << "Donnot support this callback";
    return false;
  }
  switch (plugin_function_map[function_name]) {
    case 1: {
      callback_res = hmi_->UpdateRecordToStatus();
    } break;
    case 2: {
      if (param_json["data"].contains("dynamic_model_name")) {
        const std::string dynamic_model_name =
            param_json["data"]["dynamic_model_name"];
        if (!dynamic_model_name.empty()) {
          callback_res = hmi_->UpdateDynamicModelToStatus(dynamic_model_name);
        }
      }
    } break;
    case 3: {
      callback_res = hmi_->UpdateVehicleToStatus();
    } break;
    case 4: {
      if (param_json["data"].contains("resource_id") &&
          param_json["data"].contains("resource_id")) {
        const std::string map_name = param_json["data"]["resource_id"];
        callback_res = hmi_->UpdateMapToStatus(map_name);
      } else {
        callback_res = hmi_->UpdateMapToStatus();
      }
    } break;
    default:
      break;
  }
  return callback_res;
}

// bool Dreamview::PerceptionCameraCallback(const std::string& param_string) {
//   bool callback_res = false;
//   callback_res = hmi_->UpdateCameraChannelToStatus(param_string);
//   return callback_res;
// }

bool Dreamview::PointCloudCallback(const std::string& param_string) {
  bool callback_res = false;
  callback_res = hmi_->UpdatePointChannelToStatus(param_string);
  return callback_res;
}

void Dreamview::RegisterUpdaters() {
  updater_manager_->RegisterUpdater("simworld", sim_world_updater_.get());
  updater_manager_->RegisterUpdater("hmistatus", hmi_.get());
  updater_manager_->RegisterUpdater("camera", perception_camera_updater_.get());
  updater_manager_->RegisterUpdater("pointcloud", point_cloud_updater_.get());
  updater_manager_->RegisterUpdater("map", map_updater_.get());
  updater_manager_->RegisterUpdater("obstacle", obstacle_updater_.get());
  updater_manager_->RegisterUpdater("channelsinfo",
                                    channels_info_updater_.get());
}

}  // namespace dreamview
}  // namespace apollo
