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
#include "cyber/time/clock.h"
#include "modules/common/configs/vehicle_config_helper.h"
#include "modules/dreamview/backend/common/dreamview_gflags.h"
#include "modules/dreamview/backend/common/sim_control_manager/sim_control_manager.h"
namespace {
std::map<std::string, int> plugin_function_map = {
    {"UpdateScenarioSetToStatus", 0},
    {"UpdateRecordToStatus", 1},
    {"UpdateDynamicModelToStatus", 2},
    {"UpdateVehicleToStatus", 3}};
std::map<std::string, int> hmi_function_map = {
    {"SimControlRestart", 0},  {"MapServiceReloadMap", 1},
    {"LoadDynamicModels", 2},  {"ChangeDynamicModel", 3},
    {"DeleteDynamicModel", 4}, {"AddDynamicModel", 5},
    {"RestartDynamicModel", 6},
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

  websocket_.reset(new WebSocketHandler("SimWorld"));
  map_ws_.reset(new WebSocketHandler("Map"));
  point_cloud_ws_.reset(new WebSocketHandler("PointCloud"));
  camera_ws_.reset(new WebSocketHandler("Camera"));
  plugin_ws_.reset(new WebSocketHandler("Plugin"));

  map_service_.reset(new MapService());
  image_.reset(new ImageHandler());
  perception_camera_updater_.reset(
      new PerceptionCameraUpdater(camera_ws_.get()));

  hmi_.reset(new HMI(websocket_.get(), map_service_.get()));
  plugin_manager_.reset(new PluginManager(plugin_ws_.get()));
  sim_world_updater_.reset(new SimulationWorldUpdater(
      websocket_.get(), map_ws_.get(), camera_ws_.get(),
      plugin_ws_.get(), map_service_.get(),
      perception_camera_updater_.get(), plugin_manager_.get(),
      FLAGS_routing_from_file));
  point_cloud_updater_.reset(
      new PointCloudUpdater(point_cloud_ws_.get(), sim_world_updater_.get()));

  server_->addWebSocketHandler("/websocket", *websocket_);
  server_->addWebSocketHandler("/map", *map_ws_);
  server_->addWebSocketHandler("/pointcloud", *point_cloud_ws_);
  server_->addWebSocketHandler("/camera", *camera_ws_);
  server_->addWebSocketHandler("/plugin", *plugin_ws_);
  server_->addHandler("/image", *image_);
#if WITH_TELEOP == 1
  teleop_ws_.reset(new WebSocketHandler("Teleop"));
  teleop_.reset(new TeleopService(teleop_ws_.get()));
  server_->addWebSocketHandler("/teleop", *teleop_ws_);
#endif
  return Status::OK();
}

Status Dreamview::Start() {
  sim_world_updater_->Start();
  point_cloud_updater_->Start([this](const std::string& param_string) -> bool {
    return PointCloudCallback(param_string);
  });
  hmi_->Start([this](const std::string& function_name,
                     const nlohmann::json& param_json) -> nlohmann::json {
    nlohmann::json ret = HMICallbackSimControl(function_name, param_json);
    ADEBUG << "ret: " << ret.dump();
    return ret;
  });
  perception_camera_updater_->Start(
      [this](const std::string& param_string) -> bool {
        return PerceptionCameraCallback(param_string);
      });
  plugin_manager_->Start([this](const std::string& function_name,
                                const nlohmann::json& param_json) -> bool {
    return PluginCallbackHMI(function_name, param_json);
  });
#if WITH_TELEOP == 1
  teleop_->Start();
#endif
  return Status::OK();
}

void Dreamview::Stop() {
  server_->close();
  SimControlManager::Instance()->Stop();
  point_cloud_updater_->Stop();
  hmi_->Stop();
  perception_camera_updater_->Stop();
  plugin_manager_->Stop();
}

nlohmann::json Dreamview::HMICallbackSimControl(
    const std::string& function_name, const nlohmann::json& param_json) {
  nlohmann::json callback_res = {};
  callback_res["result"] = false;
  if (hmi_function_map.find(function_name) == hmi_function_map.end()) {
    AERROR << "Donnot support this callback";
    return callback_res;
  }
  switch (hmi_function_map[function_name]) {
    case 0: {
      // 解析结果
      if (param_json.contains("x") && param_json.contains("y")) {
        const double x = param_json["x"];
        const double y = param_json["y"];
        SimControlManager::Instance()->Restart(x, y);
        callback_res["result"] = true;
      }
    } break;
    case 1: {
      map_service_->ReloadMap(true);
      callback_res["result"] = true;
      break;
    }
    case 2: {
      // loadDynamicModels
      if (SimControlManager::Instance()->IsEnabled()) {
        nlohmann::json load_res =
            SimControlManager::Instance()->LoadDynamicModels();
        callback_res["loaded_dynamic_models"] =
            load_res["loaded_dynamic_models"];
        callback_res["result"] = true;
      } else {
        AERROR << "Sim control is not enabled!";
      }
      break;
    }
    case 3: {
      // 解析结果
      if (param_json.contains("dynamic_model_name") &&
          SimControlManager::Instance()->IsEnabled()) {
        callback_res["result"] =
            SimControlManager::Instance()->ChangeDynamicModel(
                param_json["dynamic_model_name"]);
      } else {
        AERROR << "Sim control is not enabled or missing dynamic model name "
                  "param!";
      }
      break;
    }
    case 4: {
      // 解析结果
      if (param_json.contains("dynamic_model_name") &&
          SimControlManager::Instance()->IsEnabled()) {
        callback_res["result"] =
            SimControlManager::Instance()->DeleteDynamicModel(
                param_json["dynamic_model_name"]);
      } else {
        AERROR << "Sim control is not enabled or missing dynamic model name "
                  "param!";
      }
      break;
    }
    case 5: {
      // addDynamicModel
      if (param_json.contains("dynamic_model_name") &&
          SimControlManager::Instance()->IsEnabled()) {
        callback_res["result"] = SimControlManager::Instance()->AddDynamicModel(
            param_json["dynamic_model_name"]);
      } else {
        AERROR << "Sim control is not enabled or missing dynamic model name "
                  "param!";
      }
      break;
    }
    case 6: {
      // restart dynamic model
      map_service_->ReloadMap(true);
      SimControlManager::Instance()->Restart();
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
    case 0: {
      // 解析结果
      if (param_json.contains("data") &&
          param_json["data"].contains("scenario_set_id") &&
          param_json["data"].contains("scenario_set_name")) {
        const std::string scenario_set_id =
            param_json["data"]["scenario_set_id"];
        const std::string scenario_set_name =
            param_json["data"]["scenario_set_name"];
        if (!scenario_set_id.empty() && !scenario_set_name.empty()) {
          callback_res = hmi_->UpdateScenarioSetToStatus(scenario_set_id,
                                                         scenario_set_name);
        }
      }
    } break;
    case 1: {
      callback_res = hmi_->UpdateRecordToStatus();
    } break;
    case 2: {
      if (param_json.contains("data") &&
          param_json["data"].contains("dynamic_model_name")) {
        const std::string dynamic_model_name =
            param_json["data"]["dynamic_model_name"];
        if (!dynamic_model_name.empty()) {
          callback_res = hmi_->UpdateDynamicModelToStatus(dynamic_model_name);
        }
      }
    } break;
    case 3: {
      callback_res = hmi_->UpdateVehicleToStatus();
    }
    default:
      break;
  }
  return callback_res;
}

bool Dreamview::PerceptionCameraCallback(const std::string& param_string) {
  bool callback_res = false;
  callback_res = hmi_->UpdateCameraChannelToStatus(param_string);
  return callback_res;
}

bool Dreamview::PointCloudCallback(const std::string& param_string) {
  bool callback_res = false;
  callback_res = hmi_->UpdatePointChannelToStatus(param_string);
  return callback_res;
}
}  // namespace dreamview
}  // namespace apollo
