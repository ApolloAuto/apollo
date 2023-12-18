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

#pragma once

#include <map>
#include <memory>
#include <string>

#include "CivetServer.h"

#include "cyber/cyber.h"
#include "modules/common/status/status.h"
#include "modules/dreamview/backend/common/handlers/image_handler.h"
#include "modules/dreamview/backend/common/handlers/proto_handler.h"
#include "modules/dreamview/backend/common/handlers/websocket_handler.h"
#include "modules/dreamview_plus/backend/dv_plugin/dv_plugin_manager.h"
#include "modules/dreamview_plus/backend/hmi/hmi.h"
#include "modules/dreamview/backend/common/map_service/map_service.h"
#include "modules/dreamview_plus/backend/map/map_updater.h"
#include "modules/dreamview_plus/backend/perception_camera_updater/perception_camera_updater.h"
#include "modules/dreamview_plus/backend/obstacle_updater/obstacle_updater.h"
#include "modules/dreamview/backend/common/plugins/plugin_manager.h"
#include "modules/dreamview_plus/backend/point_cloud/point_cloud_updater.h"
#include "modules/dreamview/backend/common/sim_control_manager/sim_control_manager.h"
#include "modules/dreamview_plus/backend/simulation_world/simulation_world_updater.h"
#include "modules/dreamview_plus/backend/socket_manager/socket_manager.h"
#include "modules/dreamview_plus/backend/updater/updater_manager.h"
#include "modules/dreamview_plus/backend/channels_updater/channels_updater.h"
#if WITH_TELEOP == 1
#include "modules/dreamview/backend/common/teleop/teleop.h"
#endif

/**
 * @namespace apollo::dreamview
 * @brief apollo::dreamview
 */
namespace apollo {
namespace dreamview {

class Dreamview {
 public:
  ~Dreamview();

  apollo::common::Status Init();
  apollo::common::Status Start();
  void Stop();
  void RegisterUpdaters();

 private:
  void TerminateProfilingMode();
  bool PluginCallbackHMI(const std::string& function_name,
                         const nlohmann::json& param_json);
  nlohmann::json HMICallbackOtherService(const std::string& function_name,
                                         const nlohmann::json& param_json);
  bool PointCloudCallback(const std::string& param_string);

  std::unique_ptr<cyber::Timer> exit_timer_;

  std::unique_ptr<PointCloudUpdater> point_cloud_updater_;
  std::unique_ptr<CivetServer> server_;
  std::unique_ptr<WebSocketHandler> websocket_ = nullptr;
  std::unique_ptr<WebSocketHandler> map_ws_ = nullptr;
  std::unique_ptr<WebSocketHandler> point_cloud_ws_ = nullptr;
  std::unique_ptr<WebSocketHandler> camera_ws_ = nullptr;
  std::unique_ptr<WebSocketHandler> plugin_ws_ = nullptr;
  std::unique_ptr<WebSocketHandler> sim_world_ws_ = nullptr;
  std::unique_ptr<WebSocketHandler> obstacle_ws_ = nullptr;
  std::unique_ptr<WebSocketHandler> hmi_ws_;
  std::unique_ptr<WebSocketHandler> socket_manager_ws_;
  std::unique_ptr<WebSocketHandler> channels_info_ws_ = nullptr;
  std::unique_ptr<DvPluginManager> dv_plugin_manager_ = nullptr;
  std::unique_ptr<ImageHandler> image_;
  std::unique_ptr<ProtoHandler> proto_handler_;
  std::unique_ptr<MapService> map_service_;
  std::unique_ptr<HMI> hmi_;
  std::unique_ptr<PerceptionCameraUpdater> perception_camera_updater_;
  std::unique_ptr<MapUpdater> map_updater_;
  std::unique_ptr<PluginManager> plugin_manager_;
  std::unique_ptr<SocketManager> socket_manager_;
  std::unique_ptr<SimulationWorldUpdater> sim_world_updater_ = nullptr;
  std::unique_ptr<UpdaterManager> updater_manager_ = nullptr;
  std::unique_ptr<ObstacleUpdater> obstacle_updater_ = nullptr;
  std::unique_ptr<ChannelsUpdater> channels_info_updater_ = nullptr;
#if WITH_TELEOP == 1
  std::unique_ptr<TeleopService> teleop_;
  std::unique_ptr<WebSocketHandler> teleop_ws_;
#endif
};

}  // namespace dreamview
}  // namespace apollo
