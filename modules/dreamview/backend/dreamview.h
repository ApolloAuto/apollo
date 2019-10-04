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

#include <memory>
#include <string>

#include "CivetServer.h"
#include "cyber/cyber.h"
#include "modules/common/status/status.h"
#include "modules/dreamview/backend/data_collection_monitor/data_collection_monitor.h"
#include "modules/dreamview/backend/handlers/image_handler.h"
#include "modules/dreamview/backend/handlers/websocket_handler.h"
#include "modules/dreamview/backend/hmi/hmi.h"
#include "modules/dreamview/backend/map/map_service.h"
#include "modules/dreamview/backend/perception_camera_updater/perception_camera_updater.h"
#include "modules/dreamview/backend/point_cloud/point_cloud_updater.h"
#include "modules/dreamview/backend/sim_control/sim_control.h"
#include "modules/dreamview/backend/simulation_world/simulation_world_updater.h"
#ifdef TELEOP
#include "modules/dreamview/backend/teleop/teleop.h"
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

 private:
  void TerminateProfilingMode();

  std::unique_ptr<cyber::Timer> exit_timer_;

  std::unique_ptr<SimulationWorldUpdater> sim_world_updater_;
  std::unique_ptr<PointCloudUpdater> point_cloud_updater_;
  std::unique_ptr<SimControl> sim_control_;
  std::unique_ptr<CivetServer> server_;
  std::unique_ptr<WebSocketHandler> websocket_;
  std::unique_ptr<WebSocketHandler> map_ws_;
  std::unique_ptr<WebSocketHandler> point_cloud_ws_;
  std::unique_ptr<WebSocketHandler> camera_ws_;
  std::unique_ptr<ImageHandler> image_;
  std::unique_ptr<MapService> map_service_;
  std::unique_ptr<HMI> hmi_;
  std::unique_ptr<DataCollectionMonitor> data_collection_monitor_;
  std::unique_ptr<PerceptionCameraUpdater> perception_camera_updater_;
#ifdef TELEOP
  std::unique_ptr<TeleopService> teleop_;
  std::unique_ptr<WebSocketHandler> teleop_ws_;
#endif
};

}  // namespace dreamview
}  // namespace apollo
