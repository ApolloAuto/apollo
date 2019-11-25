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

#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/dreamview/backend/data_collection_monitor/data_collection_monitor.h"
#include "modules/dreamview/backend/handlers/websocket_handler.h"
#include "modules/dreamview/backend/hmi/hmi_worker.h"
#include "modules/dreamview/backend/map/map_service.h"

/**
 * @namespace apollo::dreamview
 * @brief apollo::dreamview
 */
namespace apollo {
namespace dreamview {

class HMI {
 public:
  HMI(WebSocketHandler *websocket, MapService *map_service,
      DataCollectionMonitor *data_collection_monitor_);
  void Start();
  void Stop();

 private:
  // Send VehicleParam to the given conn, or broadcast if conn is null.
  void SendVehicleParam(WebSocketHandler::Connection *conn = nullptr);
  void SendStatus(WebSocketHandler::Connection *conn = nullptr);

  void RegisterMessageHandlers();

  std::unique_ptr<HMIWorker> hmi_worker_;
  apollo::common::monitor::MonitorLogBuffer monitor_log_buffer_;

  // No ownership.
  WebSocketHandler *websocket_;
  MapService *map_service_;
  DataCollectionMonitor *data_collection_monitor_;
};

}  // namespace dreamview
}  // namespace apollo
