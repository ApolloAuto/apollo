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

#include "modules/common/monitor_log/monitor_log_buffer.h"
#include "modules/dreamview/backend/common/handlers/websocket_handler.h"
#include "modules/dreamview_plus/backend/hmi/hmi_worker.h"
#include "modules/dreamview/backend/common/map_service/map_service.h"
#include "modules/dreamview_plus/backend/updater/updater_base.h"

/**
 * @namespace apollo::dreamview
 * @brief apollo::dreamview
 */
namespace apollo {
namespace dreamview {

class HMI : public UpdaterBase {
 public:
  using DvCallback = std::function<nlohmann::json(
      const std::string &function_name, const nlohmann::json &param_json)>;
  HMI(WebSocketHandler *websocket, MapService *map_service,
      WebSocketHandler *hmi_websocket);
  void Start(DvCallback callback_api);
  void StartStream(const double &time_interval_ms,
                   const std::string &channel_name = "",
                   nlohmann::json *subscribe_param = nullptr) override;
  void PublishMessage(const std::string &channel_name = "") override;
  void Stop();
  void StopStream(const std::string &channel_name = "") override;
  void OnTimer(const std::string &channel_name = "");
  bool UpdateDynamicModelToStatus(const std::string &dynamic_model_name);
  bool UpdateMapToStatus(const std::string &map_name = "");
  bool UpdateRecordToStatus();
  bool UpdateVehicleToStatus();
  bool UpdateCameraChannelToStatus(const std::string &channel_name);
  bool UpdatePointChannelToStatus(const std::string &channel_name);
  bool isProcessRunning(const std::string &process_name);

 private:
  // Send VehicleParam to the given conn, or broadcast if conn is null.
  void SendVehicleParam(WebSocketHandler::Connection *conn = nullptr);
  // void SendStatus(WebSocketHandler::Connection *conn = nullptr);

  void RegisterMessageHandlers();
  void RegisterDBMessageHandlers();

  apollo::common::monitor::MonitorLogBuffer monitor_log_buffer_;
  std::unique_ptr<HMIWorker> hmi_worker_;

  // No ownership.
  WebSocketHandler *websocket_;
  MapService *map_service_;
  WebSocketHandler *hmi_ws_;
  double time_interval_ms_;
  std::unique_ptr<cyber::Timer> timer_;
};

}  // namespace dreamview
}  // namespace apollo
