/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

/**
 * @file
 */

#pragma once
#include<string>
#include<memory>
#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/dreamview_plus/proto/data_handler.pb.h"
#include "modules/common_msgs/dreamview_msgs/simulation_world.pb.h"

#include "cyber/common/log.h"
#include "cyber/cyber.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/dreamview/backend/common/dreamview_gflags.h"
#include "modules/dreamview/backend/common/handlers/websocket_handler.h"
#include "modules/dreamview/backend/common/map_service/map_service.h"
#include "modules/dreamview_plus/backend/map/map_updater.h"
#include "modules/dreamview_plus/backend/updater/updater_base.h"
/**
 * @namespace apollo::dreamview
 * @brief apollo::dreamview
 */
namespace apollo {
namespace dreamview {

using apollo::localization::LocalizationEstimate;

/**
 * @class MapUpdater
 * @brief A wrapper around WebSocketHandler to keep pushing map to
 * frontend via websocket.
 */
class MapUpdater : public UpdaterBase {
 public:
  /**
   * @brief Constructor with the websocket handler.
   * @param websocket Pointer of the websocket handler that has been attached to
   * the server.
   * @param map_service A service object that handles map data.
   */
  MapUpdater(WebSocketHandler *websocket, const MapService *map_service);
  ~MapUpdater() override {}
  void StartStream(const double &time_interval_ms,
                   const std::string &channel_name = "",
                   nlohmann::json *subscribe_param = nullptr) override;
  void StopStream(const std::string& channel_name = "") override;
  void OnTimer(const std::string& channel_name = "");
  void PublishMessage(const std::string& channel_name = "") override;

 private:
  std::unique_ptr<cyber::Node> node_;
  WebSocketHandler *websocket_ = nullptr;
  const MapService *map_service_;

  // retrieved map string by map element ids.
  std::string retrieved_map_string_;

  /**
   * @brief Get adc localization position.
   * @param localization Data obtained from the positioning channel.
   */
  void UpdateAdcPosition(
      const std::shared_ptr<apollo::localization::LocalizationEstimate>
          &localization);
  std::shared_ptr<cyber::Reader<apollo::localization::LocalizationEstimate>>
      localization_reader_;

  /**
   * @brief Get map elment ids by sim map radius.
   * @param radius Retrieves the radius bounds of a map element.
   * @param ids Map element ids.
   */
  void GetMapElementIds(double radius, MapElementIds *ids);

  // adc localization position
  apollo::common::PointENU adc_point_;

  std::unique_ptr<cyber::Timer> timer_;
  MapElementIds map_element_ids_;
};

}  // namespace dreamview
}  // namespace apollo
