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

/**
 * @file
 */

#ifndef MODULES_TOOLS_NAVI_GENERATOR_BACKEND_HMI_TOPICS_SERVICE_H_
#define MODULES_TOOLS_NAVI_GENERATOR_BACKEND_HMI_TOPICS_SERVICE_H_

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "modules/common/log.h"
#include "modules/tools/navi_generator/backend/util/navigation_editor.h"
#include "modules/tools/navi_generator/backend/util/navigation_provider.h"
#include "modules/tools/navi_generator/backend/util/trajectory_collector.h"
#include "modules/tools/navi_generator/backend/util/trajectory_processor.h"
#include "modules/tools/navi_generator/backend/webserver/navi_generator_websocket.h"
#include "modules/tools/navi_generator/proto/simulation_world.pb.h"
#include "third_party/json/json.hpp"

namespace apollo {
namespace navi_generator {

/**
 * @class TopicsService
 * @brief
 * NOTE: This class is not thread-safe.
 */
class TopicsService {
 public:
  /**
   * @brief Constructor of TopicsService.
   * @param websocket Pointer of the websocket handler that has been attached to
   * the server.
   */
  explicit TopicsService(NaviGeneratorWebSocket *websocket);

  /**
   * @brief The function Update() is periodically called to check for updates
   * from the adapters.
   */
  void Update();

  nlohmann::json GetUpdateAsJson() const;

  nlohmann::json GetCommandResponseAsJson(const std::string &type,
                                          const std::string &module,
                                          const std::string &command,
                                          const int success) const;

  inline const SimulationWorld &world() const { return world_; }
  /**
   * @brief Sets the flag to send system status and localization to frontend.
   */
  void SetReadyToSend(bool ready_to_send) { ready_to_send_ = ready_to_send; }
  bool ReadyToSend() const { return ready_to_send_; }
  /**
   * @brief Check whether the SimulationWorld object has enough information.
   * The backend won't push the SimulationWorld to frontend if it is not ready.
   * @return True if the object is ready to push.
   */
  bool ReadyToPush() const { return ready_to_push_.load(); }
  void SetReadyToPush(bool read_to_push) { ready_to_push_.store(read_to_push); }
  /**
   * @brief Update simulation world with incoming data, e.g.localization.
   */
  void UpdateObserved(
      const apollo::localization::LocalizationEstimate &localization);

  // A callback function which updates the GUI.
  static void UpdateGUI(const std::string &msg, void *service);
  // A callback function which informs the bag files has been processed.
  static void NotifyBagFilesProcessed(void *service);

  bool SetCommonBagFileInfo(
      const apollo::navi_generator::util::CommonBagFileInfo &common_file_info);
  bool ProcessBagFileSegment(
      const apollo::navi_generator::util::FileSegment &file_segment);
  bool SaveFilesToDatabase();

  bool InitCollector();
  bool StartCollector(const std::string &collection_type,
                      const std::size_t min_speed_limit,
                      const std::size_t max_speed_limit);
  bool StopCollector();
  bool UpdateCollector(const std::string &collection_type,
                       const std::size_t min_speed_limit,
                       const std::size_t max_speed_limit);
  bool GetCollectorOptions(const std::string &collection_type,
                           const std::size_t min_speed_limit,
                           const std::size_t max_speed_limit,
                           util::CollectorOptions *const options);

  nlohmann::json GetRoutePathAsJson(const nlohmann::json &map_data);

  nlohmann::json GetNavigationPathAsJson(const nlohmann::json &map_data);

  void EnableRoadDeviationCorrection(bool enable_road_deviation_correction) {
    road_deviation_correction_enabled_ = enable_road_deviation_correction;
  }

  bool CorrectRoadDeviation();
  bool SaveRoadCorrection();

  bool ModifySpeedLimit(const apollo::localization::msf::WGS84Corr &start_point,
                        const apollo::localization::msf::WGS84Corr &end_point,
                        const std::uint8_t new_speed_min,
                        const std::uint8_t new_speed_max);
  bool SaveSpeedLimit();

 private:
  // The pointer of NaviGeneratorWebSocket, not owned by TopicsService.
  NaviGeneratorWebSocket *websocket_ = nullptr;

  std::unique_ptr<util::TrajectoryProcessor> trajectory_processor_;
  std::unique_ptr<util::TrajectoryCollector> trajectory_collector_;
  std::unique_ptr<util::NavigationEditor> navigation_editor_;
  std::unique_ptr<util::NavigationProvider> navigation_provider_;

  bool road_deviation_correction_enabled_ = false;

  // The underlying SimulationWorld object, owned by the
  // SimulationWorldService instance.
  SimulationWorld world_;

  bool ready_to_send_ = false;

  // Whether the sim_world is ready to push to frontend
  std::atomic<bool> ready_to_push_;
};

}  // namespace navi_generator
}  // namespace apollo

#endif  // MODULES_TOOLS_NAVI_GENERATOR_BACKEND_HMI_TOPICS_SERVICE_H_
