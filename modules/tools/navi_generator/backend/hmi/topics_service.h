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
#include "modules/tools/navi_generator/backend/webserver/navi_generator_websocket.h"

namespace apollo {
namespace navi_generator {

/**
 * @class TopicsService
 * @brief
 * NOTE: This class is not thread-safe.
 */
class TopicsService {
 public:
  // The maximum number of monitor message items to be kept in
  // SimulationWorld.
  static constexpr int kMaxMonitorItems = 30;

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

 private:
  // The pointer of NaviGeneratorWebSocket, not owned by TopicsService.
  NaviGeneratorWebSocket *websocket_ = nullptr;
};

}  // namespace navi_generator
}  // namespace apollo

#endif  // MODULES_TOOLS_NAVI_GENERATOR_BACKEND_HMI_TOPICS_SERVICE_H_
