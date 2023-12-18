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
 * @file updater_manager.h
 * @brief UpdaterManager to manage all data updater.
 * @date 2023/06/29
 */

#pragma once
#include <map>
#include <string>

#include "cyber/common/log.h"
#include "modules/dreamview/backend/common/handlers/websocket_handler.h"
#include "modules/dreamview_plus/backend/updater/updater_base.h"

namespace apollo {
namespace dreamview {

using apollo::dreamview::UpdaterBase;

/**
 * @class UpdaterManager
 * @brief Management for all data updater.
 */
class UpdaterManager {
 public:
  UpdaterManager() {}
  virtual ~UpdaterManager() {}

  /**
   * @brief Register data updater by websocket path name.
   * @param path_name The path name of the data updater.
   * @param updater  The data updater.
   */
  void RegisterUpdater(std::string path_name, UpdaterBase *updater);

  /**
   * @brief Start a updater implemetnent.
   * @param path_name The path name of the data updater.
   * @param time_interval_ms  Data update frequency,
   * If field time_interval_ms equals to 0, it is considered
   * a single subscribe and will not start timer
   * @param subscribe_param bring extra params for some updater.
   */
  bool Start(const std::string &path_name, const double &time_interval_ms,
             const std::string &channel_name = "",
             nlohmann::json *subscribe_param = nullptr);

  /**
   * @brief Stop updater publish data.
   * @param path_name The path name of the data updater.
   */
  bool Stop(const std::string &path_name, const std::string& channel_name);

  /**
   * @brief Get registered updater
   * @param path_name The path name of the data updater.
   */
  UpdaterBase* GetUpdater(const std::string &path_name);

 private:
  std::map<std::string, UpdaterBase *> updater_map_;
};

}  // namespace dreamview
}  // namespace apollo
