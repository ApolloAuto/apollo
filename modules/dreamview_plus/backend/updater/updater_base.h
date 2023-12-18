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

/**
 * @file UpdaterBase.h
 * @brief UpdaterBase class
 * @date 2023/06/20
 */
#pragma once
#include <string>
#include "nlohmann/json.hpp"

namespace apollo {

namespace dreamview {

/**
 * @class UpdaterBase
 * @brief Base Class for all data updater.
 * The updater is responsible for maintaining and updating a data type used by
 * the dreamview module, such as simulation_world, pointcloud, camera,
 * pointcloud etc. Each type of data is a data stream that supports front-end
 * and back-end communication of dreamview.
 */
class UpdaterBase {
 public:
  /**
   * @brief Updaterbase
   */
  UpdaterBase();

  /**
   * @brief Start data flow.
   * @param time_interval_ms Data stream sending frequency.
   * 0 means single subscribe
   * @param subscribe_param: subscribe some updater may need extra params
   */
  virtual void StartStream(const double& time_interval_ms,
                           const std::string& channel_name = "",
                           nlohmann::json* subscribe_param = nullptr) = 0;

  /**
   * @brief Stop data flow.
   */
  virtual void StopStream(const std::string& channel_name = "") = 0;

  /**
   * @brief Publish Message to dreamview frontend.
   */
  virtual void PublishMessage(const std::string& channel_name = "") = 0;
  virtual ~UpdaterBase() {}
};

}  // namespace dreamview
}  // namespace apollo
