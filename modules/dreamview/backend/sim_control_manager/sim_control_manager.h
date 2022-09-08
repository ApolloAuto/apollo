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

#include "cyber/cyber.h"

#include "modules/common/configs/config_gflags.h"
#include "modules/dreamview/backend/sim_control_manager/common/sim_control_gflags.h"
#include "modules/dreamview/backend/sim_control_manager/core/dynamic_model_factory.h"
#include "modules/dreamview/backend/sim_control_manager/core/sim_control_base.h"

/**
 * @namespace apollo::dreamview
 * @brief apollo::dreamview
 */
namespace apollo {
namespace dreamview {

/**
 * @class SimControlManager
 */
class SimControlManager {
 public:
  SimControlManager() {}
  bool IsEnabled() const { return enabled_; }
  nlohmann::json LoadDynamicModels();
  bool ChangeDynamicModel(std::string& dynamic_model_name);
  void DeleteDynamicModel(std::string& dynamic_model_name);
  bool ResetDynamicModel();

  virtual ~SimControlManager() {
    Stop();
  }

  std::string Name() const;

  void RunOnce();

  /**
   * @brief module initialization function
   * @return initialization status
   */
  bool Init(bool set_start_point, double start_velocity = 0.0,
            double start_acceleration = 0.0,
            double start_heading = std::numeric_limits<double>::max());

  /**
   * @brief module start function
   */
  void Start();

  /**
   * @brief module stop function
   */
  void Stop();

 private:
  std::unique_ptr<SimControlBase> model_ptr_;
  std::string current_dynamic_model_ = "";
  // whether the sim control manager is enabled!
  // 与sim control这种dm的enabled区分。enabled 等于正在运行
  // 此处enabled 只是控制sim control manager 的开关
  // 相当于标识了是否开着
  bool enabled_ = false;
};

}  // namespace simulation
}  // namespace apollo
