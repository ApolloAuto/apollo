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

#include <limits>
#include <memory>
#include <string>

#include "cyber/cyber.h"
#include "cyber/common/macros.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/dreamview/backend/common/sim_control_manager/common/sim_control_gflags.h"
#include "modules/dreamview/backend/common/sim_control_manager/core/dynamic_model_factory.h"
#include "modules/dreamview/backend/common/sim_control_manager/core/sim_control_base.h"

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
  bool IsEnabled() const { return enabled_; }
  nlohmann::json LoadDynamicModels();
  bool AddDynamicModel(const std::string &dynamic_model_name);
  bool ChangeDynamicModel(const std::string &dynamic_model_name);
  bool DeleteDynamicModel(const std::string &dynamic_model_name);
  void ResetDynamicModel();
  void Restart(double x, double y, double v = 0.0, double a = 0.0);
  void ReSetPoinstion(double x, double y, double heading);
  void Restart();
  void Reset();

  virtual ~SimControlManager() { Stop(); }

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
  SimControlBase *model_ptr_ = nullptr;
  std::string current_dynamic_model_ = "";
  bool enabled_ = false;

  DECLARE_SINGLETON(SimControlManager)
};

}  // namespace dreamview
}  // namespace apollo
