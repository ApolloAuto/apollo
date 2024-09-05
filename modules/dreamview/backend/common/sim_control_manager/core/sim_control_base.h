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
#pragma once

#include <limits>
#include <memory>
#include <string>

#include "nlohmann/json.hpp"

#include "modules/common_msgs/localization_msgs/localization.pb.h"
#include "modules/common_msgs/prediction_msgs/prediction_obstacle.pb.h"
#include "modules/dreamview/backend/common/sim_control_manager/proto/sim_control_internal.pb.h"

#include "cyber/common/log.h"
#include "cyber/cyber.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/math/math_utils.h"
#include "modules/common/math/quaternion.h"
#include "modules/common/util/message_util.h"
#include "modules/dreamview/backend/common/dreamview_gflags.h"
#include "modules/dreamview/backend/common/sim_control_manager/common/sim_control_gflags.h"
#include "modules/dreamview/backend/common/sim_control_manager/common/sim_control_util.h"
/**
 * @namespace apollo::dreamview
 * @brief apollo::dreamview
 */
namespace apollo {
namespace dreamview {

/**
 * @class SimControlBase
 *
 * @brief Interface of simulated control algorithm
 */
class SimControlBase {
 public:
  virtual ~SimControlBase() {}
  /**
   * @brief Main logic of the simulated control algorithm.
   */
  virtual void RunOnce() = 0;

  /**
   * @brief Initialization.
   */
  virtual void Init(bool set_start_point, nlohmann::json start_point_attr,
                    bool use_start_point_position = false) = 0;

  /**
   * @brief Starts running the simulated control algorithm, e.g., publish
   * simulated localization and chassis messages triggered by timer.
   */
  virtual void Start() = 0;

  /**
   * @brief Starts running the simulated control algorithm with position, e.g.,
   * publish simulated localization and chassis messages triggered by timer.
   */
  virtual void Start(double x, double y, double v = 0.0, double a = 0.0) = 0;

  /**
   * @brief Set vehicle position.
   */
  virtual void ReSetPoinstion(double x, double y, double heading) = 0;

  /**
   * @brief Stops the algorithm.
   */
  virtual void Stop() = 0;

  /**
   * @brief Resets the internal state.
   */
  virtual void Reset() = 0;

  // virtual std::unique_ptr<SimControlBase> GetDynamicModel();

 protected:
  void TransformToVRF(const apollo::common::Point3D& point_mrf,
                      const apollo::common::Quaternion& orientation,
                      apollo::common::Point3D* point_vrf);

  // The timer to publish simulated localization and chassis messages.
  std::unique_ptr<cyber::Timer> sim_control_timer_;

  // The timer to publish dummy prediction
  std::unique_ptr<cyber::Timer> sim_prediction_timer_;

  // Linearized reader/timer callbacks and external operations.
  std::mutex mutex_;

  // Whether the sim control is enabled / initialized.
  bool enabled_ = false;

  // Whether start point is initialized from actual localization data
  bool start_point_from_localization_ = false;

  // Initial state of the ego vehicle
  double start_velocity_ = 0.0;
  double start_acceleration_ = 0.0;
  double start_heading_ = std::numeric_limits<double>::max();
};

/**
 * @brief Get SimControl class
 */
typedef SimControlBase* create_t(std::string dynamic_name,
                                 std::string home_path);

}  // namespace dreamview
}  // namespace apollo
