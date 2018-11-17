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

#ifndef MODULES_DREAMVIEW_BACKEND_SIM_CONTROL_SIM_CONTROL_INTERFACE_H_
#define MODULES_DREAMVIEW_BACKEND_SIM_CONTROL_SIM_CONTROL_INTERFACE_H_

/**
 * @namespace apollo::dreamview
 * @brief apollo::dreamview
 */
namespace apollo {
namespace dreamview {

/**
 * @class SimControlInterface
 *
 * @brief Interface of simulated control algorithm
 */
class SimControlInterface {
 public:
  /**
   * @brief Main logic of the simulated control algorithm.
   */
  virtual void RunOnce() = 0;

  /**
   * @brief Initialization.
   */
  virtual void Init(bool set_start_point, double start_velocity = 0.0,
                    double start_acceleration = 0.0) = 0;

  /**
   * @brief Starts running the simulated control algorithm, e.g., publish
   * simulated localization and chassis messages triggered by timer.
   */
  virtual void Start() = 0;

  /**
   * @brief Stops the algorithm.
   */
  virtual void Stop() = 0;

  /**
   * @brief Resets the internal state.
   */
  virtual void Reset() = 0;
};

}  // namespace dreamview
}  // namespace apollo

#endif /* MODULES_DREAMVIEW_BACKEND_SIM_CONTROL_SIM_CONTROL_INTERFACE_H_ */
