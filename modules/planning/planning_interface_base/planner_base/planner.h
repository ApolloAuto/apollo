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

#include <cxxabi.h>

#include "modules/common_msgs/basic_msgs/pnc_point.pb.h"

#include "cyber/common/file.h"
#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/common/status/status.h"
#include "modules/planning/planning_base/common/dependency_injector.h"
#include "modules/planning/planning_base/common/frame.h"

/**
 * @namespace apollo::planning
 * @brief apollo::planning
 */
namespace apollo {
namespace planning {

/**
 * @class Planner
 * @brief Planner is a base class for specific planners.
 *        It contains a pure virtual function Plan which must be implemented in
 * derived class.
 */
class Planner {
 public:
  /**
   * @brief Destructor
   */
  virtual ~Planner() = default;

  virtual std::string Name() = 0;

  virtual apollo::common::Status Init(
      const std::shared_ptr<DependencyInjector>& injector,
      const std::string& config_path = "") {
    injector_ = injector;
    return common::Status::OK();
  }

  /**
   * @brief Compute trajectories for execution.
   * @param planning_init_point The trajectory point where planning starts.
   * @param frame Current planning frame.
   * @return OK if planning succeeds; error otherwise.
   */
  virtual apollo::common::Status Plan(
      const common::TrajectoryPoint& planning_init_point, Frame* frame,
      ADCTrajectory* ptr_computed_trajectory) = 0;

  virtual void Stop() = 0;

  virtual void Reset(Frame* frame) {}

  template <typename T>
  bool LoadConfig(const std::string& custom_config_path, T* config);

 protected:
  std::shared_ptr<DependencyInjector> injector_ = nullptr;
};

template <typename T>
bool Planner::LoadConfig(const std::string& custom_config_path, T* config) {
  std::string config_path = custom_config_path;
  // Get the default config file if "custom_config_path" is empty.
  if ("" == config_path) {
    int status;
    // Get the name of this class.
    std::string class_name =
        abi::__cxa_demangle(typeid(*this).name(), 0, 0, &status);
    config_path = apollo::cyber::plugin_manager::PluginManager::Instance()
                      ->GetPluginConfPath<Planner>(
                          class_name, "conf/planner_config.pb.txt");
  }
  return apollo::cyber::common::LoadConfig<T>(config_path, config);
}

class PlannerWithReferenceLine : public Planner {
 public:
  /**
   * @brief Destructor
   */
  virtual ~PlannerWithReferenceLine() = default;

  /**
   * @brief Compute a trajectory for execution.
   * @param planning_init_point The trajectory point where planning starts.
   * @param frame Current planning frame.
   * @param reference_line_info The computed reference line.
   * @return OK if planning succeeds; error otherwise.
   */
  virtual apollo::common::Status PlanOnReferenceLine(
      const common::TrajectoryPoint& planning_init_point, Frame* frame,
      ReferenceLineInfo* reference_line_info) {
    CHECK_NOTNULL(frame);
    return apollo::common::Status::OK();
  }
};

}  // namespace planning
}  // namespace apollo
