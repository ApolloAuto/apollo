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

/// @brief 
/// @tparam T 
/// @param custom_config_path 表示自定义配置文件路径
/// @param config 指向类型 T 的指针 config（用于存放加载的配置）
/// @return 
// 如果提供了自定义的配置文件路径，就加载它。如果没有提供，则通过反射机制获取当前类的名字，并根据类名查找默认的配置文件路径，最终加载配置文件并填充配置对象
template <typename T>
bool Planner::LoadConfig(const std::string& custom_config_path, T* config) {
  std::string config_path = custom_config_path;
  // Get the default config file if "custom_config_path" is empty.
  if ("" == config_path) {
    int status;
    // Get the name of this class.
    //通过 typeid(*this).name() 获取当前对象的类型信息，并使用 abi::__cxa_demangle 函数将类型名称解码为更可读的格式
    // （例如，从 typeid 返回的名称通常是编译器特定的符号，需要解码）。最终，解码后的类名存储在 class_name 字符串中
    std::string class_name =
        abi::__cxa_demangle(typeid(*this).name(), 0, 0, &status);
    config_path = apollo::cyber::plugin_manager::PluginManager::Instance()
                      ->GetPluginConfPath<Planner>(
                          class_name, "conf/planner_config.pb.txt");
  }
  // 读取配置文件并将内容填充到 config 指向的对象中
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
