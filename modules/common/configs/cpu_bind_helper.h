/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#ifndef MODULES_CONFIGS_CPU_BIND_H_
#define MODULES_CONFIGS_CPU_BIND_H_

#include <string.h>
#include <unordered_map>
#include <vector>

#include "gtest/gtest_prod.h"

#include "modules/common/macro.h"

/**
 * @namespace apollo::common
 * @brief apollo::common
 */
namespace apollo {
namespace common {

/**
 * @class CpuBindHelper
 *
 * @Brief This is a helper class that helps bind a specified process on a cpu core.
 * The binding rules are defined in modules/common/data/cpu_set.json
 */
class CpuBindHelper {
public:
  /**
   * @brief Load the content of the binding rules file into memory.
   * Then, process function can call BindCpu function to bind cpu with
   * the processname as the parameter.
   */
  void Init(const std::string &config_file);

  /**
   * @brief Given the process name, and bind current process on the reserved cpu.
   * If the given process name can not be found, then nothing will happen,
   * so changing the process name to another in configuration file is a 
   * easy way to cancel binding for a process.
   */
  void BindCpu(const std::string& module_name);

private:
  std::unordered_map<std::string, std::vector<int>> bindrule_map_;
  bool init_ = false;

  FRIEND_TEST(CpuBindHelperTest, Init_BindCpu);

  DECLARE_SINGLETON(CpuBindHelper);
};

}  // namespace common
}  // namespace apollo

#endif  // MODULES_CONFIGS_CPU_BIND_H_


