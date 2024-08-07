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

#ifndef CYBER_MAINBOARD_MODULE_ARGUMENT_H_
#define CYBER_MAINBOARD_MODULE_ARGUMENT_H_

#include <list>
#include <string>

#include "cyber/common/global_data.h"
#include "cyber/common/log.h"
#include "cyber/common/types.h"

namespace apollo {
namespace cyber {
namespace mainboard {

static const char DEFAULT_process_group_[] = "mainboard_default";
static const char DEFAULT_sched_name_[] = "CYBER_DEFAULT";

// code for command line arguments without short parameters
static const int ARGS_OPT_CODE_PLUGIN = 1001;
static const int ARGS_OPT_CODE_DISABLE_PLUGIN_AUTOLOAD = 1002;

class ModuleArgument {
 public:
  ModuleArgument() = default;
  virtual ~ModuleArgument() = default;
  void DisplayUsage();
  void ParseArgument(int argc, char* const argv[]);
  void GetOptions(const int argc, char* const argv[]);
  const std::string& GetBinaryName() const;
  const std::string& GetProcessGroup() const;
  const std::string& GetSchedName() const;
  const std::list<std::string>& GetDAGConfList() const;
  const std::list<std::string>& GetPluginDescriptionList() const;
  const bool GetEnableCpuprofile() const { return enable_cpuprofile_; }
  const std::string GetProfileFilename() const { return profile_filename_; }
  const bool GetEnableHeapprofile() const { return enable_heapprofile_; }
  const std::string GetHeapProfileFilename() const {
    return heapprofile_filename_;
  }
  const bool& GetDisablePluginsAutoLoad() const;

 private:
  std::list<std::string> dag_conf_list_;
  std::list<std::string> plugin_description_list_;
  std::string binary_name_;
  std::string process_group_;
  std::string sched_name_;
  bool enable_cpuprofile_ = false;
  std::string profile_filename_;
  bool enable_heapprofile_ = false;
  std::string heapprofile_filename_;
  bool disable_plugin_autoload_ = false;
};

inline const std::string& ModuleArgument::GetBinaryName() const {
  return binary_name_;
}

inline const std::string& ModuleArgument::GetProcessGroup() const {
  return process_group_;
}

inline const std::string& ModuleArgument::GetSchedName() const {
  return sched_name_;
}

inline const std::list<std::string>& ModuleArgument::GetDAGConfList() const {
  return dag_conf_list_;
}

inline const std::list<std::string>& ModuleArgument::GetPluginDescriptionList()
    const {
  return plugin_description_list_;
}

inline const bool& ModuleArgument::GetDisablePluginsAutoLoad() const {
  return disable_plugin_autoload_;
}

}  // namespace mainboard
}  // namespace cyber
}  // namespace apollo

#endif  // CYBER_MAINBOARD_MODULE_ARGUMENT_H_
