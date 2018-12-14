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

#include "modules/tools/navi_generator/backend/hmi/hmi_worker.h"

#include <vector>

#include "gflags/gflags.h"
#include "modules/common/log.h"
#include "modules/common/util/file.h"
#include "modules/common/util/map_util.h"
#include "modules/tools/navi_generator/backend/common/navi_generator_gflags.h"

namespace apollo {
namespace navi_generator {
namespace {

using apollo::common::util::ContainsKey;
using apollo::common::util::FindOrNull;
using apollo::common::util::GetProtoFromFile;
using google::protobuf::Map;
using RLock = boost::shared_lock<boost::shared_mutex>;
using WLock = boost::unique_lock<boost::shared_mutex>;

// Run a supported command of some component, return the ret code.
int RunComponentCommand(const Map<std::string, Component> &components,
                        const std::string &component_name,
                        const std::string &command_name) {
  const auto *component = FindOrNull(components, component_name);
  if (component == nullptr) {
    AERROR << "Cannot find component " << component_name;
    return -1;
  }
  const auto *cmd = FindOrNull(component->supported_commands(), command_name);
  if (cmd == nullptr) {
    AERROR << "Cannot find command " << component_name << "." << command_name;
    return -1;
  }
  AINFO << "Execute system command: " << *cmd;
  const int ret = std::system(cmd->c_str());

  AERROR_IF(ret != 0) << "Command returns " << ret << ": " << *cmd;
  return ret;
}
}  // namespace

HMIWorker::HMIWorker() {
  // Init HMIConfig.
  CHECK(common::util::GetProtoFromFile(FLAGS_hmi_config_filename, &config_))
      << "Unable to parse HMI config file " << FLAGS_hmi_config_filename;

  ADEBUG << "Loaded HMI config: " << config_.DebugString();
  // Init HMIStatus.
  const auto &modes = config_.modes();
  if (!ContainsKey(modes, status_.current_mode())) {
    CHECK(!modes.empty()) << "No available modes to run vehicle.";
    // If the default mode is unavailable, select the first one.
    status_.set_current_mode(modes.begin()->first);
  }
}

int HMIWorker::RunModuleCommand(const std::string &module,
                                const std::string &command) {
  return RunComponentCommand(config_.modules(), module, command);
}

int HMIWorker::RunHardwareCommand(const std::string &hardware,
                                  const std::string &command) {
  return RunComponentCommand(config_.hardware(), hardware, command);
}

void HMIWorker::RunModeCommand(const std::string &command_name) {
  std::string current_mode;
  {
    RLock rlock(status_mutex_);
    current_mode = status_.current_mode();
  }
  const Mode &mode_conf = config_.modes().at(current_mode);
  if (command_name == "start" || command_name == "stop") {
    // Run the command on all live modules.
    for (const auto &module : mode_conf.live_modules()) {
      RunModuleCommand(module, command_name);
    }
  }
}

void HMIWorker::UpdateSystemStatus(const monitor::SystemStatus &system_status) {
  WLock wlock(status_mutex_);
  *status_.mutable_system_status() = system_status;
}

}  // namespace navi_generator
}  // namespace apollo
