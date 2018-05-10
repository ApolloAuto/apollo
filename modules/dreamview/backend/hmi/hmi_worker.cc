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

#include "modules/dreamview/backend/hmi/hmi_worker.h"

#include <vector>

#include "gflags/gflags.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/kv_db/kv_db.h"
#include "modules/common/util/file.h"
#include "modules/common/util/map_util.h"
#include "modules/common/util/string_tokenizer.h"
#include "modules/common/util/string_util.h"
#include "modules/control/proto/pad_msg.pb.h"
#include "modules/data/util/info_collector.h"
#include "modules/dreamview/backend/common/dreamview_gflags.h"
#include "modules/dreamview/backend/hmi/vehicle_manager.h"

DEFINE_string(map_data_path, "/apollo/modules/map/data", "Path to map data.");

DEFINE_string(vehicle_data_path, "/apollo/modules/calibration/data",
              "Path to vehicle data.");

namespace apollo {
namespace dreamview {
namespace {

using apollo::canbus::Chassis;
using apollo::common::adapter::AdapterManager;
using apollo::common::util::ContainsKey;
using apollo::common::util::FindOrNull;
using apollo::common::util::StringTokenizer;
using apollo::control::DrivingAction;
using google::protobuf::Map;
using RLock = boost::shared_lock<boost::shared_mutex>;
using WLock = boost::unique_lock<boost::shared_mutex>;

constexpr char kNavigationModeName[] = "Navigation";

// Convert a string to be title-like. E.g.: "hello_world" -> "Hello World".
std::string TitleCase(const std::string &origin,
                      const std::string &delimiter = "_") {
  std::vector<std::string> parts = StringTokenizer::Split(origin, delimiter);
  for (auto &part : parts) {
    if (!part.empty()) {
      // Upper case the first char.
      part[0] = toupper(part[0]);
    }
  }

  return apollo::common::util::PrintIter(parts);
}

// List subdirs and return a dict of {subdir_title: subdir_path}.
Map<std::string, std::string> ListDirAsDict(const std::string &dir) {
  Map<std::string, std::string> result;
  const auto subdirs = apollo::common::util::ListSubDirectories(dir);
  for (const auto &subdir : subdirs) {
    const auto subdir_title = TitleCase(subdir);
    const auto subdir_path = apollo::common::util::StrCat(dir, "/", subdir);
    result.insert({subdir_title, subdir_path});
  }
  return result;
}

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

template <class FlagType, class ValueType>
void SetGlobalFlag(const std::string &flag_name, const ValueType &value,
                   FlagType *flag) {
  static constexpr char kGlobalFlagfile[] =
      "/apollo/modules/common/data/global_flagfile.txt";
  if (*flag != value) {
    *flag = value;
    // Overwrite global flagfile.
    std::ofstream fout(kGlobalFlagfile, std::ios_base::app);
    CHECK(fout) << "Fail to open global flagfile " << kGlobalFlagfile;
    fout << "\n--" << flag_name << "=" << value << std::endl;
  }
}

}  // namespace

HMIWorker::HMIWorker() {
  // Init HMIConfig.
  CHECK(common::util::GetProtoFromFile(FLAGS_hmi_config_filename, &config_))
      << "Unable to parse HMI config file " << FLAGS_hmi_config_filename;
  config_.set_docker_image(apollo::data::InfoCollector::GetDockerImage());

  // If the module path doesn't exist, remove it from list.
  auto *modules = config_.mutable_modules();
  for (auto iter = modules->begin(); iter != modules->end();) {
    const auto &conf = iter->second;
    if (conf.has_path() && !common::util::PathExists(conf.path())) {
      iter = modules->erase(iter);
    } else {
      ++iter;
    }
  }

  // Get available maps and vehicles by listing data directory.
  *config_.mutable_available_maps() = ListDirAsDict(FLAGS_map_data_path);
  *config_.mutable_available_vehicles() =
      ListDirAsDict(FLAGS_vehicle_data_path);
  config_.set_utm_zone_id(FLAGS_local_utm_zone_id);
  ADEBUG << "Loaded HMI config: " << config_.DebugString();

  // Init HMIStatus.
  const auto &modes = config_.modes();
  if (FLAGS_use_navigation_mode && ContainsKey(modes, kNavigationModeName)) {
    // If the FLAG_use_navigation_mode is set, set it in HMIStatus.
    status_.set_current_mode(kNavigationModeName);
  } else if (!ContainsKey(modes, status_.current_mode())) {
    CHECK(!modes.empty()) << "No available modes to run vehicle.";
    // If the default mode is unavailable, select the first one.
    status_.set_current_mode(modes.begin()->first);
  }

  // If the FLAGS_map_dir is set, set it in HMIStatus.
  if (!FLAGS_map_dir.empty()) {
    for (const auto &entry : config_.available_maps()) {
      // entry is (map_name, map_path)
      if (entry.second == FLAGS_map_dir) {
        status_.set_current_map(entry.first);
        apollo::common::KVDB::Put("apollo:dreamview:map", entry.first);
        break;
      }
    }
  }
}

bool HMIWorker::Trigger(const HMIAction action) {
  AINFO << "HMIAction " << HMIAction_Name(action) << " was triggered!";
  switch (action) {
    case HMIAction::SETUP:
      RunModeCommand("start");
      break;
    case HMIAction::AUTO_MODE:
      return ChangeToDrivingMode(Chassis::COMPLETE_AUTO_DRIVE);
    case HMIAction::DISENGAGE:
      return ChangeToDrivingMode(Chassis::COMPLETE_MANUAL);
    default:
      AERROR << "HMIAction not implemented, yet!";
      return false;
  }
  return true;
}

int HMIWorker::RunModuleCommand(const std::string &module,
                                const std::string &command) {
  return RunComponentCommand(config_.modules(), module, command);
}

int HMIWorker::RunHardwareCommand(const std::string &hardware,
                                  const std::string &command) {
  return RunComponentCommand(config_.hardware(), hardware, command);
}

int HMIWorker::RunToolCommand(const std::string &tool,
                              const std::string &command) {
  return RunComponentCommand(config_.tools(), tool, command);
}

void HMIWorker::SubmitDriveEvent(const uint64_t event_time_ms,
                                 const std::string &event_msg) {
  apollo::common::DriveEvent drive_event;
  AdapterManager::FillDriveEventHeader("HMI", &drive_event);
  drive_event.mutable_header()->set_timestamp_sec(event_time_ms / 1000.0);
  drive_event.set_event(event_msg);
  AdapterManager::PublishDriveEvent(drive_event);
}

bool HMIWorker::ChangeToDrivingMode(const Chassis::DrivingMode mode) {
  // Always reset to MANUAL mode before changing to other mode.
  if (mode != Chassis::COMPLETE_MANUAL) {
    if (!ChangeToDrivingMode(Chassis::COMPLETE_MANUAL)) {
      AERROR << "Failed to reset to MANUAL mode before changing to "
             << Chassis::DrivingMode_Name(mode);
      return false;
    }
  }

  control::PadMessage pad;
  switch (mode) {
    case Chassis::COMPLETE_MANUAL:
      pad.set_action(DrivingAction::RESET);
      break;
    case Chassis::COMPLETE_AUTO_DRIVE:
      pad.set_action(DrivingAction::START);
      break;
    default:
      AFATAL << "Unknown action to change driving mode to "
             << Chassis::DrivingMode_Name(mode);
  }

  constexpr int kMaxTries = 3;
  constexpr auto kTryInterval = std::chrono::milliseconds(500);
  auto *chassis = CHECK_NOTNULL(AdapterManager::GetChassis());
  for (int i = 0; i < kMaxTries; ++i) {
    // Send driving action periodically until entering target driving mode.
    AdapterManager::FillPadHeader("HMI", &pad);
    AdapterManager::PublishPad(pad);

    std::this_thread::sleep_for(kTryInterval);
    chassis->Observe();
    if (chassis->Empty()) {
      AERROR << "No Chassis message received!";
    } else if (chassis->GetLatestObserved().driving_mode() == mode) {
      return true;
    }
  }
  AERROR << "Failed to change driving mode to "
         << Chassis::DrivingMode_Name(mode);
  return false;
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

void HMIWorker::ChangeToMap(const std::string &map_name,
                            MapService *map_service) {
  const auto *map_dir = FindOrNull(config_.available_maps(), map_name);
  if (map_dir == nullptr) {
    AERROR << "Unknown map " << map_name;
    return;
  }

  {
    // Update current_map status.
    WLock wlock(status_mutex_);
    if (status_.current_map() == map_name) {
      return;
    }
    status_.set_current_map(map_name);
  }
  apollo::common::KVDB::Put("apollo:dreamview:map", map_name);

  SetGlobalFlag("map_dir", *map_dir, &FLAGS_map_dir);
  // Also reload simulation map.
  CHECK(map_service->ReloadMap(true)) << "Failed to load map from " << *map_dir;
  RunModeCommand("stop");
}

void HMIWorker::ChangeToVehicle(const std::string &vehicle_name) {
  const auto *vehicle = FindOrNull(config_.available_vehicles(), vehicle_name);
  if (vehicle == nullptr) {
    AERROR << "Unknown vehicle " << vehicle_name;
    return;
  }

  {
    // Update current_vehicle status.
    WLock wlock(status_mutex_);
    if (status_.current_vehicle() == vehicle_name) {
      return;
    }
    status_.set_current_vehicle(vehicle_name);
  }
  apollo::common::KVDB::Put("apollo:dreamview:vehicle", vehicle_name);

  CHECK(VehicleManager::instance()->UseVehicle(*vehicle));
  RunModeCommand("stop");
}

void HMIWorker::ChangeToMode(const std::string &mode_name) {
  if (!ContainsKey(config_.modes(), mode_name)) {
    AERROR << "Unknown mode " << mode_name;
    return;
  }

  std::string old_mode;
  {
    // Update current_mode status.
    WLock wlock(status_mutex_);
    old_mode = status_.current_mode();
    if (old_mode == mode_name) {
      return;
    }
    status_.set_current_mode(mode_name);
  }
  apollo::common::KVDB::Put("apollo:dreamview:mode", mode_name);

  const auto &old_modules = config_.modes().at(old_mode).live_modules();
  const bool use_navigation_mode = (mode_name == kNavigationModeName);
  SetGlobalFlag("use_navigation_mode", use_navigation_mode,
                &FLAGS_use_navigation_mode);
  // Now stop all old modules.
  for (const auto &module : old_modules) {
    RunModuleCommand(module, "stop");
  }
}

void HMIWorker::UpdateSystemStatus(const monitor::SystemStatus &system_status) {
  WLock wlock(status_mutex_);
  *status_.mutable_system_status() = system_status;
}

}  // namespace dreamview
}  // namespace apollo
