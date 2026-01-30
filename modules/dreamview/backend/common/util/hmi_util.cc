/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include "modules/dreamview/backend/common/util/hmi_util.h"

#include <vector>

#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"
#include "absl/strings/str_split.h"

#include "cyber/common/file.h"
#include "modules/common/util/map_util.h"
#include "modules/dreamview/backend/common/dreamview_gflags.h"

namespace apollo {
namespace dreamview {
namespace util {
using apollo::dreamview::HMIConfig;
using apollo::dreamview::HMIMode;
using google::protobuf::Map;

std::string HMIUtil::TitleCase(std::string_view origin) {
  std::vector<std::string> parts = absl::StrSplit(origin, '_');
  for (auto &part : parts) {
    if (!part.empty()) {
      // Upper case the first char.
      part[0] = static_cast<char>(toupper(part[0]));
    }
  }

  return absl::StrJoin(parts, " ");
}

// List files by pattern and return a dict of {file_title: file_path}.
Map<std::string, std::string> ListFilesAsDict(std::string_view dir,
                                              std::string_view extension) {
  Map<std::string, std::string> result;
  const std::string pattern = absl::StrCat(dir, "/*", extension);
  for (const std::string &file_path : cyber::common::Glob(pattern)) {
    // Remove the extension and convert to title case as the file title.
    const std::string filename = cyber::common::GetFileName(file_path);
    const std::string file_title = HMIUtil::TitleCase(
        filename.substr(0, filename.length() - extension.length()));
    result.insert({file_title, file_path});
  }
  return result;
}

// List subdirs and return a dict of {subdir_title: subdir_path}.
Map<std::string, std::string> HMIUtil::ListDirAsDict(const std::string &dir) {
  Map<std::string, std::string> result;
  const auto subdirs = cyber::common::ListSubPaths(dir);
  for (const auto &subdir : subdirs) {
    const auto subdir_title = HMIUtil::TitleCase(subdir);
    const auto subdir_path = absl::StrCat(dir, "/", subdir);
    result.insert({subdir_title, subdir_path});
  }
  return result;
}

HMIConfig HMIUtil::LoadConfig(std::string config_path) {
  HMIConfig config;
  if (config_path.empty()) {
    *config.mutable_modes() =
        ListFilesAsDict(FLAGS_dv_hmi_modes_config_path, ".pb.txt");
    Map<std::string, std::string> dv_plus_modes =
        ListFilesAsDict(FLAGS_dv_plus_hmi_modes_config_path, ".pb.txt");
    for (auto iter = dv_plus_modes.begin(); iter != dv_plus_modes.end();
         iter++) {
    config.mutable_modes()->insert({iter->first, iter->second});
    }
  } else {
    // Get available modes, maps and vehicles by listing data directory.
    *config.mutable_modes() =
        ListFilesAsDict(config_path, ".pb.txt");
  }
  ACHECK(!config.modes().empty())
      << "No modes config loaded";

  *config.mutable_maps() = ListDirAsDict(FLAGS_maps_data_path);
  *config.mutable_vehicles() = ListDirAsDict(FLAGS_vehicles_config_path);
  AINFO << "Loaded HMI config: " << config.DebugString();
  return config;
}

HMIMode HMIUtil::LoadMode(const std::string &mode_config_path) {
  HMIMode mode;
  ACHECK(cyber::common::GetProtoFromFile(mode_config_path, &mode))
      << "Unable to parse HMIMode from file " << mode_config_path;
  TranslateCyberModules(&mode);
  // For global components.
  HMIMode mode_temp;
  ACHECK(cyber::common::GetProtoFromFile(FLAGS_global_components_config_path,
                                         &mode_temp))
      << "Unable to parse HMIMode from file "
      << FLAGS_global_components_config_path;
  for (const auto &iter : mode_temp.global_components()) {
    (*mode.mutable_global_components())[iter.first] = iter.second;
  }
  AINFO << "Loaded HMI mode: " << mode.DebugString();
  return mode;
}

void HMIUtil::TranslateCyberModules(HMIMode *mode) {
  // Translate cyber_modules to regular modules.
  for (const auto &iter : mode->cyber_modules()) {
    const std::string &module_name = iter.first;
    const CyberModule &cyber_module = iter.second;
    // Each cyber module should have at least one dag file.
    ACHECK(!cyber_module.dag_files().empty())
        << "None dag file is provided for " << module_name;

    Module &module = LookupOrInsert(mode->mutable_modules(), module_name, {});
    module.set_required_for_safety(cyber_module.required_for_safety());

    // Construct start_command:
    //     nohup mainboard -p <process_group> -d <dag> ... &
    module.set_start_command("nohup mainboard");
    const auto &process_group = cyber_module.process_group();
    if (!process_group.empty()) {
      absl::StrAppend(module.mutable_start_command(), " -p ", process_group);
    }
    for (const std::string &dag : cyber_module.dag_files()) {
      absl::StrAppend(module.mutable_start_command(), " -d ", dag);
    }
    absl::StrAppend(module.mutable_start_command(), " &");

    // Construct stop_command: pkill -f '<dag[0]>'
    const std::string &first_dag = cyber_module.dag_files(0);
    module.set_stop_command(absl::StrCat("pkill -f \"", first_dag, "\""));
    // Construct process_monitor_config.
    module.mutable_process_monitor_config()->add_command_keywords("mainboard");
    module.mutable_process_monitor_config()->add_command_keywords(first_dag);
  }
  mode->clear_cyber_modules();
}

}  // namespace util
}  // namespace dreamview
}  // namespace apollo
