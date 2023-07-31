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

#include "modules/perception/lidar_detection_filter/object_filter_bank/object_filter_bank.h"

#include "modules/perception/lidar_detection_filter/object_filter_bank/proto/filter_bank_config.pb.h"

#include "cyber/common/file.h"
#include "modules/perception/common/util.h"
#include "modules/perception/common/lidar/common/config_util.h"
#include "modules/perception/common/lidar/common/lidar_log.h"
namespace apollo {
namespace perception {
namespace lidar {

bool ObjectFilterBank::Init(const ObjectFilterInitOptions& options) {
  std::string config_file =
      GetConfigFile(options.config_path, options.config_file);

  FilterBankConfig config;
  ACHECK(apollo::cyber::common::GetProtoFromFile(config_file, &config));
  filter_bank_.clear();
  for (int i = 0; i < config.plugins_size(); ++i) {
    const auto& plugin = config.plugins(i);
    const auto& name = plugin.name();
    std::shared_ptr<BaseObjectFilter> filter =
        apollo::cyber::plugin_manager::PluginManager::Instance()
            ->CreateInstance<BaseObjectFilter>(
                ConfigUtil::GetFullClassName(name));
    if (!filter) {
      AINFO << "Failed to find object filter: " << name << ", skipped";
      continue;
    }
    ObjectFilterInitOptions option;
    option.config_path = plugin.config_path();
    option.config_file = plugin.config_file();
    if (!filter->Init(option)) {
      AINFO << "Failed to init object filter: " << name << ", skipped";
      continue;
    }
    filter_bank_.push_back(filter);
    AINFO << "Filter bank add filter: " << name;
  }
  return true;
}

bool ObjectFilterBank::Filter(const ObjectFilterOptions& options,
                              LidarFrame* frame) {
  size_t object_number = frame->segmented_objects.size();
  for (auto& filter : filter_bank_) {
    if (!filter->Filter(options, frame)) {
      AINFO << "Failed to filter objects in: " << filter->Name();
    }
  }
  AINFO << "Object filter bank, filtered objects size: from " << object_number
        << " to " << frame->segmented_objects.size();
  return true;
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
