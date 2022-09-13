/******************************************************************************
 * Copyright 2022 The Apollo Authors. All Rights Reserved.
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


#include "modules/perception/pipeline/plugin_factory.h"

#include "modules/perception/lidar/lib/object_filter_bank/roi_boundary_filter/roi_boundary_filter.h"

namespace apollo {
namespace perception {
namespace pipeline {


apollo::common::util::Factory<
    PluginType, Plugin,
    Plugin *(*)(const PluginConfig& config),
    std::unordered_map<
        PluginType,
        Plugin *(*)(const PluginConfig& config),
        std::hash<int>>>
    PluginFactory::plugin_factory_;

void PluginFactory::Init() {
  plugin_factory_.Register(
      PluginType::ROI_BOUNDARY_FILTER,
      [](const PluginConfig& plugin_config) -> Plugin* {
        return new lidar::ROIBoundaryFilter(plugin_config);
      });
  // Todo(zero): need to add more type
  // need to deal with PipelineConfig& config

}

std::unique_ptr<Plugin> PluginFactory::CreatePlugin(
    const PluginConfig& plugin_config) {
  return plugin_factory_.CreateObject(
            plugin_config.plugin_type(), plugin_config);
}

} // namespace pipeline
} // namespace perception
} // namespace apollo
