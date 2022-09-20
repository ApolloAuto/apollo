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
#include "modules/perception/lidar/lib/pointcloud_detection_postprocessor/pointcloud_get_objects/pointcloud_get_objects.h"
#include "modules/perception/lidar/lib/pointcloud_detection_preprocessor/pointcloud_down_sample/pointcloud_down_sample.h"

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
  plugin_factory_.Register(
      PluginType::POINTCLOUD_GET_OBJECTS,
      [](const PluginConfig& plugin_config) -> Plugin* {
        return new lidar::PointCloudGetObjects(plugin_config);
      });
  plugin_factory_.Register(
      PluginType::POINTCLOUD_DOWN_SAMPLE,
      [](const PluginConfig& plugin_config) -> Plugin* {
        return new lidar::PointCloudDownSample(plugin_config);
      });
  plugin_factory_.Register(
      PluginType::CCRF_ONESHOT_TYPE_FUSION,
      [](const PluginConfig& plugin_config) -> Plugin* {
        return new lidar::CCRFOneShotTypeFusion(plugin_config);
      });
  plugin_factory_.Register(
      PluginType::CCRF_SEQUENCE_TYPE_FUSION,
      [](const PluginConfig& plugin_config) -> Plugin* {
        return new lidar::CCRFSequenceTypeFusion(plugin_config);
      });
  plugin_factory_.Register(
      PluginType::ROI_BOUNDARY_FILTER,
      [](const PluginConfig& plugin_config) -> Plugin* {
        return new lidar::ROIBoundaryFilter(plugin_config);
      });
  plugin_factory_.Register(
      PluginType::POINTCLOUD_GET_OBJECTS,
      [](const PluginConfig& plugin_config) -> Plugin* {
        return new lidar::PointCloudGetObjects(plugin_config);
      });
  plugin_factory_.Register(
      PluginType::POINTCLOUD_DOWN_SAMPLE,
      [](const PluginConfig& plugin_config) -> Plugin* {
        return new lidar::PointCloudDownSample(plugin_config);
      });
  plugin_factory_.Register(
      PluginType::MLF_TRACK_OBJECT_MATCHER,
      [](const PluginConfig& plugin_config) -> Plugin* {
        return new lidar::MlfTrackObjectMatcher(plugin_config);
      });
  plugin_factory_.Register(
      PluginType::MLF_TRACKER,
      [](const PluginConfig& plugin_config) -> Plugin* {
        return new lidar::MlfTracker(plugin_config);
      });
}

std::unique_ptr<Plugin> PluginFactory::CreatePlugin(
    const PluginConfig& plugin_config) {
  return plugin_factory_.CreateObject(
            plugin_config.plugin_type(), plugin_config);
}

} // namespace pipeline
} // namespace perception
} // namespace apollo
