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

#include "modules/perception/obstacle/lidar/object_filter/low_object_filter/low_object_filter.h"

#include <algorithm>
#include <limits>

#include "modules/perception/common/pcl_types.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/obstacle/base/types.h"

namespace apollo {
namespace perception {

using pcl_util::PointCloud;
using pcl_util::PointCloudPtr;

bool LowObjectFilter::Init() {
  ConfigManager* config_manager = ConfigManager::instance();
  if (!config_manager->Init()) {
    AERROR << "failed to init ConfigManager.";
    return false;
  }

  std::string model_name = "LowObjectFilter";
  const ModelConfig* model_config = config_manager->GetModelConfig(model_name);
  if (model_config == nullptr) {
    AERROR << " not found model: " << model_name;
    return false;
  }

  if (!model_config->GetValue("object_height_threshold",
                              &object_height_threshold_)) {
    AERROR << "object_height_threshold not found.";
    object_height_threshold_ = 0.10;
  }

  if (!model_config->GetValue("object_position_height_threshold",
                              &object_position_height_threshold_)) {
    AERROR << "object_position_height_threshold not found.";
    object_position_height_threshold_ = -1.6;
  }

  return true;
}

bool LowObjectFilter::Filter(const ObjectFilterOptions& obj_filter_options,
                             std::vector<std::shared_ptr<Object>>* objects) {
  FilterLowObject(obj_filter_options, objects);

  return true;
}

void LowObjectFilter::FilterLowObject(
    const ObjectFilterOptions& obj_filter_options,
    std::vector<std::shared_ptr<Object>>* objects) {
  int object_number = objects->size();
  int valid_objects_num = 0;
  for (std::size_t i = 0; i < objects->size(); ++i) {
    std::shared_ptr<Object> obj = objects->at(i);
    float max_height = -100.0;
    float min_height = 100.0;
    for (std::size_t pi = 0; pi < obj->cloud->points.size(); ++pi) {
      auto pt = obj->cloud->points[pi];
      if (pt.z < min_height) {
        min_height = pt.z;
      } else if (pt.z > max_height) {
        max_height = pt.z;
      }
    }

    // object is low and flat
    if (max_height - min_height < 0.10 && max_height < -1.6) {
      continue;
    }

    if (static_cast<int>(i) != valid_objects_num) {
      (*objects)[valid_objects_num] = (*objects)[i];
    }

    valid_objects_num++;
  }
  objects->resize(valid_objects_num);
  AINFO << "low_object_filter: object number " << object_number << " -> "
        << valid_objects_num << "\n";
}

}  // namespace perception
}  // namespace apollo
