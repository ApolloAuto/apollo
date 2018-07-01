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

#include "modules/common/util/file.h"
#include "modules/perception/common/pcl_types.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/obstacle/base/types.h"

namespace apollo {
namespace perception {

using pcl_util::PointCloud;
using pcl_util::PointCloudPtr;
using apollo::common::util::GetProtoFromFile;

bool LowObjectFilter::Init() {
  if (!GetProtoFromFile(FLAGS_low_object_filter_config, &config_)) {
    AERROR << "Cannot get config proto from file: "
           << FLAGS_low_object_filter_config;
    return false;
  }
  object_height_threshold_ = config_.object_height_threshold();
  object_position_height_threshold_ =
                    config_.object_position_height_threshold();
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
    if (max_height - min_height < object_height_threshold_
        && max_height < object_position_height_threshold_) {
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
