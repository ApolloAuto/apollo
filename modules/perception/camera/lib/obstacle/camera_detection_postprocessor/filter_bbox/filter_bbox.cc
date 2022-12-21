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

#include "modules/perception/camera/lib/obstacle/camera_detection_postprocessor/filter_bbox/filter_bbox.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"

namespace apollo {
namespace perception {
namespace camera {

FilterBbox::FilterBbox(const PluginConfig& plugin_config) {
  Init(plugin_config);
}

bool FilterBbox::Init(const PluginConfig &plugin_config) {
  ACHECK(plugin_config.has_filter_bbox_config());
  min_dims_.min_2d_height = plugin_config.filter_bbox_config().min_2d_height();
  min_dims_.min_3d_height = plugin_config.filter_bbox_config().min_3d_height();
  min_dims_.min_3d_width = plugin_config.filter_bbox_config().min_3d_width();
  min_dims_.min_3d_length = plugin_config.filter_bbox_config().min_3d_length();
  return true;
}

bool FilterBbox::Process(DataFrame *data_frame) {
  if (nullptr == data_frame) {
    AERROR << "Input null dataframe ptr.";
    return false;
  }

  auto frame = data_frame->camera_frame;
  filter_bbox(min_dims_, &(frame->detected_objects));

  return true;
}

void FilterBbox::filter_bbox(const MinDims &min_dims,
                             std::vector<base::ObjectPtr> *objects) {
  int valid_obj_idx = 0;
  int total_obj_idx = 0;
  while (total_obj_idx < static_cast<int>(objects->size())) {
    const auto &obj = (*objects)[total_obj_idx];
    if ((obj->camera_supplement.box.ymax - obj->camera_supplement.box.ymin) >=
            min_dims.min_2d_height &&
        (min_dims.min_3d_height <= 0 ||
         obj->size[2] >= min_dims.min_3d_height) &&
        (min_dims.min_3d_width <= 0 || obj->size[1] >= min_dims.min_3d_width) &&
        (min_dims.min_3d_length <= 0 ||
         obj->size[0] >= min_dims.min_3d_length)) {
      (*objects)[valid_obj_idx] = (*objects)[total_obj_idx];
      ++valid_obj_idx;
    }
    ++total_obj_idx;
  }
  AINFO << valid_obj_idx << " of " << total_obj_idx << " obstacles kept";
  objects->resize(valid_obj_idx);
  AINFO << "Number of detected obstacles: " << objects->size();
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
