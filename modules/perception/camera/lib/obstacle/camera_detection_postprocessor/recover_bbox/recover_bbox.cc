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

#include "modules/perception/camera/lib/obstacle/camera_detection_postprocessor/recover_bbox/recover_bbox.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"

namespace apollo {
namespace perception {
namespace camera {

RecoverBbox::RecoverBbox(const PluginConfig& plugin_config) {
  Init(plugin_config);
}

bool RecoverBbox::Init(const PluginConfig &plugin_config) {
  ACHECK(plugin_config.has_recover_bbox_config());

  roi_w_ = plugin_config.recover_bbox_config().roi_w();
  roi_h_ = plugin_config.recover_bbox_config().roi_h();
  offset_y_ = plugin_config.recover_bbox_config().offset_y();
  return true;
}

bool RecoverBbox::Process(DataFrame *data_frame) {
  if (nullptr == data_frame) {
    AERROR << "Input null dataframe ptr.";
    return false;
  }

  auto frame = data_frame->camera_frame;
  recover_bbox(roi_w_, roi_h_, offset_y_, &(frame->detected_objects));

  return true;
}


void RecoverBbox::recover_bbox(int roi_w, int roi_h, int offset_y,
                  std::vector<base::ObjectPtr> *objects) {
  for (auto &obj : *objects) {
    float xmin = obj->camera_supplement.box.xmin;
    float ymin = obj->camera_supplement.box.ymin;
    float xmax = obj->camera_supplement.box.xmax;
    float ymax = obj->camera_supplement.box.ymax;
    float x = xmin * static_cast<float>(roi_w);
    float w = (xmax - xmin) * static_cast<float>(roi_w);
    float y = ymin * static_cast<float>(roi_h) + static_cast<float>(offset_y);
    float h = (ymax - ymin) * static_cast<float>(roi_h);
    base::RectF rect_det(x, y, w, h);
    base::RectF rect_img(0, 0, static_cast<float>(roi_w),
                         static_cast<float>(roi_h + offset_y));
    base::RectF rect = rect_det & rect_img;
    obj->camera_supplement.box = rect;

    double eps = 1e-2;

    // Truncation assignment based on bbox positions
    if ((ymin < eps) || (ymax >= (1.0 - eps))) {
      obj->camera_supplement.truncated_vertical = 0.5;
    } else {
      obj->camera_supplement.truncated_vertical = 0.0;
    }
    if ((xmin < eps) || (xmax >= (1.0 - eps))) {
      obj->camera_supplement.truncated_horizontal = 0.5;
    } else {
      obj->camera_supplement.truncated_horizontal = 0.0;
    }
  }
}


}  // namespace camera
}  // namespace perception
}  // namespace apollo
