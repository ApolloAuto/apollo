/******************************************************************************
 * Copyright 2020 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/perception/camera/lib/obstacle/detector/smoke/region_output.h"

#include "cyber/common/log.h"
#include "modules/perception/camera/lib/obstacle/detector/smoke/object_maintainer.h"

namespace apollo {
namespace perception {
namespace camera {

void filter_bbox(const SmokeMinDims &min_dims,
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

void recover_smoke_bbox(int roi_w, int roi_h, int offset_y,
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

void fill_smoke_base(base::ObjectPtr obj, const float *bbox,
                     int width, int height) {
  obj->camera_supplement.box.xmin = bbox[0]/width;
  obj->camera_supplement.box.ymin = bbox[1]/height;
  obj->camera_supplement.box.xmax = bbox[2]/width;
  obj->camera_supplement.box.ymax = bbox[3]/height;
}

void fill_smoke_bbox3d(bool with_box3d, base::ObjectPtr obj,
                       const float *bbox) {
  if (with_box3d) {
    obj->camera_supplement.alpha = bbox[1];
    obj->size[2] = bbox[6];
    obj->size[1] = bbox[7];
    obj->size[0] = bbox[8];

    obj->camera_supplement.local_center[0] = bbox[9];
    obj->camera_supplement.local_center[1] = bbox[10];
    obj->camera_supplement.local_center[2] = bbox[11];
  }
}

base::ObjectSubType get_smoke_object_subtype(int cls) {
  if (cls == 0) {
    return base::ObjectSubType::CAR;
  } else if (cls == 1) {
    return base::ObjectSubType::CYCLIST;
  } else if (cls == 2) {
    return base::ObjectSubType::PEDESTRIAN;
  } else {
    return base::ObjectSubType::UNKNOWN;
  }
}

void get_smoke_objects_cpu(const SmokeBlobs &smoke_blobs,
                     const std::vector<base::ObjectSubType> &types,
                     const smoke::ModelParam &model_param,
                     float light_vis_conf_threshold,
                     float light_swt_conf_threshold,
                     base::Blob<bool> *overlapped, base::Blob<int> *idx_sm,
                     std::vector<base::ObjectPtr> *objects,
                     int width, int height) {
  const float* detect_result = smoke_blobs.det1_loc_blob->cpu_data();
  objects->clear();

  int len_pred = 14;
  for (int i = 0; i < 50; i++) {
    const float* bbox = detect_result + i * len_pred;
    float score = bbox[13];
    if (score < model_param.confidence_threshold()) {
      continue;
    }

    float label = bbox[0];
    base::ObjectPtr obj = nullptr;
    obj.reset(new base::Object);
    obj->sub_type = get_smoke_object_subtype(label);
    obj->type = base::kSubType2TypeMap.at(obj->sub_type);
    obj->type_probs.assign(
        static_cast<int>(base::ObjectType::MAX_OBJECT_TYPE), 0);
    obj->sub_type_probs.assign(
        static_cast<int>(base::ObjectSubType::MAX_OBJECT_TYPE), 0);
    obj->type_probs[static_cast<int>(obj->type)] = score;
    obj->sub_type_probs[static_cast<int>(obj->sub_type)] = score;
    obj->confidence = score;

    fill_smoke_base(obj, bbox + 2, width, height);
    fill_smoke_bbox3d(model_param.with_box3d(), obj, bbox);

    objects->push_back(obj);
  }
}
}  // namespace camera
}  // namespace perception
}  // namespace apollo
