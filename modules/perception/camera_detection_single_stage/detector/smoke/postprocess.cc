/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/camera_detection_single_stage/detector/smoke/postprocess.h"

#include "cyber/common/log.h"
#include "modules/perception/common/base/box.h"

namespace apollo {
namespace perception {
namespace camera {

void FilterByMinDims(const smoke::MinDims &min_dims,
                     std::vector<base::ObjectPtr> *objects) {
  std::size_t valid_index = 0, cur_index = 0;
  while (cur_index < objects->size()) {
    const auto& obj = objects->at(cur_index);
    float height =
        obj->camera_supplement.box.ymax - obj->camera_supplement.box.ymin;
    if (height >= min_dims.min_2d_height() &&
        obj->size[2] >= min_dims.min_3d_height() &&
        obj->size[1] >= min_dims.min_3d_width() &&
        obj->size[0] >= min_dims.min_3d_length()) {
      if (valid_index != cur_index)
        objects->at(valid_index) = objects->at(cur_index);
      ++valid_index;
    }
    ++cur_index;
  }

  objects->resize(valid_index);
  AINFO << valid_index << " of " << cur_index << " obstacles kept";
}

void RecoverBBox(int roi_w, int roi_h, int offset_y,
                 std::vector<base::ObjectPtr> *objects) {
  for (auto &obj : *objects) {
    float xmin = obj->camera_supplement.box.xmin;
    float ymin = obj->camera_supplement.box.ymin;
    float xmax = obj->camera_supplement.box.xmax;
    float ymax = obj->camera_supplement.box.ymax;

    float x = xmin * roi_w;
    float w = (xmax - xmin) * roi_w;
    float y = ymin * roi_h + offset_y;
    float h = (ymax - ymin) * roi_h;
    base::RectF rect_det(x, y, w, h);
    base::RectF rect_img(0, 0, roi_w, roi_h + offset_y);
    obj->camera_supplement.box = rect_det & rect_img;

    constexpr double eps = 1e-2;
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

void FillBBox(base::ObjectPtr obj, const float *bbox, int width, int height) {
  obj->camera_supplement.box.xmin = bbox[2] / width;
  obj->camera_supplement.box.ymin = bbox[3] / height;
  obj->camera_supplement.box.xmax = bbox[4] / width;
  obj->camera_supplement.box.ymax = bbox[5] / height;
}

void FillBBox3d(base::ObjectPtr obj, const float *bbox) {
  obj->camera_supplement.alpha = bbox[1];
  // todo: why obj size not put in camera_supplement
  obj->size[2] = bbox[6];
  obj->size[1] = bbox[7];
  obj->size[0] = bbox[8];

  obj->camera_supplement.local_center[0] = bbox[9];
  obj->camera_supplement.local_center[1] = bbox[10];
  obj->camera_supplement.local_center[2] = bbox[11];
}

base::ObjectSubType GetSubtype(
    int cls, const std::vector<base::ObjectSubType> &types) {
  if (cls < 0 || cls >= static_cast<int>(types.size())) {
    return base::ObjectSubType::UNKNOWN;
  }

  return types[cls];
}

void GetObjectsCpu(const std::shared_ptr<base::Blob<float>> &output_blob,
                   const std::vector<base::ObjectSubType> &types,
                   const smoke::ModelParam &model_param,
                   std::vector<base::ObjectPtr> *objects,
                   int width, int height) {
  const float* detect_result = output_blob->cpu_data();
  objects->clear();
  auto shape = output_blob->shape();

  int len_pred = shape[1];
  int total = shape[0] * shape[1];
  int step = 0;
  while (step < total) {
    const float* bbox = detect_result + step;
    step += len_pred;
    float score = bbox[13];
    if (score < model_param.confidence_threshold()) {
      continue;
    }

    base::ObjectPtr obj = std::make_shared<base::Object>();
    int label = static_cast<int>(bbox[0]);
    obj->sub_type = GetSubtype(label, types);
    obj->type = base::kSubType2TypeMap.at(obj->sub_type);
    obj->type_probs.assign(
        static_cast<int>(base::ObjectType::MAX_OBJECT_TYPE), 0);
    obj->sub_type_probs.assign(
        static_cast<int>(base::ObjectSubType::MAX_OBJECT_TYPE), 0);
    obj->type_probs[static_cast<int>(obj->type)] = score;
    obj->sub_type_probs[static_cast<int>(obj->sub_type)] = score;
    obj->confidence = score;

    FillBBox(obj, bbox, width, height);
    FillBBox3d(obj, bbox);
    objects->push_back(obj);
  }
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
