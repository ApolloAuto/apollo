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
#include "modules/perception/camera_detection_bev/detector/petr/postprocess.h"

#include <memory>

#include "cyber/common/log.h"

namespace apollo {
namespace perception {
namespace camera {

base::ObjectSubType GetSubtype(int cls,
                               const std::vector<base::ObjectSubType> &types) {
  if (cls < 0 || cls >= static_cast<int>(types.size())) {
    return base::ObjectSubType::UNKNOWN;
  }

  return types[cls];
}

void GetObjects(const base::BlobPtr<float> &box3ds,
                const base::BlobPtr<float> &labels,
                const base::BlobPtr<float> &scores,
                const std::vector<base::ObjectSubType> &types,
                float score_threshold, std::vector<base::ObjectPtr> *objects) {
  const auto bbox_ptr = box3ds->cpu_data();
  const auto label_ptr = labels->cpu_data();
  const auto score_ptr = scores->cpu_data();

  int feature_size = box3ds->shape(1);
  objects->clear();
  for (int i = 0; i < scores->count(); ++i) {
    float score = score_ptr[i];
    if (score < score_threshold) {
      continue;
    }

    base::ObjectPtr obj;
    obj.reset(new base::Object());

    obj->sub_type = GetSubtype(label_ptr[i], types);
    obj->type = base::kSubType2TypeMap.at(obj->sub_type);
    obj->type_probs.assign(static_cast<int>(base::ObjectType::MAX_OBJECT_TYPE),
                           0);
    obj->sub_type_probs.assign(
        static_cast<int>(base::ObjectSubType::MAX_OBJECT_TYPE), 0);
    obj->type_probs[static_cast<int>(obj->type)] = score;
    obj->sub_type_probs[static_cast<int>(obj->sub_type)] = score;
    obj->confidence = score;

    FillBBox3d(bbox_ptr + i * feature_size, obj);

    objects->push_back(obj);
  }
}

void FillBBox3d(const float *bbox, base::ObjectPtr obj) {
  obj->camera_supplement.local_center[0] = bbox[0];
  obj->camera_supplement.local_center[1] = bbox[1];
  obj->camera_supplement.local_center[2] = bbox[2];
  // size: length, width, height of bbox
  obj->size[0] = bbox[4];
  obj->size[1] = bbox[3];
  obj->size[2] = bbox[5];

  // yaw
  obj->theta = bbox[6];

  obj->center(0) = static_cast<double>(obj->camera_supplement.local_center[0]);
  obj->center(1) = static_cast<double>(obj->camera_supplement.local_center[1]);
  obj->center(2) = static_cast<double>(obj->camera_supplement.local_center[2]);
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
