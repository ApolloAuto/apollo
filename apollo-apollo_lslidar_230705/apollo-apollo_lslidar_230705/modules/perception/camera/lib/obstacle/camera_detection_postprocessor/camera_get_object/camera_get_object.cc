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

#include "modules/perception/camera/lib/obstacle/camera_detection_postprocessor/camera_get_object/camera_get_object.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"

namespace apollo {
namespace perception {
namespace camera {

CameraGetObject::CameraGetObject(const PluginConfig &plugin_config) {
  Init(plugin_config);
}

bool CameraGetObject::Init(const PluginConfig &plugin_config) {
  // ACHECK(plugin_config.CameraGetObject());
  // confidence_threshold_ = plugin_config.confidence_threshold();
  return true;
}

// input  : DataFrame *data_frame
// output : DataFrame * data_frame
bool CameraGetObject::Process(const std::vector<float> &detect_result,
                              DataFrame *data_frame) {
  if (nullptr == data_frame) {
    AERROR << "Input null dataframe ptr.";
    return false;
  }

  auto frame = data_frame->camera_frame;
  get_smoke_objects_cpu(
      detect_result, confidence_threshold_, frame->data_provider->src_width(),
      frame->data_provider->src_height(), &(frame->detected_objects));
  return true;
}

void CameraGetObject::get_smoke_objects_cpu(
    const std::vector<float> &detect_result, float confidence_threshold,
    int width, int height, std::vector<base::ObjectPtr> *objects) {
  objects->clear();

  // todo(zero): need fix model_param
  // int len_pred = 14;
  // for (int i = 0; i < 50; i++) {
  //   const float *bbox = detect_result.data() + i * len_pred;
  //   float score = bbox[13];
  //   if (score < model_param.confidence_threshold()) {
  //     continue;
  //   }

  //   float label = bbox[0];
  //   base::ObjectPtr obj = nullptr;
  //   obj.reset(new base::Object);
  //   obj->sub_type = get_smoke_object_subtype(label);
  //   obj->type = base::kSubType2TypeMap.at(obj->sub_type);
  //   obj->type_probs.assign(static_cast<int>(base::ObjectType::MAX_OBJECT_TYPE),
  //                          0);
  //   obj->sub_type_probs.assign(
  //       static_cast<int>(base::ObjectSubType::MAX_OBJECT_TYPE), 0);
  //   obj->type_probs[static_cast<int>(obj->type)] = score;
  //   obj->sub_type_probs[static_cast<int>(obj->sub_type)] = score;
  //   obj->confidence = score;

  //   fill_smoke_base(obj, bbox + 2, width, height);
  //   fill_smoke_bbox3d(model_param.with_box3d(), obj, bbox);

  //   objects->push_back(obj);
  // }
}

void CameraGetObject::fill_smoke_base(base::ObjectPtr obj, const float *bbox,
                                      int width, int height) {
  obj->camera_supplement.box.xmin = bbox[0] / width;
  obj->camera_supplement.box.ymin = bbox[1] / height;
  obj->camera_supplement.box.xmax = bbox[2] / width;
  obj->camera_supplement.box.ymax = bbox[3] / height;
}

void CameraGetObject::fill_smoke_bbox3d(bool with_box3d, base::ObjectPtr obj,
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

}  // namespace camera
}  // namespace perception
}  // namespace apollo
