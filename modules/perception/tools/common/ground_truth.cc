/******************************************************************************
 * Copyright 2022 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/tools/common/ground_truth.h"

#include <algorithm>
#include <cstring>
#include <fstream>
#include <iostream>
#include <map>

#include "cyber/common/log.h"

namespace apollo {
namespace perception {
namespace camera {

const std::map<std::string, base::ObjectSubType> object_subtype_map = {
  {"car", base::ObjectSubType::CAR},
  {"van", base::ObjectSubType::VAN},
  {"bus", base::ObjectSubType::BUS},
  {"truck", base::ObjectSubType::TRUCK},
  {"cyclist", base::ObjectSubType::CYCLIST},
  {"motorcyclist", base::ObjectSubType::MOTORCYCLIST},
  {"tricyclelist", base::ObjectSubType::TRICYCLIST},
  {"pedestrian", base::ObjectSubType::PEDESTRIAN},
  {"trafficcone", base::ObjectSubType::TRAFFICCONE},
};

bool LoadKittiLabel(onboard::CameraFrame* frame, const std::string& kitti_path,
    const std::string& dist_type) {
  frame->detected_objects.clear();
  FILE *fp = fopen(kitti_path.c_str(), "r");
  if (fp == nullptr) {
    AERROR << "Failed to load object file: " << kitti_path;
    return false;
  }

  while (!feof(fp)) {
    base::ObjectPtr obj = nullptr;
    obj.reset(new base::Object);
    float trash = 0.0f;
    float score = 0.0f;
    char type[255];
    float x1 = 0.0f;
    float y1 = 0.0f;
    float x2 = 0.0f;
    float y2 = 0.0f;
    memset(type, 0, sizeof(type));

    int ret = 0;
    ret = fscanf(fp, "%254s %f %f %lf %f %f %f %f %f %f %f %lf %lf %lf %f %f",
                 type, &trash, &trash, &obj->camera_supplement.alpha, &x1, &y1,
                 &x2, &y2, &obj->size[2], &obj->size[1], &obj->size[0],
                 &obj->center[0], &obj->center[1], &obj->center[2], &obj->theta,
                 &score);
    AINFO << "fscanf return: " << ret;
    if (dist_type == "H-from-h") {
      obj->size[0] = static_cast<float>(obj->center[2]);
    } else if (dist_type == "H-on-h") {
      obj->size[0] = static_cast<float>(obj->center[2]) * (y2 - y1);
    } else {
      AERROR << "Not supported dist type! " << dist_type;
      return false;
    }
    obj->camera_supplement.box.xmin = std::max<float>(x1, 0);
    obj->camera_supplement.box.ymin = std::max<float>(y1, 0);
    obj->camera_supplement.box.xmax =
        std::min<float>(
            x2, static_cast<float>(frame->data_provider->src_width()));
    obj->camera_supplement.box.ymax =
        std::min<float>(
            y2, static_cast<float>(frame->data_provider->src_height()));
    obj->camera_supplement.area_id = 5;

    if (object_subtype_map.find(type) == object_subtype_map.end()) {
      obj->sub_type = base::ObjectSubType::UNKNOWN;
    } else {
      obj->sub_type = object_subtype_map.at(type);
    }
    obj->type = base::kSubType2TypeMap.at(obj->sub_type);
    obj->type_probs.assign(
        static_cast<int>(base::ObjectType::MAX_OBJECT_TYPE), 0);
    obj->sub_type_probs.assign(
        static_cast<int>(base::ObjectSubType::MAX_OBJECT_TYPE), 0);
    obj->sub_type_probs[static_cast<int>(obj->sub_type)] = score;
    obj->type_probs[static_cast<int>(obj->type)] = score;

    frame->detected_objects.push_back(obj);
  }

  fclose(fp);
  return true;
}


}  // namespace camera
}  // namespace perception
}  // namespace apollo
