/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/obstacle/camera/detector/yolo_camera_detector/util.h"

#include <fstream>
#include <iostream>
#include <map>

#include "opencv2/opencv.hpp"

#include "modules/common/log.h"

namespace apollo {
namespace perception {
namespace yolo {

using std::string;
using std::vector;
using std::map;
using std::ifstream;

bool load_types(const string &path, vector<ObjectType> *types) {
  const map<string, ObjectType> type_map = {
      {"UNKNOWN", apollo::perception::ObjectType::UNKNOWN},
      {"UNKNOWN_MOVABLE", apollo::perception::ObjectType::UNKNOWN_MOVABLE},
      {"UNKNOWN_UNMOVABLE", apollo::perception::ObjectType::UNKNOWN_UNMOVABLE},
      {"PEDESTRIAN", apollo::perception::ObjectType::PEDESTRIAN},
      {"BICYCLE", apollo::perception::ObjectType::BICYCLE},
      {"VEHICLE", apollo::perception::ObjectType::VEHICLE},
  };

  ifstream ifs(path, ifstream::in);
  if (!ifs.good()) {
    AERROR << "type_list not found, use default: VEHICLE, BICYCLE, PEDESTRIAN";
    (*types) = {apollo::perception::ObjectType::VEHICLE,
                apollo::perception::ObjectType::BICYCLE,
                apollo::perception::ObjectType::PEDESTRIAN};
  } else {
    string type;
    AINFO << "Supported types: ";
    while (ifs >> type) {
      if (type_map.find(type) == type_map.end()) {
        AERROR << "Invalid type: " << type;
        return false;
      }

      (*types).push_back(type_map.at(type));
      AINFO << "\t\t" << type;
    }
    AINFO << "\t\t" << (*types).size() << " in total.";
    ifs.close();
  }
  return true;
}

bool load_anchors(const string &path, vector<float> *anchors) {
  if (anchors == NULL) {
    AERROR << "input anchors is a null pointer.";
    return false;
  }

  int num_anchors = 0;
  ifstream ifs(path, ifstream::in);
  ifs >> num_anchors;
  if (!ifs.good()) {
    AERROR << "Failed to get number of anchors!";
    return false;
  }
  anchors->resize(num_anchors * 2);
  for (int i = 0; i < num_anchors; ++i) {
    ifs >> anchors->at(i * 2) >> anchors->at(i * 2 + 1);
    if (!ifs.good()) {
      AERROR << "Failed to load the " << i << "-th anchor!";
      return false;
    }
  }
  ifs.close();

  return true;
}

void recover_bbox(int roi_w, int roi_h, int offset_y,
                  std::vector<std::shared_ptr<VisualObject>> *objects) {
  for (auto &obj : *objects) {
    float xmin = obj->upper_left[0];
    float ymin = obj->upper_left[1];
    float xmax = obj->lower_right[0];
    float ymax = obj->lower_right[1];
    int x = xmin * roi_w;
    int w = (xmax - xmin) * roi_w;
    int y = ymin * roi_h + offset_y;
    int h = (ymax - ymin) * roi_h;
    cv::Rect rect_det(x, y, w, h);
    cv::Rect rect_img(0, 0, roi_w, roi_h + offset_y);
    cv::Rect rect = rect_det & rect_img;
    obj->upper_left[0] = rect.x;
    obj->upper_left[1] = rect.y;
    obj->lower_right[0] = rect.x + rect.width;
    obj->lower_right[1] = rect.y + rect.height;
    double eps = 1e-2;

    // Truncation assignment based on bbox positions
    if ((ymin < eps) || (ymax >= 1.0 - eps)) {
      obj->trunc_height = 0.5;
    } else {
      obj->trunc_height = 0.0;
    }
    if ((xmin < eps) || (xmax >= 1.0 - eps)) {
      obj->trunc_width = 0.5;
    } else {
      obj->trunc_width = 0.0;
    }
  }
}

}  // namespace yolo
}  // namespace perception
}  // namespace apollo
