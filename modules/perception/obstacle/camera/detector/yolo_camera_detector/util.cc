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

#include "modules/obstacle/camera/detector/yolo_camera_detector/util.h"

#include <fstream>
#include <iostream>
#include <map>
#include "modules/common/log.h

namespace apollo {
namespace perception {
namespace obstacle {
namespace yolo {

bool load_types(const std::string &path, std::vector<ObjectType> *types) {
  const std::map<std::string, ObjectType> type_map = {
      {"UNKNOWN", UNKNOWN},
      {"UNKNOWN_MOVABLE", UNKNOWN_MOVABLE},
      {"UNKNOWN_UNMOVABLE", UNKNOWN_UNMOVABLE},
      {"PEDESTRIAN", PEDESTRIAN},
      {"BICYCLE", BICYCLE},
      {"VEHICLE", VEHICLE},
  };

  std::ifstream ifs(path, std::ifstream::in);
  if (!ifs.good()) {
    AERROR << "type_list not found, use default: VEHICLE, BICYCLE, PEDESTRIAN";
    (*types) = {VEHICLE, BICYCLE, PEDESTRIAN};
  } else {
    std::string type;
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

bool load_anchors(const std::string &path, std::vector<float> *anchors) {
  int num_anchors = 0;
  std::ifstream ifs(path, std::ifstream::in);
  ifs >> num_anchors;
  if (!ifs.good()) {
    AERROR << "Failed to get number of anchors!";
    return false;
  }
  (*anchors).resize(num_anchors * 2);
  for (int i = 0; i < num_anchors; ++i) {
    ifs >> (*anchors)[i * 2] >> (*anchors)[i * 2 + 1];
    if (!ifs.good()) {
      AERROR << "Failed to load the " << i << "-th anchor!";
      return false;
    }
  }
  ifs.close();
  return true;
}

}  // namespace yolo
}  // namespace obstacle
}  // namespace perception
}  // namespace apollo
