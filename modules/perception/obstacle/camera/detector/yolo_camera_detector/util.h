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

#define MODULES_PERCEPTION_OBSTACLE_CAMERA_DETECTOR_YOLO_CAMERA_DETECTOR_UTIL_H_

#include <vector>
#include "modules/obstacle/base/types.h"

namespace apollo {
namespace perception {
namespace obstacle {
namespace yolo {

bool load_types(const std::string &path, std::vector<ObjectType> *types);
bool load_anchors(const std::string &path, std::vector<float> *anchors);

}  // namespace yolo
}  // namespace obstacle
}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_CAMERA_DETECTOR_YOLO_CAMERA_DETECTOR_UTIL_H_
