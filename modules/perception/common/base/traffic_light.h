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
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "modules/perception/common/base/box.h"

namespace apollo {
namespace perception {
namespace base {

enum class TLColor {
  TL_UNKNOWN_COLOR = 0,
  TL_RED = 1,
  TL_YELLOW = 2,
  TL_GREEN = 3,
  TL_BLACK = 4,
  TL_TOTAL_COLOR_NUM = 5
};

enum class TLDetectionClass {
  TL_UNKNOWN_CLASS = -1,
  TL_VERTICAL_CLASS = 0,
  TL_QUADRATE_CLASS = 1,
  TL_HORIZONTAL_CLASS = 2
};

// @brief Light Region in the Image
struct LightRegion {
  // roi is marked by map & projection, it may be too large or not accuracy.
  Rect<int> projection_roi;
  Rect<int> crop_roi;
  bool outside_image = false;

  std::vector<Rect<int>> debug_roi;
  std::vector<float> debug_roi_detect_scores;

  Rect<int> detection_roi;
  bool is_detected = false;
  bool is_selected = false;
  TLDetectionClass detect_class_id = TLDetectionClass::TL_UNKNOWN_CLASS;

  // 3d polygon
  std::vector<base::PointXYZID> points;

  // output score by detection
  float detect_score = 0.0f;
};

// @brief Light Status
struct LightStatus {
  // Traffic light color status.
  TLColor color = TLColor::TL_UNKNOWN_COLOR;
  // How confidence about the detected results, between 0 and 1.
  double confidence = 0.0;
  // Duration of the traffic light since detected.
  double tracking_time = 0.0;
  // blink status
  bool blink = false;
};

// @brief A Traffic Light.
struct TrafficLight {
  TrafficLight() = default;

  std::string id;
  int semantic = 0;
  LightRegion region;  // Light region.
  LightStatus status;  // Light Status.
};

typedef std::shared_ptr<TrafficLight> TrafficLightPtr;
typedef std::vector<TrafficLightPtr> TrafficLightPtrs;

}  // namespace base
}  // namespace perception
}  // namespace apollo
