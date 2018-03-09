/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#ifndef MODULES_PERCEPTION_TRAFFIC_LIGHT_BASE_LIGHT_H_
#define MODULES_PERCEPTION_TRAFFIC_LIGHT_BASE_LIGHT_H_

#include <memory>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "opencv2/opencv.hpp"

#include "modules/map/proto/map_signal.pb.h"
#include "modules/perception/proto/traffic_light_detection.pb.h"

#include "modules/perception/traffic_light/base/image.h"

namespace apollo {
namespace perception {
namespace traffic_light {

typedef apollo::perception::TrafficLight::Color TLColor;
const TLColor UNKNOWN_COLOR = TrafficLight::UNKNOWN;
const TLColor GREEN = TrafficLight::GREEN;
const TLColor RED = TrafficLight::RED;
const TLColor YELLOW = TrafficLight::YELLOW;
const TLColor BLACK = TrafficLight::BLACK;
// When the light has been covered by some objected, the color returned.
const TLColor DEFAULT_UNKNOWN_COLOR = TrafficLight::UNKNOWN;

enum DetectionClassId {
  UNKNOWN_CLASS = -1,
  VERTICAL_CLASS = 0,
  QUADRATE_CLASS = 1,
  HORIZONTAL_CLASS = 2
};

/**
 * @class LightRegion
 * @brief Light Region in the Image
 */
struct LightRegion {
  // roi is marked by map & projection, it may be too large or not accuracy.
  cv::Rect projection_roi;

  std::vector<cv::Rect> debug_roi;
  std::vector<float> debug_roi_detect_scores;

  // rectified_roi is the region marked by Rectifier, it should be accuracy
  cv::Rect rectified_roi;
  bool is_detected = false;
  bool is_selected = false;
  // detection 输出结果，实际取值 -1、0 或 1
  // 为 0 则 UnityRecognize 中使用白天模型
  // 为 1 则使用夜晚模型
  DetectionClassId detect_class_id = UNKNOWN_CLASS;

  // output score by detection
  float detect_score = 0.0f;

  std::string to_string() const {
    std::ostringstream oss;
    oss << "LightRegion: [projection_roi:<(" << projection_roi.tl().x << ","
        << projection_roi.tl().y << "),(" << projection_roi.br().x << ","
        << projection_roi.br().y << "] ";
    return oss.str();
  }
};

/**
 * @class LightStatus
 * @brief Light Status
 */
struct LightStatus {
  // Traffic light color status.
  TLColor color = UNKNOWN_COLOR;
  // How confidence about the detected results, between 0 and 1.
  double confidence = 0.0;

  std::string to_string() const {
    std::string light_color =
        (color == UNKNOWN_COLOR
             ? "unknown color"
             : (color == RED ? "red"
                             : (color == GREEN
                                    ? "green"
                                    : (color == YELLOW ? "yellow" : "black"))));
    // std::string light_color;
    std::ostringstream oss;
    oss << "Status: [color:" << light_color << " confidence:" << confidence
        << "]";
    return oss.str();
  }
};

/**
 * @class Light
 * @brief A Traffic Light
 */
struct Light {
  Light() = default;

  explicit Light(const apollo::hdmap::Signal &signal) : info(signal) {}
  apollo::hdmap::Signal info;  //  Light info in the map.
  LightRegion region;          //  Light region on the image.
  LightStatus status;          //  Light Status.

  std::string to_string() const {
    std::ostringstream oss;
    oss << "Light: {" << status.to_string() << region.to_string()
        << "Signal Info: [" << info.ShortDebugString() << "]}";
    return oss.str();
  }
};

std::ostream &operator<<(std::ostream &os, const Light &light);

typedef std::shared_ptr<Light> LightPtr;
typedef std::vector<LightPtr> LightPtrs;

/**
 * @brief compute stopline to car's distance
 * @param car pose
 * @param stoplines
 * @return distance
 */
double Distance2Stopline(
    const Eigen::Matrix4d &car_pose,
    const google::protobuf::RepeatedPtrField<apollo::hdmap::Curve> &stoplines);

}  //  namespace traffic_light
}  //  namespace perception
}  //  namespace apollo

#endif  //  MODULES_PERCEPTION_TRAFFIC_LIGHT_BASE_LIGHT_H_
