// Copyright 2016 Baidu Inc. All Rights Reserved.
// @author: Hengyu Li (lihengyu@baidu.com)
// @file: light.h
// @brief: traffic light basic data struction definition.

#ifndef ADU_PERCEPTION_TRAFFIC_LIGHT_BASE_LIGHT_H
#define ADU_PERCEPTION_TRAFFIC_LIGHT_BASE_LIGHT_H

#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <map_signal.pb.h>
#include <traffic_light_detection.pb.h>

#include "modules/perception/traffic_light/base/image.h"

namespace adu {
namespace perception {
namespace traffic_light {

typedef adu::common::traffic_light::TrafficLight::Color TLColor;
const TLColor UNKNOWN_COLOR = adu::common::traffic_light::TrafficLight::UNKNOWN;
const TLColor GREEN = adu::common::traffic_light::TrafficLight::GREEN;
const TLColor RED = adu::common::traffic_light::TrafficLight::RED;
const TLColor YELLOW = adu::common::traffic_light::TrafficLight::YELLOW;
const TLColor BLACK = adu::common::traffic_light::TrafficLight::BLACK;
//When the light has been covered by some objected, the color returned.
const TLColor DEFAULT_UNKNOWN_COLOR = adu::common::traffic_light::TrafficLight::UNKNOWN;

// enum DetectionClassId {
//     UNKNOWN_CLASS = -1,
//     DAY_CLASS = 0,
//     NIGHT_CLASS = 1
// };
enum DetectionClassId {
  UNKNOWN_CLASS = -1,
  VERTICAL_CLASS = 0,
  QUADRATE_CLASS = 1,
  HORIZONTAL_CLASS = 2
};

//@brief Light Region in the Image
struct LightRegion {
  //roi is marked by map & projection, it may be too large or not accuracy.
  cv::Rect projection_roi;
  int projection_radius;

  std::vector<cv::Rect> debug_roi;
  std::vector<float> debug_roi_detect_scores;

  //rectified_roi is the region marked by Rectifier, it should be accuracy and small.
  //A Light can have more than one roi, Rectifier may found more than one region seems like TL.
  //Each roi can has many candidates, Recognizer can votes for them.
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

//@brief Light Status
struct LightStatus {
  // Traffic light color status.
  TLColor color = UNKNOWN_COLOR;
  // How confidence about the detected results, between 0 and 1.
  double confidence = 0.0;
  // Duration of the traffic light since detected.
  double tracking_time = 0.0;

  std::string to_string() const {
    std::string light_color = (color == UNKNOWN_COLOR ? "unknown color" :
                               (color == RED ? "red" :
                                (color == GREEN ? "green" :
                                 (color == YELLOW ? "yellow" : "black"))));
    //std::string light_color;
    std::ostringstream oss;
    oss << "Status: [color:" << light_color << " confidence:" << confidence
        << " tracking_time:" << tracking_time << "]";
    return oss.str();
  }
};

//@brief A Traffic Light.
struct Light {
  Light() = default;

  explicit Light(const adu::common::hdmap::Signal &signal) :
      info(signal) {
  }
  adu::common::hdmap::Signal info;    // Light info in the map.
  LightRegion region;  // Light region on the image.
  LightStatus status;  // Light Status.

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

//@brief compute stopline to car's distance
double stopline_distance(
    const Eigen::Matrix4d &car_pose,
    const ::google::protobuf::RepeatedPtrField<::adu::common::hdmap::Curve> &stoplines);

//@brief compute traffic light to car's distance
double trafficlight_distance(
    const Eigen::Matrix4d &car_pose,
    const ::google::protobuf::RepeatedPtrField<::adu::common::hdmap::Subsignal> &subsignal);

}  // namespace traffic_light
}  // namespace perception
}  // namespace adu

#endif  // ADU_PERCEPTION_TRAFFIC_LIGHT_BASE_LIGHT_H