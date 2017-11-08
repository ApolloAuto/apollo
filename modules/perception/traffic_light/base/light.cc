// Copyright (c) 2016 Baidu.com, Inc. All Rights Reserved
// @author erlangz(zhengwenchao@baidu.com)
// @date 2016/10/08 16:47:37
// @file ../traffic_light/base/light.cpp
// @brief 
// 
#include "module/perception/traffic_light/base/light.h"

#include <cmath>

#include <xlog.h>

namespace adu {
namespace perception {
namespace traffic_light {

std::ostream &operator<<(std::ostream &os, const Light &light) {
  os << "Light id:" << light.info.id().id()
     << " status:" << light.status.color;
  return os;
}

double stopline_distance(
    const Eigen::Matrix4d &car_pose,
    const ::google::protobuf::RepeatedPtrField<::adu::common::hdmap::Curve> &stoplines) {
  if (stoplines.size() == 0) {
    XLOG(WARN) << "compute car to stopline's distance failed(no stopline). "
               << "car_pose:" << car_pose;
    return -1;
  }
  const ::adu::common::hdmap::Curve &stopline = stoplines.Get(0);
  if (stopline.segment_size() == 0) {
    XLOG(WARN) << "compute car to stopline's distance failed(stopline has no segment line). "
               << "car_pose:" << car_pose << " stopline:" << stopline.ShortDebugString();
    return -1;
  }
  if (!stopline.segment(0).has_line_segment()) {
    XLOG(WARN) << "compute car to stopline's distance failed(stopline has no segment). "
               << "car_pose:" << car_pose << " stopline:" << stopline.ShortDebugString();
    return -1;
  }

  if (stopline.segment(0).line_segment().point_size() == 0) {
    XLOG(WARN) << "compute car to stopline's distance failed(stopline has no point). "
               << "car_pose:" << car_pose << " stopline:" << stopline.ShortDebugString();
    return -1;
  }

  double car_x = car_pose(0, 3);
  double car_y = car_pose(1, 3);
  double stopline_x = stopline.segment(0).line_segment().point(0).x();
  double stopline_y = stopline.segment(0).line_segment().point(0).y();

  return sqrt(std::pow(car_x - stopline_x, 2) + std::pow(car_y - stopline_y, 2));
}

//@brief compute traffic light to car's distance
double trafficlight_distance(
    const Eigen::Matrix4d &car_pose,
    const ::google::protobuf::RepeatedPtrField<::adu::common::hdmap::Subsignal> &subsignal) {
  if (subsignal.size() == 0) {
    XLOG(WARN) << "compute car to traffic_light's distance failed(no traffic_light). "
               << "car_pose:" << car_pose;
    return -1;
  }
  double car_x = car_pose(0, 3);
  double car_y = car_pose(1, 3);
  double traffic_light_x = subsignal.Get(0).location().x();
  double traffic_light_y = subsignal.Get(0).location().y();
  return sqrt(std::pow(car_x - traffic_light_x, 2) + std::pow(car_y - traffic_light_y, 2));
}

}  // namespace traffic_light
}  // namespace perception
}  // namespace adu
