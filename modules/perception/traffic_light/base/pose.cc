// Copyright (c) 2016 Baidu.com, Inc. All Rights Reserved
// @author erlangz(erlangz@baidu.com)
// @date 2016/09/18 11:12:51
#include "modules/perception/traffic_light/base/pose.h"

namespace apollo {
namespace perception {
namespace traffic_light {

bool CarPose::init(const Eigen::Matrix4d &pose) {
  _pose = pose;
  return true;
}

const Eigen::Matrix4d CarPose::pose() const {
  return _pose;
}

std::ostream &operator<<(std::ostream &os, const CarPose &pose) {
  os << pose._pose;
  return os;
}

} // namespace traffic_light
} // namespace perception
} // namespace apollo
