// Copyright (c) 2016 Baidu.com, Inc. All Rights Reserved
// @author erlangz(erlangz@baidu.com)
// @date 2016/09/18 11:12:51
#include "modules/perception/traffic_light/base/pose.h"

namespace adu {
namespace perception {
namespace traffic_light {

bool CarPose::init(const Eigen::Matrix4d &pose) {
  _pose = pose;
  return true;
}

const Eigen::Matrix4d CarPose::get_pose() const {
  return _pose;
}

const pcl_util::PointD CarPose::get_position() const {

  pcl_util::PointD p;
  p.x = _pose(0, 3);
  p.y = _pose(1, 3);
  p.z = _pose(2, 3);
  return p;
}

std::ostream &operator<<(std::ostream &os, const CarPose &pose) {
  os << pose._pose;
  return os;
}

} // namespace traffic_light
} // namespace perception
} // namespace adu
