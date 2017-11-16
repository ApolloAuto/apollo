// Copyright (c) 2016 Baidu.com, Inc. All Rights Reserved
// @author zhengwenchao(zhengwenchao@baidu.com)
// @file: pose.h
// @brief: the data-structure for Car's pose(translation & rotation).
//
#ifndef MODULES_PERCEPTION_TRAFFIC_LIGHT_BASE_POSE_H
#define MODULES_PERCEPTION_TRAFFIC_LIGHT_BASE_POSE_H

#include <eigen3/Eigen/Core>

namespace apollo {
namespace perception {
namespace traffic_light {

//@brief Car's Pose
class CarPose {
 public:
  CarPose() = default;

  virtual ~CarPose() = default;

  bool set_pose(const Eigen::Matrix4d &pose);

  const Eigen::Matrix4d pose() const;

 private:
  Eigen::Matrix4d _pose;

  friend std::ostream &operator<<(std::ostream &os, const CarPose &);
};

std::ostream &operator<<(std::ostream &os, const CarPose &pose);

} // namespace traffic_light
} // namespace perception
} // namespace apollo

#endif  // MODULES_PERCEPTION_TRAFFIC_LIGHT_BASE_POSE_H
// @date 2016/09/08 17:48:06
