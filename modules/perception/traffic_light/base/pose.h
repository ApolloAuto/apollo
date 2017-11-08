// Copyright (c) 2016 Baidu.com, Inc. All Rights Reserved
// @author zhengwenchao(zhengwenchao@baidu.com)
// @file: pose.h
// @brief: the data-structure for Car's pose(translation & rotation).
//
#ifndef ADU_PERCEPTION_TRAFFIC_LIGHT_BASE_POSE_H
#define ADU_PERCEPTION_TRAFFIC_LIGHT_BASE_POSE_H

#include <eigen3/Eigen/Core>
#include "lib/pcl_util/pcl_types.h"

namespace adu {
namespace perception {
namespace traffic_light {

//@brief Car's Pose
class CarPose {
 public:
  CarPose() = default;

  virtual ~CarPose() = default;

  bool init(const Eigen::Matrix4d &pose);

  const Eigen::Matrix4d get_pose() const;

  const pcl_util::PointD get_position() const;

 private:
  Eigen::Matrix4d _pose;

  friend std::ostream &operator<<(std::ostream &os, const CarPose &);
};

std::ostream &operator<<(std::ostream &os, const CarPose &pose);

} // namespace traffic_light
} // namespace perception
} // namespace adu

#endif  // ADU_PERCEPTION_TRAFFIC_LIGHT_BASE_POSE_H
// @date 2016/09/08 17:48:06
