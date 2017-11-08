// Copyright (c) 2016 Baidu.com, Inc. All Rights Reserved
// @author erlangz(zhengwenchao@baidu.com)
// @date 2016/09/26 17:48:42
// @file projection.h
// @brief Project a traffic_light onto image.
//        We were told traffic_light's location & we have known our location,
//        and then we mark the traffic_light region on the image.
#ifndef ADU_PERCEPTION_TRAFFIC_LIGHT_PROJECTION_BASE_LIGHTS_PROJECTION_H
#define ADU_PERCEPTION_TRAFFIC_LIGHT_PROJECTION_BASE_LIGHTS_PROJECTION_H

#include <cmath>
#include <eigen3/Eigen/Core>

#include "modules/perception/traffic_light/interface/base_projection.h"

namespace apollo {
namespace perception {
namespace traffic_light {

//@brief Projection for each Camera.
class SingleBoundaryBasedProjection : public BaseProjection {
 public:
  virtual bool project(const CameraCoeffient &camera_coeffient,
                       const Eigen::Matrix4d &pose,
                       const apollo::hdmap::Signal &tl_info,
                       Light *light) const override;

 private:
  bool project_point(const CameraCoeffient &coeffient,
                     const Eigen::Matrix4d &pose,
                     const apollo::common::Point3D &point,
                     int *center_x, int *center_y) const;

  bool project_point_distort(const CameraCoeffient &coeffient,
                             const Eigen::Matrix4d &pose,
                             const apollo::common::Point3D &point,
                             int *center_x, int *center_y) const;

  Eigen::Matrix<double, 2, 1> pixel_denormalize(
      const Eigen::Matrix<double, 2, 1> &pt2d,
      const Eigen::Matrix<double, 3, 4> &camera_intrinsic,
      const Eigen::Matrix<double, 5, 1> &distort_params) const;
};

} // namespace traffic_light
} // namespace perception
} // namespace apollo

#endif  // ADU_PERCEPTION_TRAFFIC_LIGHT_PROJECTION_BASE_LIGHTS_PROJECTION_H
