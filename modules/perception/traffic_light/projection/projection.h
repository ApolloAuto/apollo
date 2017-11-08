// Copyright (c) 2016 Baidu.com, Inc. All Rights Reserved
// @author erlangz(zhengwenchao@baidu.com)
// @date 2016/09/26 17:48:42
// @file projection.h
// @brief Project a traffic_light onto image.
//        We were told traffic_light's location & we have known our location, 
//        and then we mark the traffic_light region on the image.
#ifndef ADU_PERCEPTION_TRAFFIC_LIGHT_ONBOARD_PROJECTION_PROJECTION_H
#define ADU_PERCEPTION_TRAFFIC_LIGHT_ONBOARD_PROJECTION_PROJECTION_H

#include <memory>

#include "lib/base/file_util.h"
#include "lib/config_manager/config_manager.h"
#include "module/perception/traffic_light/interface/base_projection.h"

namespace adu {
namespace perception {
namespace traffic_light {

//@brief 2 Camera Projection project the Light into the image. 
class TwoCamerasProjection {
 public:
  virtual ~TwoCamerasProjection() = default;
  virtual bool init();
  virtual bool project(const CarPose &pose, const ProjectOption &option, Light *light) const;
 private:
  CameraCoeffient _long_focus_camera_coeffient;
  CameraCoeffient _short_focus_camera_coeffient;
  std::unique_ptr<BaseProjection> _projection;
};

} // namespace traffic_light
} // namespace perception
} // namespace adu

#endif  // ADU_PERCEPTION_TRAFFIC_LIGHT_ONBOARD_PROJECTION_PROJECTION_H
