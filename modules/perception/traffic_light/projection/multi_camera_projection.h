// Copyright (c) 2016 Baidu.com, Inc. All Rights Reserved
// @author guiyilin(guiyilin@baidu.com)
// @date 2017/07/31 
// @file multi_camera_projection.h
// @brief Project a traffic_light onto image.
//        We were told traffic_light's location & we have known our location, 
//        and then we mark the traffic_light region on the image.
#ifndef ADU_PERCEPTION_TRAFFIC_LIGHT_PROJECTION_MULTI_CAMERA_PROJECTION_H
#define ADU_PERCEPTION_TRAFFIC_LIGHT_PROJECTION_MULTI_CAMERA_PROJECTION_H

#include <memory>
#include <vector>
#include <map>

#include "modules/perception/lib/base/file_util.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/traffic_light/interface/base_projection.h"

namespace apollo {
namespace perception {
namespace traffic_light {

//@brief 2 Camera Projection project the Light into the image. 
class MultiCamerasProjection {
 public:
  MultiCamerasProjection() {
  }

  virtual ~MultiCamerasProjection() = default;
  virtual bool init();
  virtual bool project(const CarPose &pose, const ProjectOption &option, Light *light) const;
  std::string name() const {
    return "TLPreprocessor";
  }
  bool has_camera(const CameraId &cam_id);

 private:
  std::map<std::string, CameraCoeffient> _camera_coeffients;
  std::vector<std::string> _camera_names;
  std::unique_ptr<BaseProjection> _projection;
};

} // namespace traffic_light
} // namespace perception
} // namespace apollo

#endif  // ADU_PERCEPTION_TRAFFIC_LIGHT_PROJECTION_MULTI_CAMERA_PROJECTION_H
