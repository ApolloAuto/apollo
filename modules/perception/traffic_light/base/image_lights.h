// Copyright (c) 2016 Baidu.com, Inc. All Rights Reserved
// @author erlangz(zhengwenchao@baidu.com)
// @date 2016/12/14 13:15:14
// @file traffic_light/onboard/image_lights.h
// @brief Struct Compine image & lights.
#ifndef ADU_PERCEPTION_TRAFFIC_LIGHT_ONBOARD_IMAGE_LIGHTS_H
#define ADU_PERCEPTION_TRAFFIC_LIGHT_ONBOARD_IMAGE_LIGHTS_H

#include <vector>
#include <memory>

#include "modules/perception/traffic_light/base/image.h"
#include "modules/perception/traffic_light/base/light.h"
#include "modules/perception/traffic_light/base/pose.h"

namespace apollo {
namespace perception {
namespace traffic_light {

struct ImagesPose {
  std::vector<ImageSharedPtr> images;
  CarPose pose;
};
struct ImageLights {
  std::shared_ptr<Image> image;
  CarPose pose;
  std::shared_ptr<LightPtrs> lights;
  std::shared_ptr<LightPtrs> lights_outside_image; //record the lights outside the lights.
  CameraId camera_id = UNKNOWN;
  double timestamp = 0.0;   // image's timestamp
  double pose_timestamp = 0.0;   // pose's timestamp

  double preprocess_receive_timestamp = 0.0;   // timestamp when received a image
  double preprocess_send_timestamp = 0.0;  // timestamp when PreprocessSubnode pub event
  bool is_pose_valid = false;
  bool has_signals = false;
  double diff_image_pose_ts = 0.0;  // image' timestamp minus the most recently pose's timestamp
  double diff_image_sys_ts = 0.0;  // image' timestamp system's timestamp
  size_t offset = 0;  // offset size between hdmap bbox and detection bbox
  size_t num_signals = 0;
};

} // namespace traffic_light
} // namespace perception
} // namespace apollo

#endif  // ADU_PERCEPTION_TRAFFIC_LIGHT_ONBOARD_IMAGE_LIGHTS_H
