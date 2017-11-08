// Copyright 2016 Baidu Inc. All Rights Reserved.
// @author: Hengyu Li (lihengyu@baidu.com)
// @file: base_rectifier.h
// @brief: interface of traffic light rectifier

#ifndef ADU_PERCEPTION_TRAFFIC_LIGHT_INTERFACE_BASE_RECTIFIER_H
#define ADU_PERCEPTION_TRAFFIC_LIGHT_INTERFACE_BASE_RECTIFIER_H

#include <string>

#include "lib/base/noncopyable.h"
#include "lib/base/registerer.h"
#include "modules/perception/traffic_light/base/image.h"
#include "modules/perception/traffic_light/base/light.h"

namespace adu {
namespace perception {
namespace traffic_light {

struct RectifyOption {
  CameraId camera_id = UNKNOWN;
};

//@brief Rectifier receives the Region of lights from HD-Map, 
//       While the region may be too large or not accuray.
//       Rectifier should rectify the region, send the accuray regions to classifier.
class BaseRectifier {
 public:
  BaseRectifier() = default;

  virtual ~BaseRectifier() = default;

  virtual bool init() = 0;

  // @brief: rectify light region from image or part of it
  // @param [in] const Image&: input image
  // @param [in] const RectifyOption&: rectify options
  // @param [in/out] Lights
  // @return  bool
  virtual bool rectify(const Image &image, const RectifyOption &option,
                       std::vector<LightPtr> *lights) = 0;

  // @brief name
  virtual std::string name() const = 0;

 private:
  DISALLOW_COPY_AND_ASSIGN(BaseRectifier);
};

REGISTER_REGISTERER(BaseRectifier);
#define REGISTER_RECTIFIER(name) REGISTER_CLASS(BaseRectifier, name)

}  // namespace traffic_light
}  // namespace perception
}  // namespace adu

#endif  // ADU_PERCEPTION_TRAFFIC_LIGHT_INTERFACE_BASE_RECTIFIER_H
