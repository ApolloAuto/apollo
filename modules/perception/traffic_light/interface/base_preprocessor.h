// Copyright (c) 2017 Baidu.com, Inc. All Rights Reserved
// @author guiyilin(guiyilin@baidu.com)
// @date 2017/08/07
// @file: base_preprocessor.h
// @brief: 
// 

#ifndef ADU_PERCEPTION_TRAFFIC_LIGHT_INTERFACE_BASE_PREPROCESSOR_H
#define ADU_PERCEPTION_TRAFFIC_LIGHT_INTERFACE_BASE_PREPROCESSOR_H
#include <string>
#include <vector>

#include "modules/perception/lib/base/noncopyable.h"
#include "modules/perception/lib/base/registerer.h"
#include "modules/perception/traffic_light/base/image.h"
#include "modules/perception/traffic_light/base/light.h"

namespace apollo {
namespace perception {
namespace traffic_light {

//@brief Reviser is the class is to revise the perception result.
//       It may use history info(Tracker) or some else info.
class BasePreprocessor {
 public:
  BasePreprocessor() = default;

  virtual ~BasePreprocessor() = default;

  //@brief init the reviser.
  virtual bool init() = 0;

  //@brief Revise's name
  virtual std::string name() const = 0;
};

REGISTER_REGISTERER(BasePreprocessor);
#define REGISTER_PREPROCESSOR(name) REGISTER_CLASS(BasePreprocessor, name)

} // namespace traffic_light
} // namespace perception
} // namespace apollo

#endif  // ADU_PERCEPTION_TRAFFIC_LIGHT_INTERFACE_BASE_PREPROCESSOR_H

