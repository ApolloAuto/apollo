// Copyright (c) 2016 Baidu.com, Inc. All Rights Reserved
// @author zhengwenchao(zhengwenchao@baidu.com)
// @file:base_reviser.h
// @brief: the interface for revise the traffic light's perception result.
//
#ifndef ADU_PERCEPTION_TRAFFIC_LIGHT_INTERFACE_BASE_REVISER_H
#define ADU_PERCEPTION_TRAFFIC_LIGHT_INTERFACE_BASE_REVISER_H
#include <string>
#include <vector>

#include "modules/perception/lib/base/registerer.h"
#include "modules/perception/traffic_light/base/image.h"
#include "modules/perception/traffic_light/base/light.h"

namespace apollo {
namespace perception {
namespace traffic_light {

struct ReviseOption {
  explicit ReviseOption(const double timestamp) :
      ts(timestamp) {
  }
  double ts;         // timestamp for lights
};

//@brief Reviser is the class is to revise the perception result.
//       It may use history info(Tracker) or some else info.
class BaseReviser {
 public:
  BaseReviser() = default;

  virtual ~BaseReviser() = default;

  //@brief init the reviser.
  virtual bool Init() = 0;

  //@brief reviser revise  the perception result
  //       ASSERT(rectifed_result.size == perception_result.size)
  //@param [in] option
  //@param [in/out] rectifed_result
  //@return true/false
  virtual bool Revise(const ReviseOption &option, std::vector<LightPtr> *lights) = 0;

  //@brief Revise's name
  virtual std::string name() const = 0;
};

REGISTER_REGISTERER(BaseReviser);
#define REGISTER_REVISER(name) REGISTER_CLASS(BaseReviser, name)

} // namespace traffic_light
} // namespace perception
} // namespace apollo

#endif  // ADU_PERCEPTION_TRAFFIC_LIGHT_INTERFACE_BASE_REVISER_H

// @date 2016/09/07 17:41:16
