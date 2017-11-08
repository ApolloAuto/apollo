// Copyright (c) 2017 Baidu.com, Inc. All Rights Reserved
// @author guiyilin(guiyilin@baidu.com)
// @date 2017/08/08
// @file: proc_data.h
// @brief: 
// 
#ifndef ADU_PERCEPTION_TRAFFIC_LIGHT_ONBOARD_PROC_DATA_H
#define ADU_PERCEPTION_TRAFFIC_LIGHT_ONBOARD_PROC_DATA_H

#include "module/perception/traffic_light/base/image_lights.h"
#include "onboard/common_shared_data.h"

namespace adu {
namespace perception {
namespace traffic_light {

class TLProcData : public onboard::CommonSharedData<ImageLights> {
 public:
  TLProcData() = default;

  virtual ~TLProcData() = default;

  virtual std::string name() const override {
    return "TLProcData";
  }
};

} // namespace traffic_light
} // namespace perception
} // namespace adu

#endif  // ADU_PERCEPTION_TRAFFIC_LIGHT_ONBOARD_PROC_DATA_H
