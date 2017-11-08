// Copyright (c) 2016 Baidu.com, Inc. All Rights Reserved
// @author erlangz(zhengwenchao@baidu.com)
// @date 2016/09/12 21:39:18
// @file: preprocessor_data is the share data 
//        from PreprocessorSubnode to DetectorSubnode
//
#ifndef ADU_PERCEPTION_TRAFFIC_LIGHT_ONBOARD_PREPROCESSOR_DATA_H
#define ADU_PERCEPTION_TRAFFIC_LIGHT_ONBOARD_PREPROCESSOR_DATA_H

#include "modules/perception/onboard/common_shared_data.h"
#include "modules/perception/traffic_light/base/image_lights.h"

namespace apollo {
namespace perception {
namespace traffic_light {

class TLPreprocessingData : public CommonSharedData<ImageLights> {
 public:
  TLPreprocessingData() = default;

  virtual ~TLPreprocessingData() = default;

  virtual std::string name() const override {
    return "TLPreprocessingData";
  }
};

} // namespace traffic_light
} // namespace perception
} // namespace apollo

#endif  // ADU_PERCEPTION_TRAFFIC_LIGHT_ONBOARD_PREPROCESSOR_DATA_H

