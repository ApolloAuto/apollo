/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#ifndef MODULES_PERCEPTION_TRAFFIC_LIGHT_ONBOARD_PREPROCESSOR_DATA_H
#define MODULES_PERCEPTION_TRAFFIC_LIGHT_ONBOARD_PREPROCESSOR_DATA_H

#include <map>
#include <string>
#include <vector>
#include "modules/perception/onboard/common_shared_data.h"
#include "modules/perception/traffic_light/base/image_lights.h"

namespace apollo {
namespace perception {
namespace traffic_light {

extern std::vector<int> image_border_size;
extern std::map<TLColor, std::string> kColorStr;
extern const int kCountCameraId;
extern const int kLongFocusIdx;
extern const int kShortFocusIdx;
extern std::map<CameraId, int> kCameraIndicator;
class TLPreprocessingData : public CommonSharedData<ImageLights> {
 public:
  TLPreprocessingData() = default;

  virtual ~TLPreprocessingData() = default;

  std::string name() const override {
    return "TLPreprocessingData";
  }
};
REGISTER_SHAREDDATA(TLPreprocessingData);
}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_TRAFFIC_LIGHT_ONBOARD_PREPROCESSOR_DATA_H
