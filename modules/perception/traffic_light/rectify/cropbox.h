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

#ifndef MODULES_PERCEPTION_TRAFFIC_LIGHT_CROPBOX_H_
#define MODULES_PERCEPTION_TRAFFIC_LIGHT_CROPBOX_H_

#include <vector>

#include "modules/perception/traffic_light/interface/green_interface.h"

namespace apollo {
namespace perception {
namespace traffic_light {

/**
 * @class CropBox
 * @brief get roi from input lights
 */
class CropBox : public IGetBox {
 public:
  CropBox(float crop_scale, float min_crop_size);

  void Init(float crop_scale, float min_crop_size);

  virtual void GetCropBox(const cv::Size &size,
                          const std::vector<LightPtr> &lights,
                          cv::Rect *cropbox);

 private:
  float crop_scale_ = 0.0;
  float min_crop_size_ = 0.0;
};

/**
 * @class CropBox
 * @brief use whole image as roi
 */
class CropBoxWholeImage : public IGetBox {
 public:
  virtual void GetCropBox(const cv::Size &size,
                          const std::vector<LightPtr> &lights,
                          cv::Rect *cropbox);
};

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_TRAFFIC_LIGHT_CROPBOX_H_
