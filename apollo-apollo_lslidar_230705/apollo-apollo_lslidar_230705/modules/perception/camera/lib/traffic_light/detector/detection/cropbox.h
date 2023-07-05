/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#pragma once

#include "modules/perception/base/traffic_light.h"

namespace apollo {
namespace perception {
namespace camera {

class IGetBox {
 public:
  virtual void getCropBox(const int width, const int height,
                          const base::TrafficLightPtr &light,
                          base::RectI *crop_box) = 0;
};

class CropBox : public IGetBox {
 public:
  CropBox(float crop_scale, int min_crop_size);

  void Init(float crop_scale, int min_crop_size);

  virtual void getCropBox(const int width, const int height,
                          const base::TrafficLightPtr &light,
                          base::RectI *crop_box);

 private:
  float crop_scale_ = 2.5f;
  int min_crop_size_ = 270;
};

class CropBoxWholeImage : public IGetBox {
 public:
  virtual void getCropBox(const int width, const int height,
                          const base::TrafficLightPtr &light,
                          base::RectI *crop_box);
};

}  // namespace camera
}  // namespace perception
}  // namespace apollo
