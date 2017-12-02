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

#ifndef MODULES_PERCEPTION_TRAFFIC_LIGHT_GREEN_INTERFACE_H
#define MODULES_PERCEPTION_TRAFFIC_LIGHT_GREEN_INTERFACE_H

#include <opencv2/opencv.hpp>
#include <vector>
#include "modules/perception/traffic_light/base/light.h"

namespace apollo {
namespace perception {
namespace traffic_light {
// class-wrapper
class ISelectLight {
 public:
  virtual void Select(const cv::Mat &ros_image,
                      const std::vector<LightPtr> &hdmap_bboxes,
                      const std::vector<LightPtr> &refined_bboxes,
                      std::vector<LightPtr> *selected_bboxes) = 0;
};

class IRefine {
 public:
  virtual void Perform(const cv::Mat &ros_image,
                       std::vector<LightPtr> *lights) = 0;
  virtual void SetCropBox(const cv::Rect &box) = 0;
};

class IGetBox {
 public:
  virtual void
  GetCropBox(const cv::Size &size, const std::vector<LightPtr> &lights,
             cv::Rect *cropbox) = 0;
};
class DummyRefine : public IRefine {
 public:
  void Perform(const cv::Mat &ros_image,
               std::vector<LightPtr> *lights) override {
  }
  void SetCropBox(const cv::Rect &box) override {
  }
};
}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
#endif
