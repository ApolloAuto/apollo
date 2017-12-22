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

#ifndef MODULES_PERCEPTION_TRAFFIC_LIGHT_GREEN_INTERFACE_H_
#define MODULES_PERCEPTION_TRAFFIC_LIGHT_GREEN_INTERFACE_H_

#include <vector>

#include "opencv2/opencv.hpp"

#include "modules/perception/traffic_light/base/light.h"

namespace apollo {
namespace perception {
namespace traffic_light {
/**
 * @class ISelectLight
 * @brief base class for selecting light
 *        given detection boxes and hdmap boxes
 */
class ISelectLight {
 public:
  /**
   * @brief: select lights corresponding to hdmap
   * @param  input image, maybe useless
   * @param  light boxes from hdmap
   * @param  light boxes detected
   * @param  selected boxes
   */
  virtual void Select(const cv::Mat &ros_image,
                      const std::vector<LightPtr> &hdmap_bboxes,
                      const std::vector<LightPtr> &refined_bboxes,
                      std::vector<LightPtr> *selected_bboxes) = 0;
};

/**
 * @class IRefine
 * @brief base class for refining light
 *        this class will add some accuracy information
 *        to input lights
 */
class IRefine {
 public:
  /**
   * @brief: main process function
   * @param  input image
   * @param  light boxes from last precudure,
   *         when perform finished, lights will
   *         contain more information
   */
  virtual void Perform(const cv::Mat &ros_image,
                       std::vector<LightPtr> *lights) = 0;
  /**
   * @brief: support ROI
   * @param  input roi
   */
  virtual void SetCropBox(const cv::Rect &box) = 0;
};

/**
 * @class IGetBox
 * @brief base class for getting ROI
 */
class IGetBox {
 public:
  /**
   * @brief: get ROI
   * @param  input image size
   * @param  lights from projection
   * @param  final roi
   */
  virtual void GetCropBox(const cv::Size &size,
                          const std::vector<LightPtr> &lights,
                          cv::Rect *cropbox) = 0;
};

/**
 * @class DummyRefine
 * @brief a dummy class
 */
class DummyRefine : public IRefine {
 public:
  void Perform(const cv::Mat &ros_image,
               std::vector<LightPtr> *lights) override {}
  void SetCropBox(const cv::Rect &box) override {}
};

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_TRAFFIC_LIGHT_GREEN_INTERFACE_H_
