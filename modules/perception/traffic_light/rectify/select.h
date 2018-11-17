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

#ifndef MODULES_PERCEPTION_TRAFFIC_LIGHT_RECTIFY_SELECT_H_
#define MODULES_PERCEPTION_TRAFFIC_LIGHT_RECTIFY_SELECT_H_

#include <vector>

#include "modules/perception/traffic_light/interface/green_interface.h"

namespace apollo {
namespace perception {
namespace traffic_light {

/**
 * @class GaussianSelect
 * @brief use gaussian score to select light
 */
class GaussianSelect : public ISelectLight {
 public:
  GaussianSelect() = default;

  virtual void Select(const cv::Mat &ros_image,
                      const std::vector<LightPtr> &hdmap_bboxes,
                      const std::vector<LightPtr> &refined_bboxes,
                      std::vector<LightPtr> *selected_bboxes);
};

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_TRAFFIC_LIGHT_RECTIFY_SELECT_H_
