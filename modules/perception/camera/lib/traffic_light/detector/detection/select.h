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

#include <utility>
#include <vector>

#include "modules/perception/base/point.h"
#include "modules/perception/base/traffic_light.h"
#include "modules/perception/common/graph/hungarian_optimizer.h"

namespace apollo {
namespace perception {
namespace camera {

class Select {
 public:
  Select() = default;

  bool Init(int rows, int cols);

  void SelectTrafficLights(
      const std::vector<base::TrafficLightPtr> &refined_bboxes,
      std::vector<base::TrafficLightPtr> *hdmap_bboxes);

  double Calc2dGaussianScore(base::Point2DI p1, base::Point2DI p2, float sigma1,
                             float sigma2);

 private:
  common::HungarianOptimizer<float> munkres_;
};
}  // namespace camera
}  // namespace perception
}  // namespace apollo
