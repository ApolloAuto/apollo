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

#include "modules/perception/common/algorithm/graph/hungarian_optimizer.h"
#include "modules/perception/common/base/point.h"
#include "modules/perception/common/base/traffic_light.h"

namespace apollo {
namespace perception {
namespace trafficlight {

class Select {
public:
    /**
     * @brief Construct a new select object.
     *
     */
    Select() = default;
    /**
     * @brief Initialize select object parameters.
     *
     * @param rows
     * @param cols
     * @return true
     * @return false
     */
    bool Init(int rows, int cols);
    /**
     * @brief Choose traffic lights using Gaussian calculation scores.
     *
     * @param refined_bboxes
     * @param hdmap_bboxes
     */
    void SelectTrafficLights(
            const std::vector<base::TrafficLightPtr> &refined_bboxes,
            std::vector<base::TrafficLightPtr> *hdmap_bboxes);
    /**
     * @brief Calculate 2d Gaussian score.
     *
     * @param p1
     * @param p2
     * @param sigma1
     * @param sigma2
     * @return double
     */
    double Calc2dGaussianScore(base::Point2DI p1, base::Point2DI p2, float sigma1, float sigma2);

private:
    algorithm::HungarianOptimizer<float> munkres_;
};
}  // namespace trafficlight
}  // namespace perception
}  // namespace apollo
