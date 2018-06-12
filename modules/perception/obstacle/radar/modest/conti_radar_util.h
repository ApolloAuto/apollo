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

#ifndef MODULES_PERCEPTION_OBSTACLE_RADAR_MODEST_CONTI_RADAR_UTIL_H_
#define MODULES_PERCEPTION_OBSTACLE_RADAR_MODEST_CONTI_RADAR_UTIL_H_

#include "modules/perception/common/geometry_util.h"
#include "modules/perception/obstacle/radar/interface/base_radar_detector.h"

namespace apollo {
namespace perception {

class ContiRadarUtil {
 public:
  static bool IsFp(const ContiRadarObs& contiobs, const ContiParams& params,
                   const int delay_frames, const int tracking_times);

  static bool IsConflict(const Eigen::Vector3f& main_velocity,
                         const Eigen::Vector3f& velocity) {
    Eigen::Vector3f vector_temp1 = main_velocity;
    Eigen::Vector3f vector_temp2 = velocity;
    const float velocity_threshold = 1e-1;
    if (vector_temp1.head(2).norm() > velocity_threshold &&
        vector_temp2.head(2).norm() > velocity_threshold) {
      double theta = VectorTheta2dXy(vector_temp1, vector_temp2);
      if ((theta > 1.0 / 4.0 * M_PI && theta < 3.0 / 4.0 * M_PI) ||
          (theta > -3.0 / 4.0 * M_PI && theta < -1.0 / 4.0 * M_PI)) {
        return true;
      }
    }
    return false;
  }
};

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_RADAR_MODEST_CONTI_RADAR_UTIL_H_
