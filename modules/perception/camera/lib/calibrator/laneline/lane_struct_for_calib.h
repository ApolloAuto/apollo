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

#include <vector>

#include "Eigen/Eigen"

namespace apollo {
namespace perception {
namespace camera {

struct VanishingPoint {
  float pixel_pos[2] = {0};
  float distance_traveled = 0.0f;  // in meters
};

struct LaneLine {
  int type = 0;
  int color = 0;
  std::vector<Eigen::Vector2f> lane_point = {};
};

struct CmpLanePointY {
  bool operator()(const Eigen::Vector2f &pt1, const Eigen::Vector2f &pt2) {
    return pt1(1) > pt2(1);
  }
};

// struct CmpLanePointX {
//   bool operator() (const Eigen::Vector2f &pt1,
//                    const Eigen::Vector2f &pt2) {
//     return pt1(0) > pt2(0);
//   }
// };

// lane points for calibration
struct EgoLane {
  LaneLine left_line;
  LaneLine right_line;
};

}  // namespace camera
}  // namespace perception
}  // namespace apollo
