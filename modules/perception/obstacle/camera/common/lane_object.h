/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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

#ifndef MODULES_PERCEPTION_OBSTACLE_CAMERA_COMMON_LANE_OBJECT_H_
#define MODULES_PERCEPTION_OBSTACLE_CAMERA_COMMON_LANE_OBJECT_H_

#include <limits>
#include <vector>

#include "Eigen/Eigen"

namespace apollo {
namespace perception {

const float MAX_FLOAT = std::numeric_limits<float>::max();
const float LOWEST_FLOAT = -std::numeric_limits<float>::max();
const float MIN_ANGLE = LOWEST_FLOAT / 180.0f;
const float B_FLT_EPSILON = 0.000001f;  // should be changed

const float FOURTY_FIVE_DEGREE = 45.0f * M_PI / 180.0f;

// required number of points for lane line to determine the CIPV
const uint32_t MIN_LANE_LINE_LENGTH_FOR_CIPV_DETERMINATION = 2;
// Average width of lane
const float AVERAGE_LANE_WIDTH_IN_METER = 3.7f;
// Maximum vehicle width
const float MAX_VEHICLE_WIDTH_IN_METER = 2.5f;
// Margin from a virtual car lane to actual lane
const float MARGIN_VEHICLE_TO_LANE =
    (AVERAGE_LANE_WIDTH_IN_METER - MAX_VEHICLE_WIDTH_IN_METER) / 2.0f;
// The width of virtual egolane when there is only one lane line
const float SINGLE_VIRTUAL_EGOLANE_WIDTH_IN_METER =
    MAX_VEHICLE_WIDTH_IN_METER + MARGIN_VEHICLE_TO_LANE;  // 3.1f

// The width of virtual egolane when there is only one lane line
const float HALF_VEHICLE_WIDTH_IN_METER = MAX_VEHICLE_WIDTH_IN_METER / 2.0f;

typedef Eigen::Vector2i Point2Di;
typedef Eigen::Vector2f Point2Df;
typedef Eigen::Vector2d Point2Dd;

// This should be from lane detector
struct LaneLine {
  int type;   // solid, broken, double, zigzag, boundary, implicit
  int color;  // yellow, white
  //    eastl::fixed_vector<Point2Df, MAX_LANE_LINE_POINT> line_point;
  std::vector<Point2Df> line_point;
};

// Line segment by two points
struct LineSegment2Df {
  Point2Df start_point;
  Point2Df end_point;
};

struct VanishingPoint {
  Point2Df vanishing_point;
  float distance_traveled;
};

// two lane lines used for camera calibration
struct EgoLane {
  LaneLine left_line;
  LaneLine right_line;
};

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_CAMERA_COMMON_LANE_OBJECT_H_
