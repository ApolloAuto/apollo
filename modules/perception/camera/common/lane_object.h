/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#pragma once

#include <limits>
#include <vector>

#include "Eigen/Eigen"

namespace apollo {
namespace perception {

constexpr float kMaxFloat = std::numeric_limits<float>::max();
constexpr float kLowestFloat = -std::numeric_limits<float>::max();
constexpr float kMinAngle = kLowestFloat / 180.0f;
constexpr float kFloatEpsilon = 0.000001f;  // should be changed

constexpr float k45DegreeInRadian = static_cast<float>(45.0f * M_PI) / 180.0f;

// required number of points for lane line to determine the CIPV
constexpr uint32_t kMinLaneLineLengthForCIPV = 2;
// Average width of lane
constexpr float kAverageLaneWidthInMeter = 3.7f;
// Maximum vehicle width
constexpr float kMaxVehicleWidthInMeter = 1.87f;
// Margin from a virtual car lane to actual lane
constexpr float kMarginVehicleToLane =
    (kAverageLaneWidthInMeter - kMaxVehicleWidthInMeter) / 2.0f;
// The width of virtual egolane when there is only one lane line
constexpr float kSingleVirtualEgolaneWidthInMeter =
    kMaxVehicleWidthInMeter + kMarginVehicleToLane;  // 3.1f

// The width of virtual egolane when there is only one lane line
constexpr float kHalfVehicleWidthInMeter = kMaxVehicleWidthInMeter / 2.0f;

typedef Eigen::Vector2i Point2Di;
typedef Eigen::Vector2f Point2Df;
typedef Eigen::Vector2d Point2Dd;

// This should be from lane detector
struct LaneLineSimple {
  LaneLineSimple() { line_point.reserve(100); }
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
  LaneLineSimple left_line;
  LaneLineSimple right_line;
};

}  // namespace perception
}  // namespace apollo
