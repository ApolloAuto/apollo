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
#pragma once

#include <vector>

#include "modules/perception/base/point.h"

namespace apollo {
namespace perception {
namespace base {

enum class LaneLineType {
  WHITE_DASHED = 0,
  WHITE_SOLID,
  YELLOW_DASHED,
  YELLOW_SOLID
};

/// Definition of the position of a lane marking in respect to the ego lane.
enum class LaneLinePositionType {
  CURB_LEFT = -5,
  FOURTH_LEFT = -4,
  THIRD_LEFT = -3,
  ADJACENT_LEFT = -2,  //!< lane marking on the left side next to ego lane
  EGO_LEFT = -1,       //!< left lane marking of the ego lane
  EGO_CENTER = 0,      //!< center lane marking of the ego lane, changing lane
  EGO_RIGHT = 1,       //!< right lane marking of the ego lane
  ADJACENT_RIGHT = 2,  //!< lane marking on the right side next to ego lane
  THIRD_RIGHT = 3,
  FOURTH_RIGHT = 4,
  CURB_RIGHT = 5,
  OTHER = 6,   //!< other types of lane
  UNKNOWN = 7  //!< background
};

// Definition of the use type of a lane mark in lane adjustment
enum class LaneLineUseType { REAL = 0, VIRTUAL };

// y = a*x^3 + b*x^2 + c*x + d
struct LaneLineCubicCurve {
  float x_start;
  float x_end;
  float a;
  float b;
  float c;
  float d;
};

//  end points for localization
struct EndPoints {
  Point2DF start;
  Point2DF end;
};

struct LaneLine {
  LaneLineType type;
  LaneLinePositionType pos_type;
  // @brief image coordinate system
  LaneLineCubicCurve curve_car_coord;
  // @brief camera coordinate system
  LaneLineCubicCurve curve_camera_coord;
  // @brief image coordinate system
  LaneLineCubicCurve curve_image_coord;
  // @brief curve image point set
  std::vector<Point2DF> curve_image_point_set;
  // @brief curve camera point set
  std::vector<Point3DF> curve_camera_point_set;
  // @brief curve car coord point set, only on XY plane
  std::vector<Point2DF> curve_car_coord_point_set;
  // @brief image end point set
  std::vector<EndPoints> image_end_point_set;
  // @brief track id
  int track_id = -1;
  // @brief confidence for lane line
  float confidence = 1.0f;

  LaneLineUseType use_type;
};

}  // namespace base
}  // namespace perception
}  // namespace apollo
