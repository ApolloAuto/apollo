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

/**
 * @file
 * @brief This file provides the declaration of the class
 * "NavigationExpander".
 */

#ifndef MODULES_TOOLS_NAVI_GENERATOR_BACKEND_UTIL_NAVIGATION_EXPANDER_H_
#define MODULES_TOOLS_NAVI_GENERATOR_BACKEND_UTIL_NAVIGATION_EXPANDER_H_

#include <list>
#include <string>
#include <vector>

#include "modules/planning/reference_line/reference_point.h"

/**
 * @namespace apollo::navi_generator::util
 * @brief apollo::navi_generator::util
 */
namespace apollo {
namespace navi_generator {
namespace util {

typedef std::vector<apollo::planning::ReferencePoint> LanePoints;

struct LanePointsInfo {
  int index;
  LanePoints points;
};

struct ExpandedFileInfo {
  int index;
  std::string file_name;
};

class NavigationExpander {
 public:
  NavigationExpander() = default;
  ~NavigationExpander() = default;

 public:
  /**
   * @brief Expand multiple lanes.
   * @param src_lane The input original lane points.
   * @param left_lane_number The left lane number.
   * @param right_lane number The right lane number.
   * @param lane_width The lane width.
   * @param dst_lane_list The list of expanded lane points.
   * @return  Return true for success.
   */
  bool ExpandLane(const LanePoints& src_lane, const int left_lane_number,
                  const int right_lane_number, const double lane_width,
                  std::list<LanePointsInfo>* const dst_lane_list);
  /**
   * @brief Expand multiple lanes.
   * @param src_smoothed_file_name The input smoothed file name.
   * @param left_lane_number The left lane number.
   * @param right_lane number The right lane number.
   * @param lane_width The lane width.
   * @param expanded_files The list of expanded file list.
   * @return  Return true for success.
   */
  bool ExpandLane(const std::string& src_smoothed_file_name,
                  const int left_lane_number, const int right_lane_number,
                  const double lane_width,
                  std::list<ExpandedFileInfo>* const expanded_files);

 private:
  /**
   * @brief Expand one lane.
   * @param src_lane The input original lane points..
   * @param lane_width The lane width.
   * @param is_left_expand The flag of left or right expand.
   * @param dst_lane The expanded lane points.
   * @return  Return true for success.
   */
  bool ExpandOneLane(const LanePoints& src_lane, const double lane_width,
                     const bool is_left_expand, LanePoints* const dst_lane);
};

}  // namespace util
}  // namespace navi_generator
}  // namespace apollo

#endif  // MODULES_TOOLS_NAVI_GENERATOR_BACKEND_UTIL_NAVIGATION_EXPANDER_H_
