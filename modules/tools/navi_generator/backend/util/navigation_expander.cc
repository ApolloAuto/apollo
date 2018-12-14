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
 * @brief This file provides the implementation of the class
 * "NavigationExpander".
 */
#include "modules/tools/navi_generator/backend/util/navigation_expander.h"

#include <cmath>
#include <list>
#include <memory>
#include <string>
#include <vector>

#include "modules/common/log.h"
#include "modules/tools/navi_generator/backend/util/file_operator.h"

namespace apollo {
namespace navi_generator {
namespace util {

using apollo::planning::ReferencePoint;

bool NavigationExpander::ExpandLane(
    const std::string& src_smoothed_file_name, const int left_lane_number,
    const int right_lane_number, const double lane_width,
    std::list<ExpandedFileInfo>* const expanded_files) {
  CHECK_NOTNULL(expanded_files);
  std::vector<apollo::planning::ReferencePoint> lane_points;
  std::list<LanePointsInfo> expanded_lane_list;
  FileOperator file_operator;
  // Import smoothed file.
  if (!file_operator.Import(src_smoothed_file_name, &lane_points)) {
    AERROR << "Import file failed";
    return false;
  }
  // Expand lane
  if (!ExpandLane(lane_points, left_lane_number, right_lane_number, lane_width,
                  &expanded_lane_list)) {
    AERROR << "Expand Lane failed";
    return false;
  }
  // Export expanded lane data to files
  if (expanded_lane_list.empty()) {
    AERROR << "Expanded Lane list is empty";
    return false;
  }
  std::string::size_type n = src_smoothed_file_name.rfind('.');
  std::string prefix = src_smoothed_file_name.substr(0, n);
  for (const auto& lane_points_info : expanded_lane_list) {
    int index = lane_points_info.index;
    std::string file_name =
        prefix + "_" + std::to_string(lane_points_info.index) + ".smoothed";
    ExpandedFileInfo expanded_file_info;
    expanded_file_info.index = index;
    expanded_file_info.file_name = file_name;
    // Skip export source smoothed file
    if (lane_points_info.index == right_lane_number) {
      expanded_file_info.file_name = src_smoothed_file_name;
      expanded_files->emplace_back(expanded_file_info);
      continue;
    }
    if (!file_operator.Export(file_name, lane_points_info.points)) {
      AERROR << "Expanded Lane list is empty";
      return false;
    }
    expanded_files->emplace_back(expanded_file_info);
  }
  return true;
}

bool NavigationExpander::ExpandLane(
    const LanePoints& src_lane, const int left_lane_number,
    const int right_lane_number, const double lane_width,
    std::list<LanePointsInfo>* const dst_lane_list) {
  CHECK_NOTNULL(dst_lane_list);
  bool is_left_expand = false;
  // Current lane
  LanePointsInfo src_lane_points_info;
  src_lane_points_info.index = right_lane_number;
  src_lane_points_info.points = src_lane;
  dst_lane_list->emplace_back(src_lane_points_info);
  // Expand right lane.
  for (int i = 0; i < right_lane_number; ++i) {
    LanePointsInfo lane_points_info = dst_lane_list->back();
    LanePointsInfo right_lane_points_info;
    right_lane_points_info.index = right_lane_number - i - 1;
    is_left_expand = false;
    ExpandOneLane(lane_points_info.points, lane_width, is_left_expand,
                  &right_lane_points_info.points);
    dst_lane_list->emplace_back(right_lane_points_info);
  }

  // Expand left lane.
  for (int j = 0; j < left_lane_number; ++j) {
    LanePointsInfo lane_points_info = dst_lane_list->front();
    LanePointsInfo left_lane_points_info;
    left_lane_points_info.index = lane_points_info.index + j + 1;
    is_left_expand = true;
    ExpandOneLane(lane_points_info.points, lane_width, is_left_expand,
                  &left_lane_points_info.points);
    dst_lane_list->emplace_front(left_lane_points_info);
  }
  return true;
}

bool NavigationExpander::ExpandOneLane(const LanePoints& src_lane,
                                       const double lane_width,
                                       const bool is_left_expand,
                                       LanePoints* const dst_lane) {
  CHECK_NOTNULL(dst_lane);

  for (const auto& ref_point : src_lane) {
    ReferencePoint pt = ref_point;
    if (is_left_expand) {
      pt.set_x(ref_point.x() +
               lane_width * std::cos(ref_point.heading() + M_PI_2));
      pt.set_y(ref_point.y() +
               lane_width * std::sin(ref_point.heading() + M_PI_2));
    } else {
      pt.set_x(ref_point.x() +
               lane_width * std::cos(ref_point.heading() - M_PI_2));
      pt.set_y(ref_point.y() +
               lane_width * std::sin(ref_point.heading() - M_PI_2));
    }
    dst_lane->push_back(pt);
  }

  return true;
}

}  // namespace util
}  // namespace navi_generator
}  // namespace apollo
