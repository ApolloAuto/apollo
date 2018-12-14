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

#include "modules/localization/lmd/predictor/perception/lm_provider.h"

#include <limits>

namespace apollo {
namespace localization {

LMProvider::LMProvider() {
  CHECK(apollo::common::util::GetProtoFromFile(FLAGS_lmd_rawinput_bin_file,
                                               &LaneMarkersPack_))
      << "Unable to get raw lanemarkers from file "
      << FLAGS_lmd_rawinput_bin_file;
}

const int64_t LMProvider::GetLaneMarkerPackSize() const {
  return LaneMarkersPack_.lane_markers_size();
}

const int64_t LMProvider::GetLaneMarkerSize(const int64_t& pack_index) const {
  return LaneMarkersPack_.lane_markers(pack_index).lane_marker_size();
}

double LMProvider::CalculateDistance(
    const apollo::common::PointENU& position,
    const apollo::common::PointENU& current_pos) const {
  double distance = sqrt(pow((position.x() - current_pos.x()), 2) +
                         pow((position.y() - current_pos.y()), 2));
  return distance;
}

const std::pair<int64_t, int64_t> LMProvider::FindNearestLaneMarkerIndex(
    const apollo::common::PointENU& position) const {
  std::pair<int64_t, int64_t> result(std::numeric_limits<int64_t>::max(),
                                     std::numeric_limits<int64_t>::max());
  if (GetLaneMarkerPackSize() == 0) {
    AERROR << "Empty LaneMarkersPack from file " << FLAGS_lmd_rawinput_bin_file;
    return result;
  }
  double distance = std::numeric_limits<double>::max();
  for (int64_t lane_mark_pack_index = 0;
       lane_mark_pack_index < GetLaneMarkerPackSize(); lane_mark_pack_index++) {
    for (int64_t lane_marker_index = 0;
         lane_marker_index < GetLaneMarkerSize(lane_mark_pack_index);
         lane_marker_index++) {
      if (LaneMarkersPack_.lane_markers(lane_mark_pack_index)
              .lane_marker(lane_marker_index)
              .points_size() == 0) {
        AERROR << "No points info in number " << lane_marker_index + 1
               << " lane marker in lane marker group "
               << (lane_mark_pack_index);
        return result;
      }
      for (int64_t point_number = 0;
           point_number < LaneMarkersPack_.lane_markers(lane_mark_pack_index)
                              .lane_marker(lane_marker_index)
                              .points_size();
           point_number++) {
        if (!LaneMarkersPack_.lane_markers(lane_mark_pack_index)
                 .lane_marker(lane_marker_index)
                 .points(point_number)
                 .has_position()) {
          AERROR << "No position info in number " << point_number + 1
                 << "point in number " << lane_marker_index + 1
                 << " lane marker in lane marker group "
                 << (lane_mark_pack_index);
          return result;
        }
        const apollo::common::PointENU current_position =
            LaneMarkersPack_.lane_markers(lane_mark_pack_index)
                .lane_marker(lane_marker_index)
                .points(point_number)
                .position();
        double temp_distance = CalculateDistance(position, current_position);
        if (temp_distance < distance) {
          distance = temp_distance;
          result.first = lane_mark_pack_index;
          result.second = lane_marker_index;
        }
      }
    }
  }
  return result;
}

const std::pair<int64_t, int64_t> LMProvider::GetPrevLaneMarkerIndex(
    const std::pair<int64_t, int64_t>& current_index) const {
  std::pair<int64_t, int64_t> result(std::numeric_limits<int64_t>::max(),
                                     std::numeric_limits<int64_t>::max());
  if (current_index.first < 0 ||
      current_index.first >= GetLaneMarkerPackSize() ||
      current_index.second <= 0 ||
      current_index.second >= GetLaneMarkerSize(current_index.first)) {
    AERROR << "No Previous LaneMarker of LaneMarker with index "
           << current_index.first << " , " << current_index.second;
    return result;
  }
  result.first = current_index.first;
  result.second = current_index.second - 1;
  return result;
}

const std::pair<int64_t, int64_t> LMProvider::GetNextLaneMarkerIndex(
    const std::pair<int64_t, int64_t>& current_index) const {
  std::pair<int64_t, int64_t> result(std::numeric_limits<int64_t>::max(),
                                     std::numeric_limits<int64_t>::max());
  if (current_index.first < 0 ||
      current_index.first >= GetLaneMarkerPackSize() ||
      current_index.second < 0 ||
      current_index.second >= GetLaneMarkerSize(current_index.first) - 1) {
    AERROR << "No Next LaneMarker of LaneMarker with index "
           << current_index.first << " , " << current_index.second;
    return result;
  }
  result.first = current_index.first;
  result.second = current_index.second + 1;
  return result;
}

const std::pair<int64_t, int64_t> LMProvider::GetLeftLaneMarkerIndex(
    const std::pair<int64_t, int64_t>& current_index) const {
  std::pair<int64_t, int64_t> result(std::numeric_limits<int64_t>::max(),
                                     std::numeric_limits<int64_t>::max());
  if (current_index.first <= 0 ||
      current_index.first >= GetLaneMarkerPackSize() ||
      current_index.second < 0 ||
      current_index.second >= GetLaneMarkerSize(current_index.first)) {
    AERROR << "No Left LaneMarker of LaneMarker with index "
           << current_index.first << " , " << current_index.second;
    return result;
  }
  result.first = current_index.first - 1;
  result.second = current_index.second;
  return result;
}

const std::pair<int64_t, int64_t> LMProvider::GetRightLaneMarkerIndex(
    const std::pair<int64_t, int64_t>& current_index) const {
  std::pair<int64_t, int64_t> result(std::numeric_limits<int>::max(),
                                     std::numeric_limits<int>::max());
  if (current_index.first < 0 ||
      current_index.first >= GetLaneMarkerPackSize() - 1 ||
      current_index.second < 0 ||
      current_index.second >= GetLaneMarkerSize(current_index.first)) {
    AERROR << "No Right LaneMarker of LaneMarker with index "
           << current_index.first << " , " << current_index.second;
    return result;
  }
  result.first = current_index.first + 1;
  result.second = current_index.second;
  return result;
}

const apollo::localization::OdometryLaneMarker* LMProvider::GetLaneMarker(
    const std::pair<int64_t, int64_t>& current_index) const {
  if (current_index.first < 0 ||
      current_index.first >= GetLaneMarkerPackSize() ||
      current_index.second < 0 ||
      current_index.second >= GetLaneMarkerSize(current_index.first)) {
    AERROR << "No LaneMarker with index (" << current_index.first << ","
           << current_index.second << ")";
    return nullptr;
  }
  return &LaneMarkersPack_.lane_markers(current_index.first)
              .lane_marker(current_index.second);
}

}  // namespace localization
}  // namespace apollo
