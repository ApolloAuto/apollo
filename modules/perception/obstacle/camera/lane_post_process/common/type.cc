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

#include "modules/perception/obstacle/camera/lane_post_process/common/type.h"

namespace apollo {
namespace perception {

void LaneObjectsToLaneMarkerProto(const LaneObjects& lane_objects,
                                  LaneMarkers* lane_markers) {
  for (auto &lane_obj : lane_objects) {
    if (lane_obj.spatial == SpatialLabelType::L_0) {
      lane_obj.ToLaneMarkerProto(lane_markers->mutable_left_lane_marker());
    } else if (lane_obj.spatial == SpatialLabelType::R_0) {
      lane_obj.ToLaneMarkerProto(lane_markers->mutable_right_lane_marker());
    } else if (lane_obj.spatial == SpatialLabelType::L_1) {
      lane_obj.ToLaneMarkerProto(lane_markers->add_next_left_lane_marker());
    } else if (lane_obj.spatial == SpatialLabelType::R_1) {
      lane_obj.ToLaneMarkerProto(lane_markers->add_next_right_lane_marker());
    }
  }
}

}  // namespace perception
}  // namespace apollo
