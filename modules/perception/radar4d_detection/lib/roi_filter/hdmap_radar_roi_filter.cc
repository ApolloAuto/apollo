/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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
#include "modules/perception/radar4d_detection/lib/roi_filter/hdmap_radar_roi_filter.h"

#include <vector>

namespace apollo {
namespace perception {
namespace radar4d {

bool HdmapRadarRoiFilter::RoiFilter(const RoiFilterOptions& options,
                                    RadarFrame* radar_frame) {
  if (options.roi == nullptr ||
     (options.roi->road_polygons.empty() &&
      options.roi->junction_polygons.empty())) {
    return true;
  }

  std::vector<base::ObjectPtr> objects = radar_frame->segmented_objects;
  for (size_t i = 0; i < objects.size(); i++) {
    objects[i]->center = radar_frame->radar2world_pose * objects[i]->center;
  }

  radar_frame->segmented_objects.clear();
  radar_frame->segmented_objects.reserve(objects.size());
  for (std::size_t i = 0; i < objects.size(); i++) {
    if (algorithm::IsObjectInRoi(options.roi, objects[i])) {
      radar_frame->segmented_objects.push_back(objects[i]);
    }
  }

  for (size_t i = 0; i < radar_frame->segmented_objects.size(); i++) {
    radar_frame->segmented_objects[i]->center =
      radar_frame->radar2world_pose.inverse() *
      radar_frame->segmented_objects[i]->center;
  }

  return radar_frame->segmented_objects.size() > 0;
}

PERCEPTION_REGISTER_ROI_FILTER(HdmapRadarRoiFilter);

}  // namespace radar4d
}  // namespace perception
}  // namespace apollo
