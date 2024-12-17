/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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
#include "modules/perception/camera_detection_occupancy/interface/base_matcher.h"

#include <numeric>

namespace apollo {
namespace perception {
namespace camera {

double BaseMatcher::s_max_match_distance_ = 2.5;
double BaseMatcher::s_bound_match_distance_ = 10.0;

void BaseMatcher::SetMaxMatchDistance(double dist) {
  s_max_match_distance_ = dist;
}

double BaseMatcher::GetMaxMatchDistance() { return s_max_match_distance_; }

void BaseMatcher::SetBoundMatchDistance(double dist) {
  s_bound_match_distance_ = dist;
}

double BaseMatcher::GetBoundMatchDistance() { return s_bound_match_distance_; }

bool BaseMatcher::RefinedTrack(const base::ObjectPtr &track_object,
                               double track_timestamp,
                               const base::ObjectPtr &camera_object,
                               double camera_timestamp) {
  // This function is supposed to return true in the base class.
  // Specific actions can be overrided in derived classes.
  return true;
}

}  // namespace camera
}  // namespace perception
}  // namespace apollo
