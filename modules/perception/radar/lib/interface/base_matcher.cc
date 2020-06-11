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
#include "modules/perception/radar/lib/interface/base_matcher.h"

namespace apollo {
namespace perception {
namespace radar {

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

void BaseMatcher::IDMatch(const std::vector<RadarTrackPtr> &radar_tracks,
                          const base::Frame &radar_frame,
                          std::vector<TrackObjectPair> *assignments,
                          std::vector<size_t> *unassigned_tracks,
                          std::vector<size_t> *unassigned_objects) {
  size_t num_track = radar_tracks.size();
  const auto &objects = radar_frame.objects;
  double object_timestamp = radar_frame.timestamp;
  size_t num_obj = objects.size();
  if (num_track == 0 || num_obj == 0) {
    unassigned_tracks->resize(num_track);
    unassigned_objects->resize(num_obj);
    std::iota(unassigned_tracks->begin(), unassigned_tracks->end(), 0);
    std::iota(unassigned_objects->begin(), unassigned_objects->end(), 0);
    return;
  }
  std::vector<bool> track_used(num_track, false);
  std::vector<bool> object_used(num_obj, false);
  for (size_t i = 0; i < num_track; ++i) {
    const auto &track_object = radar_tracks[i]->GetObsRadar();
    double track_timestamp = radar_tracks[i]->GetTimestamp();
    if (track_object.get() == nullptr) {
      AERROR << "track_object is not available";
      continue;
    }
    int track_object_track_id = track_object->track_id;
    for (size_t j = 0; j < num_obj; ++j) {
      int object_track_id = objects[j]->track_id;
      if (track_object_track_id == object_track_id &&
          RefinedTrack(track_object, track_timestamp, objects[j],
                       object_timestamp)) {
        assignments->push_back(std::pair<size_t, size_t>(i, j));
        track_used[i] = true;
        object_used[j] = true;
      }
    }
  }
  for (size_t i = 0; i < track_used.size(); ++i) {
    if (!track_used[i]) {
      unassigned_tracks->push_back(i);
    }
  }
  for (size_t i = 0; i < object_used.size(); ++i) {
    if (!object_used[i]) {
      unassigned_objects->push_back(i);
    }
  }
}

bool BaseMatcher::RefinedTrack(const base::ObjectPtr &track_object,
                               double track_timestamp,
                               const base::ObjectPtr &radar_object,
                               double radar_timestamp) {
  // This function is supposed to return true in the base class.
  // Specific actions can be overrided in derived classes.
  return true;
}

}  // namespace radar
}  // namespace perception
}  // namespace apollo
