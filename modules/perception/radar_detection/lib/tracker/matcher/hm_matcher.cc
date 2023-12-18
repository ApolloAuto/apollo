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
#include "modules/perception/radar_detection/lib/tracker/matcher/hm_matcher.h"

#include <string>
#include <utility>

#include "modules/perception/radar_detection/proto/matcher_config.pb.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/perception/common/algorithm/graph/secure_matrix.h"
#include "modules/perception/common/util.h"

namespace apollo {
namespace perception {
namespace radar {

bool HMMatcher::Init(const MatcherInitOptions &options) {
  std::string config_file =
      GetConfigFile(options.config_path, options.config_file);
  MatcherConfig config_params;
  ACHECK(cyber::common::GetProtoFromFile(config_file, &config_params))
      << "Failed to parse MatcherConfig config file.";
  double max_match_distance = config_params.max_match_distance();
  double bound_match_distance = config_params.bound_match_distance();
  BaseMatcher::SetMaxMatchDistance(max_match_distance);
  BaseMatcher::SetBoundMatchDistance(bound_match_distance);
  return true;
}

// @brief match radar objects to tracks
// @params[IN] radar_tracks: global tracks
// @params[IN] radar_frame: current radar frame
// @params[IN] options: matcher options for future use
// @params[OUT] assignments: matched pair of tracks and measurements
// @params[OUT] unassigned_tracks: unmatched tracks
// @params[OUT] unassigned_objects: unmatched objects
// @return nothing
bool HMMatcher::Match(const std::vector<RadarTrackPtr> &radar_tracks,
                      const base::Frame &radar_frame,
                      const TrackObjectMatcherOptions &options,
                      std::vector<TrackObjectPair> *assignments,
                      std::vector<size_t> *unassigned_tracks,
                      std::vector<size_t> *unassigned_objects) {
  IDMatch(radar_tracks, radar_frame, assignments, unassigned_tracks,
          unassigned_objects);
  TrackObjectPropertyMatch(radar_tracks, radar_frame, assignments,
                           unassigned_tracks, unassigned_objects);
  return true;
}

bool HMMatcher::RefinedTrack(const base::ObjectPtr &track_object,
                             double track_timestamp,
                             const base::ObjectPtr &radar_object,
                             double radar_timestamp) {
  double dist = 0.5 * DistanceBetweenObs(track_object, track_timestamp,
                                         radar_object, radar_timestamp) +
                0.5 * DistanceBetweenObs(radar_object, radar_timestamp,
                                         track_object, track_timestamp);

  return dist < BaseMatcher::GetMaxMatchDistance();
}

void HMMatcher::TrackObjectPropertyMatch(
    const std::vector<RadarTrackPtr> &radar_tracks,
    const base::Frame &radar_frame, std::vector<TrackObjectPair> *assignments,
    std::vector<size_t> *unassigned_tracks,
    std::vector<size_t> *unassigned_objects) {
  if (unassigned_tracks->empty() || unassigned_objects->empty()) {
    return;
  }
  std::vector<std::vector<double>> association_mat(unassigned_tracks->size());
  for (size_t i = 0; i < association_mat.size(); ++i) {
    association_mat[i].resize(unassigned_objects->size(), 0);
  }
  ComputeAssociationMat(radar_tracks, radar_frame, *unassigned_tracks,
                        *unassigned_objects, &association_mat);

  // from perception-common
  algorithm::SecureMat<double> *global_costs =
      hungarian_matcher_.mutable_global_costs();
  global_costs->Resize(unassigned_tracks->size(), unassigned_objects->size());
  for (size_t i = 0; i < unassigned_tracks->size(); ++i) {
    for (size_t j = 0; j < unassigned_objects->size(); ++j) {
      (*global_costs)(i, j) = association_mat[i][j];
    }
  }
  std::vector<TrackObjectPair> property_assignments;
  std::vector<size_t> property_unassigned_tracks;
  std::vector<size_t> property_unassigned_objects;
  hungarian_matcher_.Match(
      BaseMatcher::GetMaxMatchDistance(), BaseMatcher::GetBoundMatchDistance(),
      algorithm::GatedHungarianMatcher<double>::OptimizeFlag::OPTMIN,
      &property_assignments, &property_unassigned_tracks,
      &property_unassigned_objects);

  for (size_t i = 0; i < property_assignments.size(); ++i) {
    size_t gt_idx = unassigned_tracks->at(property_assignments[i].first);
    size_t go_idx = unassigned_objects->at(property_assignments[i].second);
    assignments->push_back(std::pair<size_t, size_t>(gt_idx, go_idx));
  }
  std::vector<size_t> temp_unassigned_tracks;
  std::vector<size_t> temp_unassigned_objects;
  for (size_t i = 0; i < property_unassigned_tracks.size(); ++i) {
    size_t gt_idx = unassigned_tracks->at(property_unassigned_tracks[i]);
    temp_unassigned_tracks.push_back(gt_idx);
  }
  for (size_t i = 0; i < property_unassigned_objects.size(); ++i) {
    size_t go_idx = unassigned_objects->at(property_unassigned_objects[i]);
    temp_unassigned_objects.push_back(go_idx);
  }
  *unassigned_tracks = temp_unassigned_tracks;
  *unassigned_objects = temp_unassigned_objects;
}

void HMMatcher::ComputeAssociationMat(
    const std::vector<RadarTrackPtr> &radar_tracks,
    const base::Frame &radar_frame,
    const std::vector<size_t> &unassigned_tracks,
    const std::vector<size_t> &unassigned_objects,
    std::vector<std::vector<double>> *association_mat) {
  double frame_timestamp = radar_frame.timestamp;
  for (size_t i = 0; i < unassigned_tracks.size(); ++i) {
    for (size_t j = 0; j < unassigned_objects.size(); ++j) {
      const base::ObjectPtr &track_object =
          radar_tracks[unassigned_tracks[i]]->GetObs();
      const base::ObjectPtr &frame_object =
          radar_frame.objects[unassigned_objects[j]];
      double track_timestamp =
          radar_tracks[unassigned_tracks[i]]->GetTimestamp();
      double distance_forward = DistanceBetweenObs(
          track_object, track_timestamp, frame_object, frame_timestamp);
      double distance_backward = DistanceBetweenObs(
          frame_object, frame_timestamp, track_object, track_timestamp);
      association_mat->at(i).at(j) =
          0.5 * distance_forward + 0.5 * distance_backward;
    }
  }
}

double HMMatcher::DistanceBetweenObs(const base::ObjectPtr &obs1,
                                     double timestamp1,
                                     const base::ObjectPtr &obs2,
                                     double timestamp2) {
  double time_diff = timestamp2 - timestamp1;
  return (obs2->center - obs1->center -
          obs1->velocity.cast<double>() * time_diff)
      .head(2)
      .norm();
}

PERCEPTION_REGISTER_MATCHER(HMMatcher);

}  // namespace radar
}  // namespace perception
}  // namespace apollo
