/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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

#include "modules/perception/radar4d_detection/lib/tracker/multi_radar_fusion/mrf_track_object_matcher.h"

#include <numeric>

#include "cyber/common/file.h"
#include "modules/perception/radar4d_detection/lib/tracker/multi_radar_fusion/proto/mrf_config.pb.h"

namespace apollo {
namespace perception {
namespace radar4d {

bool MrfTrackObjectMatcher::Init(
    const MrfTrackObjectMatcherInitOptions &options) {
  std::string config_file = "mrf_track_object_matcher.pb.txt";
  if (!options.config_file.empty()) {
    config_file = options.config_file;
  }
  config_file = GetConfigFile(options.config_path, config_file);
  MrfTrackObjectMatcherConfig config;
  ACHECK(cyber::common::GetProtoFromFile(config_file, &config));

  foreground_matcher_ = BaseBipartiteGraphMatcherRegisterer::GetInstanceByName(
      config.foreground_mathcer_method());
  ACHECK(foreground_matcher_ != nullptr);
  AINFO << "MrfTrackObjectMatcher, fg: " << foreground_matcher_->Name();
  background_matcher_ = BaseBipartiteGraphMatcherRegisterer::GetInstanceByName(
      config.background_matcher_method());
  ACHECK(background_matcher_ != nullptr);
  AINFO << "MrfTrackObjectMatcher, bg: " << background_matcher_->Name();
  foreground_matcher_->cost_matrix()->Reserve(1000, 1000);
  background_matcher_->cost_matrix()->Reserve(1000, 1000);

  track_object_distance_.reset(new MrfTrackObjectDistance);
  MrfTrackObjectDistanceInitOptions distance_init_options;
  distance_init_options.config_path = options.config_path;
  ACHECK(track_object_distance_->Init(distance_init_options));

  bound_value_ = config.bound_value();
  max_match_distance_ = config.max_match_distance();
  return true;
}

void MrfTrackObjectMatcher::Match(
    const MrfTrackObjectMatcherOptions &options,
    const std::vector<TrackedObjectPtr> &objects,
    const std::vector<MrfTrackDataPtr> &tracks,
    std::vector<std::pair<size_t, size_t>> *assignments,
    std::vector<size_t> *unassigned_tracks,
    std::vector<size_t> *unassigned_objects) {
  assignments->clear();
  unassigned_objects->clear();
  unassigned_tracks->clear();
  if (objects.empty() || tracks.empty()) {
    unassigned_objects->resize(objects.size());
    unassigned_tracks->resize(tracks.size());
    std::iota(unassigned_objects->begin(), unassigned_objects->end(), 0);
    std::iota(unassigned_tracks->begin(), unassigned_tracks->end(), 0);
    return;
  }

  BipartiteGraphMatcherOptions matcher_options;
  matcher_options.cost_thresh = max_match_distance_;
  matcher_options.bound_value = bound_value_;

  BaseBipartiteGraphMatcher *matcher =
      objects[0]->is_background ? background_matcher_ : foreground_matcher_;

  algorithm::SecureMat<float> *association_mat = matcher->cost_matrix();

  association_mat->Resize(tracks.size(), objects.size());
  ComputeAssociateMatrix(tracks, objects, association_mat);
  matcher->Match(matcher_options, assignments, unassigned_tracks,
                 unassigned_objects);
  for (size_t i = 0; i < assignments->size(); ++i) {
    objects[assignments->at(i).second]->association_score =
        (*association_mat)(assignments->at(i).first,
                           assignments->at(i).second) /
        max_match_distance_;
  }
}

void MrfTrackObjectMatcher::ComputeAssociateMatrix(
    const std::vector<MrfTrackDataPtr> &tracks,
    const std::vector<TrackedObjectPtr> &new_objects,
    algorithm::SecureMat<float> *association_mat) {
  for (size_t i = 0; i < tracks.size(); ++i) {
    for (size_t j = 0; j < new_objects.size(); ++j) {
      (*association_mat)(i, j) =
          track_object_distance_->ComputeDistance(new_objects[j], tracks[i]);
    }
  }
}

}  // namespace radar4d
}  // namespace perception
}  // namespace apollo
