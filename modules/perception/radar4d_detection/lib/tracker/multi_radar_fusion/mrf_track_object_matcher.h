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
#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "cyber/common/macros.h"
#include "modules/perception/common/algorithm/graph/secure_matrix.h"
#include "modules/perception/common/lib/interface/base_init_options.h"
#include "modules/perception/radar4d_detection/interface/base_bipartite_graph_matcher.h"
#include "modules/perception/radar4d_detection/lib/tracker/multi_radar_fusion/mrf_track_object_distance.h"

namespace apollo {
namespace perception {
namespace radar4d {

using apollo::perception::BaseInitOptions;

struct MrfTrackObjectMatcherInitOptions : public BaseInitOptions {};

struct MrfTrackObjectMatcherOptions {};

class MrfTrackObjectMatcher {
 public:
  MrfTrackObjectMatcher() = default;
  ~MrfTrackObjectMatcher() = default;

  /**
   * @brief Initalize mrf track object matcher
   *
   * @param options
   * @return true
   * @return false
   */
  bool Init(const MrfTrackObjectMatcherInitOptions &options =
                MrfTrackObjectMatcherInitOptions());

  /**
   * @brief Match detected objects to tracks
   *
   * @param options
   * @param objects new detected objects for matching
   * @param tracks maintaining tracks for matching
   * @param assignments assignment pair of object & track
   * @param unassigned_tracks racks without matched object
   * @param unassigned_objects objects without matched track
   */
  void Match(const MrfTrackObjectMatcherOptions &options,
             const std::vector<TrackedObjectPtr> &objects,
             const std::vector<MrfTrackDataPtr> &tracks,
             std::vector<std::pair<size_t, size_t>> *assignments,
             std::vector<size_t> *unassigned_tracks,
             std::vector<size_t> *unassigned_objects);

  /**
   * @brief Get class name
   *
   * @return std::string
   */
  std::string Name() const { return "MrfTrackObjectMatcher"; }

 protected:
  /**
   * @brief Compute association matrix
   *
   * @param tracks maintained tracks for matching
   * @param new_objects new detected objects for matching
   * @param association_mat matrix of association distance
   */
  void ComputeAssociateMatrix(const std::vector<MrfTrackDataPtr> &tracks,
                              const std::vector<TrackedObjectPtr> &new_objects,
                              algorithm::SecureMat<float> *association_mat);

 protected:
  std::unique_ptr<MrfTrackObjectDistance> track_object_distance_;
  BaseBipartiteGraphMatcher *foreground_matcher_;
  BaseBipartiteGraphMatcher *background_matcher_;

  float bound_value_ = 100.f;
  float max_match_distance_ = 4.0f;
  bool use_semantic_map = false;

 private:
  DISALLOW_COPY_AND_ASSIGN(MrfTrackObjectMatcher);
};  // class MrfTrackObjectMatcher

}  // namespace radar4d
}  // namespace perception
}  // namespace apollo
