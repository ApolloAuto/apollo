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
#pragma once

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "cyber/common/macros.h"
#include "modules/perception/common/graph/secure_matrix.h"
#include "modules/perception/lidar/lib/interface/base_bipartite_graph_matcher.h"
#include "modules/perception/lidar/lib/tracker/multi_lidar_fusion/mlf_track_object_distance.h"

namespace apollo {
namespace perception {
namespace lidar {

struct MlfTrackObjectMatcherInitOptions {};

struct MlfTrackObjectMatcherOptions {};

class MlfTrackObjectMatcher {
 public:
  MlfTrackObjectMatcher() = default;
  ~MlfTrackObjectMatcher() = default;

  bool Init(const MlfTrackObjectMatcherInitOptions &options =
                MlfTrackObjectMatcherInitOptions());

  // @brief: match detected objects to tracks
  // @params [in]: new detected objects for matching
  // @params [in]: maintaining tracks for matching
  // @params [out]: assignment pair of object & track
  // @params [out]: tracks without matched object
  // @params [out]: objects without matched track
  void Match(const MlfTrackObjectMatcherOptions &options,
             const std::vector<TrackedObjectPtr> &objects,
             const std::vector<MlfTrackDataPtr> &tracks,
             std::vector<std::pair<size_t, size_t> > *assignments,
             std::vector<size_t> *unassigned_tracks,
             std::vector<size_t> *unassigned_objects);

  std::string Name() const { return "MlfTrackObjectMatcher"; }

 protected:
  // @brief: compute association matrix
  // @params [in]: maintained tracks for matching
  // @params [in]: new detected objects for matching
  // @params [out]: matrix of association distance
  void ComputeAssociateMatrix(const std::vector<MlfTrackDataPtr> &tracks,
                              const std::vector<TrackedObjectPtr> &new_objects,
                              common::SecureMat<float> *association_mat);

 protected:
  std::unique_ptr<MlfTrackObjectDistance> track_object_distance_;
  std::unique_ptr<BaseBipartiteGraphMatcher> foreground_matcher_;
  std::unique_ptr<BaseBipartiteGraphMatcher> background_matcher_;

  float bound_value_ = 100.f;
  float max_match_distance_ = 4.0f;

 private:
  DISALLOW_COPY_AND_ASSIGN(MlfTrackObjectMatcher);
};  // class MlfTrackObjectMatcher

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
