/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#ifndef MODULES_PERCEPTION_OBSTACLE_FUSION_PROBABILISTIC_FUSION_PBF_BASE_TRACK_OBJECT_MATCHER_H_  // NOLINT
#define MODULES_PERCEPTION_OBSTACLE_FUSION_PROBABILISTIC_FUSION_PBF_BASE_TRACK_OBJECT_MATCHER_H_  // NOLINT

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "modules/common/macro.h"
#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_sensor_object.h"
#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_track.h"

namespace apollo {
namespace perception {

struct TrackObjectMatcherOptions {
  const Eigen::Vector3d *ref_point = nullptr;
};

class PbfBaseTrackObjectMatcher {
 public:
  PbfBaseTrackObjectMatcher() {}
  virtual ~PbfBaseTrackObjectMatcher() = default;

  // @brief match sensor objects to global tracks build previously
  // @params[IN] fusion_tracks: global tracks
  // @params[IN] sensor_objects: sensor objects
  // @params[IN] options: matcher options for future use
  // @params[OUT] assignments: matched pair of tracks and measurements
  // @params[OUT] unassigned_tracks: unmatched tracks
  // @params[OUT] unassigned_objects: unmatched objects
  // @params[OUT] track2measurements_dist:minimum match distance to measurements
  // for each track
  // @prams[OUT] measurement2track_dist:minimum match distacne to tracks for
  // each measurement
  // @return nothing
  virtual bool Match(
      const std::vector<PbfTrackPtr> &fusion_tracks,
      const std::vector<std::shared_ptr<PbfSensorObject>> &sensor_objects,
      const TrackObjectMatcherOptions &options,
      std::vector<std::pair<int, int>> *assignments,
      std::vector<int> *unassigned_fusion_tracks,
      std::vector<int> *unassigned_sensor_tracks,
      std::vector<double> *track2measurements_dist,
      std::vector<double> *measurement2track_dist) = 0;

  virtual bool Init() = 0;

  virtual std::string name() const = 0;

  void IdAssign(
      const std::vector<PbfTrackPtr> &fusion_tracks,
      const std::vector<std::shared_ptr<PbfSensorObject>> &sensor_objects,
      std::vector<std::pair<int, int>> *assignments,
      std::vector<int> *unassigned_fusion_tracks,
      std::vector<int> *unassigned_sensor_objects);

  static void SetMaxMatchDistance(const double dist);

  static double GetMaxMatchDistance();

 protected:
  static double s_max_match_distance_;

 private:
  DISALLOW_COPY_AND_ASSIGN(PbfBaseTrackObjectMatcher);
};

}  // namespace perception
}  // namespace apollo

// clang-format off
#endif  // MODULES_PERCEPTION_OBSTACLE_FUSION_PROBABILISTIC_FUSION_PBF_BASE_TRACK_OBJECT_MATCHER_H_ // NOLINT
// clang-format on
