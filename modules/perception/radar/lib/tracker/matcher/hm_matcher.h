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
#pragma once

#include <vector>

#include "modules/perception/base/frame.h"
#include "modules/perception/common/graph/gated_hungarian_bigraph_matcher.h"
#include "modules/perception/lib/registerer/registerer.h"
#include "modules/perception/radar/lib/interface/base_matcher.h"

namespace apollo {
namespace perception {
namespace radar {

class HMMatcher : public BaseMatcher {
 public:
  HMMatcher();
  virtual ~HMMatcher();
  bool Init() override;
  // @brief match radar objects to tracks
  // @params[IN] radar_tracks: global tracks
  // @params[IN] radar_frame: current radar frame
  // @params[IN] options: matcher options for future use
  // @params[OUT] assignments: matched pair of tracks and measurements
  // @params[OUT] unassigned_tracks: unmatched tracks
  // @params[OUT] unassigned_objects: unmatched objects
  // @return nothing
  bool Match(const std::vector<RadarTrackPtr> &radar_tracks,
             const base::Frame &radar_frame,
             const TrackObjectMatcherOptions &options,
             std::vector<TrackObjectPair> *assignments,
             std::vector<size_t> *unassigned_tracks,
             std::vector<size_t> *unassigned_objects) override;

 protected:
  bool RefinedTrack(const base::ObjectPtr &track_object, double track_timestamp,
                    const base::ObjectPtr &radar_object,
                    double radar_timestamp) override;

 private:
  common::GatedHungarianMatcher<double> hungarian_matcher_;
  void TrackObjectPropertyMatch(const std::vector<RadarTrackPtr> &radar_tracks,
                                const base::Frame &radar_frame,
                                std::vector<TrackObjectPair> *assignments,
                                std::vector<size_t> *unassigned_tracks,
                                std::vector<size_t> *unassigned_objects);
  void ComputeAssociationMat(
      const std::vector<RadarTrackPtr> &radar_tracks,
      const base::Frame &radar_frame,
      const std::vector<size_t> &unassigned_tracks,
      const std::vector<size_t> &unassigned_objects,
      std::vector<std::vector<double> > *association_mat);
  double DistanceBetweenObs(const base::ObjectPtr &obs1, double timestamp1,
                            const base::ObjectPtr &obs2, double timestamp2);
};

}  // namespace radar
}  // namespace perception
}  // namespace apollo
