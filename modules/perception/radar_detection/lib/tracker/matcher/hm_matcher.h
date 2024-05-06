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

#include <string>
#include <vector>

#include "modules/perception/common/algorithm/graph/gated_hungarian_bigraph_matcher.h"
#include "modules/perception/common/base/frame.h"
#include "modules/perception/common/lib/registerer/registerer.h"
#include "modules/perception/radar_detection/interface/base_matcher.h"

namespace apollo {
namespace perception {
namespace radar {

class HMMatcher : public BaseMatcher {
 public:
  HMMatcher() = default;
  virtual ~HMMatcher() = default;

  /**
   * @brief Init HMMatcher config
   *
   * @param options init options
   * @return true
   * @return false
   */
  bool Init(const MatcherInitOptions &options) override;

  /**
   * @brief match radar objects to tracks
   *
   * @param radar_tracks global tracks
   * @param radar_frame current radar frame
   * @param options matcher options for future use
   * @param assignments matched pair of tracks and measurements
   * @param unassigned_tracks unmatched tracks
   * @param unassigned_objects unmatched objects
   * @return true
   * @return false
   */
  bool Match(const std::vector<RadarTrackPtr> &radar_tracks,
             const base::Frame &radar_frame,
             const TrackObjectMatcherOptions &options,
             std::vector<TrackObjectPair> *assignments,
             std::vector<size_t> *unassigned_tracks,
             std::vector<size_t> *unassigned_objects) override;

  /**
   * @brief The name of the HMMatcher
   *
   * @return std::string
   */
  std::string Name() const override { return "HMMatcher"; }

 protected:
  /**
   * @brief After the IDMatch, then track targets based on their features.
   *
   * @param track_object tracked targets
   * @param track_timestamp target tracked timestamp
   * @param radar_object radar detection objects
   * @param radar_timestamp radar detection timestamp
   * @return true
   * @return false
   */
  bool RefinedTrack(const base::ObjectPtr &track_object, double track_timestamp,
                    const base::ObjectPtr &radar_object,
                    double radar_timestamp) override;

 private:
  algorithm::GatedHungarianMatcher<double> hungarian_matcher_;
  void TrackObjectPropertyMatch(const std::vector<RadarTrackPtr> &radar_tracks,
                                const base::Frame &radar_frame,
                                std::vector<TrackObjectPair> *assignments,
                                std::vector<size_t> *unassigned_tracks,
                                std::vector<size_t> *unassigned_objects);
  void ComputeAssociationMat(const std::vector<RadarTrackPtr> &radar_tracks,
                             const base::Frame &radar_frame,
                             const std::vector<size_t> &unassigned_tracks,
                             const std::vector<size_t> &unassigned_objects,
                             std::vector<std::vector<double>> *association_mat);
  double DistanceBetweenObs(const base::ObjectPtr &obs1, double timestamp1,
                            const base::ObjectPtr &obs2, double timestamp2);
};

}  // namespace radar
}  // namespace perception
}  // namespace apollo
