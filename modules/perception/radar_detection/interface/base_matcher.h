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
#include <utility>
#include <vector>

#include "gtest/gtest_prod.h"

#include "cyber/common/macros.h"
#include "modules/perception/common/base/frame.h"
#include "modules/perception/common/lib/interface/base_init_options.h"
#include "modules/perception/common/lib/registerer/registerer.h"
#include "modules/perception/radar_detection/lib/tracker/common/radar_track.h"

namespace apollo {
namespace perception {
namespace radar {

typedef std::pair<size_t, size_t> TrackObjectPair;

struct MatcherInitOptions : public BaseInitOptions {
  // reserved
};

struct TrackObjectMatcherOptions {
  Eigen::Vector3d *ref_point = nullptr;
};

class BaseMatcher {
 public:
  /**
   * @brief Construct a new Base Matcher object
   *
   */
  BaseMatcher() = default;
  virtual ~BaseMatcher() = default;

  /**
   * @brief Init Base Matcher config
   *
   * @param options init options
   * @return true
   * @return false
   */
  virtual bool Init(const MatcherInitOptions &options) { return true; }

  /**
   * @brief Match radar objects to tracks
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
  virtual bool Match(const std::vector<RadarTrackPtr> &radar_tracks,
                     const base::Frame &radar_frame,
                     const TrackObjectMatcherOptions &options,
                     std::vector<TrackObjectPair> *assignments,
                     std::vector<size_t> *unassigned_tracks,
                     std::vector<size_t> *unassigned_objects) {
    return true;
  }

  /**
   * @brief Target association by id. The radar driver already has the id of
   * the target, so we choose to trust it and do this part of the match first.
   * We match the detection targets in current frame with the targets has been
   * tracked in the past.
   *
   * @param radar_tracks targets tracking by radar driver
   * @param radar_frame radar frame
   * @param assignments track and object pairs already matched
   * @param unassigned_tracks tracks not matched
   * @param unassigned_objects objects not matched
   */
  virtual void IDMatch(const std::vector<RadarTrackPtr> &radar_tracks,
                       const base::Frame &radar_frame,
                       std::vector<TrackObjectPair> *assignments,
                       std::vector<size_t> *unassigned_tracks,
                       std::vector<size_t> *unassigned_objects);

  /**
   * @brief The name of the radar base Matcher
   *
   * @return std::string
   */
  virtual std::string Name() const { return "BaseMatcher"; }

  /**
   * @brief Set the Max Match Distance
   *
   * @param dist
   */
  static void SetMaxMatchDistance(double dist);

  /**
   * @brief Get the Max Match Distance
   *
   * @return double
   */
  static double GetMaxMatchDistance();

  /**
   * @brief Set the Bound Match Distance
   *
   * @param dist
   */
  static void SetBoundMatchDistance(double dist);

  /**
   * @brief Get the Bound Match Distance
   *
   * @return double
   */
  static double GetBoundMatchDistance();

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
  virtual bool RefinedTrack(const base::ObjectPtr &track_object,
                            double track_timestamp,
                            const base::ObjectPtr &radar_object,
                            double radar_timestamp);

 protected:
  static double s_max_match_distance_;
  static double s_bound_match_distance_;
  FRIEND_TEST(BaseMatcherTest, base_matcher_test);

 private:
  DISALLOW_COPY_AND_ASSIGN(BaseMatcher);
};

PERCEPTION_REGISTER_REGISTERER(BaseMatcher);
#define PERCEPTION_REGISTER_MATCHER(name) \
  PERCEPTION_REGISTER_CLASS(BaseMatcher, name)

}  // namespace radar
}  // namespace perception
}  // namespace apollo
