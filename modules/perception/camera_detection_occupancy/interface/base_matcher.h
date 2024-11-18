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
#pragma once

#include <string>
#include <utility>
#include <vector>

#include "gtest/gtest_prod.h"

#include "cyber/common/macros.h"
#include "modules/perception/camera_detection_occupancy/tracker/common/camera_track.h"
#include "modules/perception/common/base/frame.h"
#include "modules/perception/common/lib/interface/base_init_options.h"
#include "modules/perception/common/lib/registerer/registerer.h"

namespace apollo {
namespace perception {
namespace camera {

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
   * @brief Match camera objects to tracks
   *
   * @param camera_tracks global tracks
   * @param camera_frame current camera frame
   * @param options matcher options for future use
   * @param assignments matched pair of tracks and measurements
   * @param unassigned_tracks unmatched tracks
   * @param unassigned_objects unmatched objects
   * @return true
   * @return false
   */
  virtual bool Match(const std::vector<CameraTrackPtr> &camera_tracks,
                     const base::Frame &camera_frame,
                     const TrackObjectMatcherOptions &options,
                     std::vector<TrackObjectPair> *assignments,
                     std::vector<size_t> *unassigned_tracks,
                     std::vector<size_t> *unassigned_objects) {
    return true;
  }

  /**
   * @brief The name of the camera base Matcher
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
   * @brief track targets based on their features.
   *
   * @param track_object tracked targets
   * @param track_timestamp target tracked timestamp
   * @param camera_object camera detection objects
   * @param camera_timestamp camera detection timestamp
   * @return true
   * @return false
   */
  virtual bool RefinedTrack(const base::ObjectPtr &track_object,
                            double track_timestamp,
                            const base::ObjectPtr &camera_object,
                            double camera_timestamp);

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

}  // namespace camera
}  // namespace perception
}  // namespace apollo
