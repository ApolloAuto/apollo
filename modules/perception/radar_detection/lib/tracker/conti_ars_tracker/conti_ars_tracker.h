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

#include <memory>
#include <string>
#include <vector>

#include "cyber/common/macros.h"
#include "modules/perception/radar_detection/interface/base_tracker.h"
#include "modules/perception/radar_detection/lib/tracker/common/radar_track_manager.h"
#include "modules/perception/radar_detection/lib/tracker/matcher/hm_matcher.h"

namespace apollo {
namespace perception {
namespace radar {

class ContiArsTracker : public BaseTracker {
 public:
  ContiArsTracker();
  virtual ~ContiArsTracker();

  /**
   * @brief Init ContiArsTracker config
   *
   * @param options
   * @return true
   * @return false
   */
  bool Init(const TrackerInitOptions &options) override;

  /**
   * @brief Tracking objects.
   *
   * @param detected_frame current object frame.
   * @param options tracking options
   * @param tracked_frame current tracked objects frame.
   * @return true
   * @return false
   */
  bool Track(const base::Frame &detected_frame, const TrackerOptions &options,
             base::FramePtr tracked_frame) override;

 private:
  std::shared_ptr<BaseMatcher> matcher_ = nullptr;
  RadarTrackManager *track_manager_ = nullptr;
  static double s_tracking_time_win_;
  void TrackObjects(const base::Frame &radar_frame);
  void UpdateAssignedTracks(const base::Frame &radar_frame,
                            std::vector<TrackObjectPair> assignments);
  void UpdateUnassignedTracks(const base::Frame &radar_frame,
                              const std::vector<size_t> &unassigned_tracks);
  void DeleteLostTracks();
  void CreateNewTracks(const base::Frame &radar_frame,
                       const std::vector<size_t> &unassigned_objects);
  void CollectTrackedFrame(base::FramePtr tracked_frame);

  DISALLOW_COPY_AND_ASSIGN(ContiArsTracker);
};

}  // namespace radar
}  // namespace perception
}  // namespace apollo
