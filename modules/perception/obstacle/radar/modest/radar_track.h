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

#ifndef MODULES_PERCEPTION_OBSTACLE_RADAR_MODEST_RADAR_TRACK_H_
#define MODULES_PERCEPTION_OBSTACLE_RADAR_MODEST_RADAR_TRACK_H_

#include <memory>

#include "Eigen/Core"

#include "modules/common/log.h"
#include "modules/perception/obstacle/base/object.h"
#include "modules/perception/obstacle/base/types.h"
#include "modules/perception/obstacle/radar/modest/radar_define.h"

namespace apollo {
namespace perception {

class RadarTrack {
 public:
  RadarTrack();

  RadarTrack(const Object &obs, const double &timestamp);

  RadarTrack(const RadarTrack &track);

  RadarTrack &operator=(const RadarTrack &track);

  ~RadarTrack() {}

  // update the object after association with a radar obervation
  void UpdataObsRadar(std::shared_ptr<Object> obs_radar,
                      const double timestamp);

  void SetObsRadar(std::shared_ptr<Object> obs_radar);

  void IncreaseTrackedTimes();

  int GetObsId() const;

  std::shared_ptr<Object> GetObsRadar();

  double GetTimestamp();

  double GetTrackingTime();

  static void SetTrackedTimesThreshold(const int &threshold) {
    s_tracked_times_threshold_ = threshold;
  }

 private:
  static int s_current_idx_;
  static int s_tracked_times_threshold_;
  int obs_id_;
  double timestamp_;
  std::shared_ptr<Object> obs_radar_;  // observation from radar
  int tracked_times_;
  double tracking_time_;
  bool id_tracked_;
};

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_RADAR_MODEST_RADAR_TRACK_H_
