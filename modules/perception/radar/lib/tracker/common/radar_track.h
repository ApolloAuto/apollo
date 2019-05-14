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

#include "Eigen/Core"

#include "cyber/common/log.h"
#include "cyber/common/macros.h"

#include "modules/perception/base/frame.h"
#include "modules/perception/base/object_pool_types.h"
#include "modules/perception/radar/common/types.h"
#include "modules/perception/radar/lib/interface/base_filter.h"

namespace apollo {
namespace perception {
namespace radar {

class RadarTrack {
 public:
  RadarTrack(const base::ObjectPtr &obs, const double timestamp);
  ~RadarTrack() {}
  // update the object after association with a radar obervation
  void UpdataObsRadar(const base::ObjectPtr &obs_radar, const double timestamp);
  void SetObsRadarNullptr();
  int GetObsId() const;
  base::ObjectPtr GetObsRadar();
  base::ObjectPtr GetObs();
  double GetTimestamp();
  double GetTrackingTime();
  bool IsDead() { return is_dead_; }
  void SetDead() { is_dead_ = true; }
  bool ConfirmTrack() { return tracked_times_ > s_tracked_times_threshold_; }
  static void SetTrackedTimesThreshold(const int &threshold) {
    s_tracked_times_threshold_ = threshold;
  }
  static void SetChosenFilter(const std::string &chosen_filter) {
    s_chosen_filter_ = chosen_filter;
  }
  static void SetUseFilter(bool use_filter) { s_use_filter_ = use_filter; }

 private:
  double timestamp_ = 0.0;
  int obs_id_ = 0;
  int tracked_times_ = 0;
  double tracking_time_ = 0.0;
  bool is_dead_ = false;
  base::ObjectPtr obs_radar_ = nullptr;  // observasion from radar
  base::ObjectPtr obs_ = nullptr;        // track result after tracking
  std::shared_ptr<BaseFilter> filter_ = nullptr;

  static std::string s_chosen_filter_;
  static int s_current_idx_;
  static int s_tracked_times_threshold_;
  static bool s_use_filter_;

  DISALLOW_COPY_AND_ASSIGN(RadarTrack);
};

typedef std::shared_ptr<RadarTrack> RadarTrackPtr;

}  // namespace radar
}  // namespace perception
}  // namespace apollo
