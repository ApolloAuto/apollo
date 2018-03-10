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

#ifndef MODULES_PERCEPTION_OBSTACLE_FUSION_ASYNC_FUSION_ASYNC_TRACK_H_  // NOLINT
#define MODULES_PERCEPTION_OBSTACLE_FUSION_ASYNC_FUSION_ASYNC_TRACK_H_  // NOLINT

#include <memory>
#include <vector>
#include <string>
#include "modules/common/macro.h"
#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_base_motion_fusion.h"
#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_sensor_object.h"
#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_track.h"

namespace apollo {
namespace perception {

class AsyncTrack {
 public:
  explicit AsyncTrack(PbfSensorObjectPtr obj);

  ~AsyncTrack();

  /**@brief Update track with sensor object */
  void UpdateWithSensorObject(PbfSensorObjectPtr obj, double match_dist);

  void UpdateWithoutSensorObject(const SensorType &sensor_type,
                                 const std::string &sensor_id,
                                 double min_match_dist, double timestamp);

  PbfSensorObjectPtr GetFusedObject();

  double GetFusedTimestamp() const;

  PbfSensorObjectPtr GetSensorObject(const SensorType &sensor_type,
                                     const std::string &sensor_id);

  int GetTrackId() const;

  double GetTrackingPeriod() const {
    return tracking_period_;
  }

  inline bool IsDead() const {
    return is_dead_;
  }
  bool AbleToPublish();

  static int GetNextTrackId();

  void SetMotionFusionMethod(const std::string motion_fusion_method);

 protected:
  void PerformMotionFusion(PbfSensorObjectPtr obj);

 protected:
  PbfSensorObjectPtr fused_object_;

  /**@brief time stamp of the track*/
  double fused_timestamp_;

  int age_;
  double tracking_period_;

  /**@brief global track id*/
  int idx_;
  double invisible_period_;

  /**@brief motion fusion*/
  std::shared_ptr<PbfBaseMotionFusion> motion_fusion_;

  bool is_dead_;

 private:
  AsyncTrack();

 private:
  static int s_track_idx_;
  std::string s_motion_fusion_method;

  FRIEND_TEST(AsyncTrackTest, test_pbf_track_constructor);
  FRIEND_TEST(AsyncTrackTest, test_pbf_get_object);
  FRIEND_TEST(AsyncTrackTest, test_pbf_update_measurements_life);
};

typedef std::shared_ptr<AsyncTrack> AsyncTrackPtr;

class AsyncTrackManager {
 public:
  static AsyncTrackManager *instance();
  ~AsyncTrackManager();

  inline std::vector<AsyncTrackPtr> &GetTracks() {
    return tracks_;
  }

  inline const std::vector<AsyncTrackPtr> &GetTracks() const {
    return tracks_;
  }

  void AddTrack(const AsyncTrackPtr &track) {
    tracks_.push_back(track);
  }

  int RemoveLostTracks();

 protected:
  std::vector<AsyncTrackPtr> tracks_;

 private:
  AsyncTrackManager();
  DISALLOW_COPY_AND_ASSIGN(AsyncTrackManager);
};

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_FUSION_ASYNC_FUSION_ASYNC_TRACK_H_
        // // NOLINT
