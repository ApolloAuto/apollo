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
#ifndef PERCEPTION_LIDAR_LIB_TRACKER_HM_TRACKER_TRACK_POST_PROCESSOR_H_
#define PERCEPTION_LIDAR_LIB_TRACKER_HM_TRACKER_TRACK_POST_PROCESSOR_H_

#include "modules/perception/lidar/lib/tracker/common/track_data.h"

namespace apollo {
namespace perception {
namespace lidar {

class TrackPostProcessor {
 public:
  TrackPostProcessor() = default;

  ~TrackPostProcessor() = default;

  void PostProcess(TrackDataPtr track_data);

 protected:
  void SmoothTrackVelocity(TrackDataPtr track_data);
  void SmoothTrackOrientation(TrackDataPtr track_data);

  bool CheckStaticHypothesisByState(TrackDataPtr track_data);
  bool CheckStaticHypothesisByVelocityAngleChange(
      TrackDataPtr track_data, const double& reasonable_angle_change_maximum);

  bool CheckStaticHypothesisByMotionScore(TrackDataPtr track_data);

  bool CheckVelocityConsistency(TrackDataPtr track_data);

  double WelshVarLoss(double dist, double th, double scale);
  double WelshBoundLoss(double dist, double lower, double upper);
  double FusedMultipleProb(const std::vector<double>& probs);

  void UpdateTrackConvergenceState(TrackDataPtr track_data);
  void UpdateTrackVelocityOnlineCovariance(TrackDataPtr track_data);

 protected:
  static double s_converged_confidence_minimum_;
  static double s_centroid_measurement_noise_;
  static double s_propagation_variance_xy_;
  static double s_noise_maximum_;
  static constexpr double s_claping_acceleration_threshold_ = 10;
  static constexpr double s_claping_speed_threshold_ = 1.0;
};

typedef std::shared_ptr<TrackPostProcessor> TrackPostProcessorPtr;
typedef std::shared_ptr<const TrackPostProcessor> TrackPostProcessorConstPtr;

}  // namespace lidar
}  // namespace perception
}  // namespace apollo

#endif  // PERCEPTION_LIDAR_LIB_TRACKER_HM_TRACKER_TRACK_POST_PROCESSOR_H_
