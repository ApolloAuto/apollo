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
#pragma once

#include <memory>
#include <string>

#include "modules/perception/lidar/lib/tracker/multi_lidar_fusion/mlf_base_filter.h"
#include "modules/perception/lidar/lib/tracker/multi_lidar_fusion/mlf_motion_measurement.h"
#include "modules/perception/lidar/lib/tracker/multi_lidar_fusion/mlf_motion_refiner.h"

namespace apollo {
namespace perception {
namespace lidar {

class MlfMotionFilter : public MlfBaseFilter {
 public:
  MlfMotionFilter() = default;
  virtual ~MlfMotionFilter() = default;

  bool Init(
      const MlfFilterInitOptions& options = MlfFilterInitOptions()) override;

  // @brief: updating motion filter with object
  // @params [in]: options for updating
  // @params [in]: track data, not include new object
  // @params [in/out]: new object for updating
  void UpdateWithObject(const MlfFilterOptions& options,
                        const MlfTrackDataConstPtr& track_data,
                        TrackedObjectPtr new_object) override;

  // @brief: updating motion filter without object
  // @params [in]: options for updating
  // @params [in]: current timestamp
  // @params [in/out]: track data to be updated
  void UpdateWithoutObject(const MlfFilterOptions& options, double timestamp,
                           MlfTrackDataPtr track_data) override;

  std::string Name() const override { return "MlfMotionFilter"; }

 protected:
  // @brief: initialize track state and store in the new object
  // @params [out]: new object to be updated
  void InitializeTrackState(TrackedObjectPtr new_object);

  // @brief: update state with kalman filter,
  //         constant acceleration motion model,
  //         only velocity measurement is observed
  // @params [in]: history track data
  // @params [in]: latest object in the track data
  // @params [in/out]: new object to be updated
  void KalmanFilterUpdateWithPartialObservation(
      const MlfTrackDataConstPtr& track_data,
      const TrackedObjectConstPtr& latest_object, TrackedObjectPtr new_object);

  // @brief: adjust kalman state gain with several strategies
  // @params [in]: history track data
  // @params [in]: latest object in the track data
  // @params [in]: new object to be updated
  // @params [in/out]: state gain
  void StateGainAdjustment(const MlfTrackDataConstPtr& track_data,
                           const TrackedObjectConstPtr& latest_object,
                           const TrackedObjectConstPtr& new_object,
                           Eigen::Vector4d* gain);

  // @brief: estimate convergence confidence and boost up state
  // @params [in]: history track data
  // @params [in]: latest object in the track data
  // @params [in/out]: new object to be updated
  void ConvergenceEstimationAndBoostUp(
      const MlfTrackDataConstPtr& track_data,
      const TrackedObjectConstPtr& latest_object, TrackedObjectPtr new_object);

  // @brief: compute convergence confidence
  // @params [in]: history track data
  // @params [in/out]: new object to be updated
  // @params [in]: whether using belief velocity or output velocity
  void ComputeConvergenceConfidence(const MlfTrackDataConstPtr& track_data,
                                    TrackedObjectPtr new_object,
                                    bool velocity_source_is_belief = true);

  // @brief: boost up state considering track history
  // @params [in]: history track data
  // @params [in/out]: new object to be updated
  void BoostupState(const MlfTrackDataConstPtr& track_data,
                    TrackedObjectPtr new_object);

  // @brief: cliping state if is within noise level
  // @params [in/out]: new object to be updated
  void ClipingState(TrackedObjectPtr object);

  // @brief: estimate covariance considering history measurment
  // @params [in]: history track data
  // @params [in/out]: new object to be updated
  void OnlineCovarianceEstimation(const MlfTrackDataConstPtr& track_data,
                                  TrackedObjectPtr object);

  // @brief: update convergence confidence
  // @params [in]: history track data
  // @params [in/out]: new object to be updated
  void UpdateConverged(const MlfTrackDataConstPtr& track_data,
                       TrackedObjectPtr object);

  // @brief: synchronize state to belief to keep consistency
  // @params [in/out]: new object to be updated
  void StateToBelief(TrackedObjectPtr object);

  // @brief: copy belief to output
  // @params [in/out]: new object to be updated
  void BeliefToOutput(TrackedObjectPtr object);

 protected:
  const double EPSION_TIME = 1e-3;
  const double DEFAULT_FPS = 0.1;
  // motion measurement
  std::shared_ptr<MlfMotionMeasurement> motion_measurer_;
  // motion refiner
  std::shared_ptr<MlfMotionRefiner> motion_refiner_;
  // switch for filter strategies
  bool use_adaptive_ = true;
  bool use_breakdown_ = true;
  bool use_convergence_boostup_ = true;
  // default covariance parameters for kalman filter
  double init_velocity_variance_ = 5.0;
  double init_acceleration_variance_ = 10.0;
  double measured_velocity_variance_ = 0.4;
  double predict_variance_per_sqrsec_ = 50.0;
  // other parameters
  size_t boostup_history_size_minimum_ = 3;
  size_t boostup_history_size_maximum_ = 6;
  double converged_confidence_minimum_ = 0.5;
  double noise_maximum_ = 0.1;
  double trust_orientation_range_ = 40.0;
};  // class MlfMotionFilter

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
