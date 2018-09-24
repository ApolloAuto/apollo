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
#ifndef PERCEPTION_LIDAR_LIB_TRACKER_HM_TRACKER_KALMAN_FILTER_H_
#define PERCEPTION_LIDAR_LIB_TRACKER_HM_TRACKER_KALMAN_FILTER_H_

#include <string>

#include <Eigen/Core>

#include "modules/perception/lib/registerer/registerer.h"
#include "modules/perception/lidar/lib/tracker/common/track_data.h"
#include "modules/perception/lidar/lib/tracker/hm_tracker/base_filter.h"

namespace apollo {
namespace perception {
namespace lidar {

class KalmanFilter : public BaseFilter {
 public:
  KalmanFilter();

  ~KalmanFilter() = default;

  static void SetUseAdaptive(bool use_adaptive);

  static void SetUseConvergenceBoostup(bool use_convergence_boostup);

  static void SetConvergedConfidenceMinimum(
      double converged_confidence_minimum);

  static void SetParams(double centroid_measurement_noise,
                        double init_velocity_variance,
                        double propagation_variance_xy,
                        double propagation_variance_z);

  static void SetBoostupHistorySizeMinmax(size_t boostup_history_size_minimum);

  bool Init(const FilterOption& option) override;

  // @brief predict the state of filter
  // @params[IN] track_data: according track data to predict state at time
  // @params[IN] time: time for predicting
  // @return predicted states of filtering
  Eigen::VectorXd Predict(TrackDataPtr track_data, double time) override;

  // @brief update filter with object
  // @params[IN] track_data: track to update new object belief info
  // @params[IN OUT] new_object: new object for current updating
  // @params[IN] time: new object time
  // @return nothing
  void UpdateWithObject(TrackDataPtr track_data, TrackedObjectPtr new_object,
                        double time) override;

  std::string name() const override { return "KalmanFilter"; }

 protected:
  // Temporary info for convenient one track update
  TrackDataPtr track_data_;
  TrackedObjectPtr new_object_;
  double new_object_time_;
  TrackedObjectPtr latest_object_;
  double latest_object_time_;
  double new_latest_time_diff_;
  bool temporal_info_filled_;

  // noise maximum
  static double s_noise_maximum_;

  // adaptive
  static bool s_use_adaptive_;

  // parameters
  static double s_centroid_measurement_noise_;
  static double s_centroid_init_velocity_variance_;
  static double s_propagation_variance_xy_;
  static double s_propagation_variance_z_;
  static Eigen::Matrix3d s_covariance_propagation_uncertainty_;

  // convergence
  static bool s_use_convergence_boostup_;
  static size_t s_boostup_history_size_minimum_;
  static size_t s_boostup_history_size_maximum_;
  static double s_converged_confidence_minimum_;

  // motion score
  static int s_motion_score_window_size_;

 private:
  bool FillTemporaryInfo(TrackDataPtr track_data, TrackedObjectPtr new_object,
                         double time);

  void InitializeTrackedObject();

  void UpdateWithTrueObject();

  void UpdateWithFakeObject();

  void NewObjectPropagate();

  void ComputeNewObjectSelectMeasuredVelocity();

  void SelectMeasuredVelocityAccordingMotionConsistency();

  void ComputeNewObjectSelectMeasuredAcceleration();

  void ComputeNewObjectBeliefAnchorPoint();

  void ComputeNewObjectBeliefVelocityAndCov();

  void ComputeNewObjectBeliefAcceleration();

  void ComputeNewObjectUpdateQuality();

  float ComputeUpdateQualityByAssociationScore();

  float ComputeUpdateQualityByPointNumberDiff();

  float ComputeBreakdownThreshold();

  void ComputeNewObjectConvergenceAndBoostupBelief();

  void UpdateConverged(int useable_measure_velocity_size);

  void ComputeConvergenceConfidence(int useable_measure_velocity_size);

  void BoostupBelief(int useable_measure_velocity_size);

  void ClapingNewObjectBelief();

  void ComputeVelocityOnlineCovariance();

  void BeliefToOutput();

  void CalculateAverageCornerVelocity(int window_size,
                                      Eigen::Vector3d* average_velocity);

  void ComputeMotionScore();
};  // class KalmanFilter

typedef std::shared_ptr<KalmanFilter> KalmanFilterPtr;
typedef std::shared_ptr<const KalmanFilter> KalmanFilterConstPtr;

}  // namespace lidar
}  // namespace perception
}  // namespace apollo

#endif  // PERCEPTION_LIDAR_LIB_TRACKER_HM_TRACKER_KALMAN_FILTER_H_
