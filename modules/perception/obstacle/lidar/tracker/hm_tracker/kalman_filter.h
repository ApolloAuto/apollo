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

#ifndef MODULES_PERCEPTION_OBSTACLE_LIDAR_TRACKER_HM_TRACKER_KALMAN_FILTER_H_
#define MODULES_PERCEPTION_OBSTACLE_LIDAR_TRACKER_HM_TRACKER_KALMAN_FILTER_H_

#include <deque>
#include <memory>
#include <vector>

#include "modules/perception/obstacle/base/object.h"
#include "modules/perception/obstacle/lidar/tracker/hm_tracker/base_filter.h"

namespace apollo {
namespace perception {

class KalmanFilter : public BaseFilter {
 public:
  KalmanFilter();
  ~KalmanFilter() {}

  // @brief set use adaptive for all the filter objects
  // @params[IN] use_adaptive: flag of whether use adaptive version or not
  // @return nothing
  static void SetUseAdaptive(const bool& use_adaptive);

  // @brief set association score maximum for computing update qaulity
  // @params[IN] association_score_maximum: association score maximum
  // @return true if set successfully, otherwise return false
  static bool SetAssociationScoreMaximum(
      const double& association_score_maximum);

  // @brief set breakdown threshold maximum for computing breakdown ratio
  // @params[IN] breakdown_threshold_maximum: breakdown threshold maximum
  // @return true if set successfully, otherwise return false
  static bool SetBreakdownThresholdMaximum(
      const double& breakdown_threshold_maximum);

  // @brief init initialize parameters for kalman filter
  // @params[IN] measurement_noise: noise of measurement
  // @params[IN] initial_velocity_noise: initial uncertainty of velocity
  // @params[IN] xy_propagation_noise: propagation uncertainty of xy
  // @params[IN] z_propagation_noise: propagation uncertainty of z
  // @return true if set successfully, otherwise return false
  static bool InitParams(const double& measurement_noise,
                         const double& initial_velocity_noise,
                         const double& xy_propagation_noise,
                         const double& z_propagation_noise);

  // @brief initialize the state of filter
  // @params[IN] anchor_point: initial anchor point for filtering
  // @params[IN] velocity: initial velocity for filtering
  // @return nothing
  void Initialize(const Eigen::Vector3f& anchor_point,
                  const Eigen::Vector3f& velocity);

  // @brief predict the state of filter
  // @params[IN] time_diff: time interval for predicting
  // @return predicted states of filtering
  Eigen::VectorXf Predict(const double& time_diff);

  // @brief update filter with object
  // @params[IN] new_object: new object for current updating
  // @params[IN] old_object: old object for last updating
  // @params[IN] time_diff: time interval from last updating
  // @return nothing
  void UpdateWithObject(const std::shared_ptr<TrackedObject>& new_object,
                        const std::shared_ptr<TrackedObject>& old_object,
                        const double& time_diff);

  // @brief update filter without object
  // @params[IN] time_diff: time interval from last updating
  // @return nothing
  void UpdateWithoutObject(const double& time_diff);

  // @brief get state of filter
  // @params[OUT] anchor_point: anchor point of current state
  // @params[OUT] velocity: velocity of current state
  // @return nothing
  void GetState(Eigen::Vector3f* anchor_point, Eigen::Vector3f* velocity);

  // @brief get state of filter with accelaration
  // @params[OUT] anchor_point: anchor point of current state
  // @params[OUT] velocity: velocity of current state
  // @params[OUT] velocity_accelaration: accelaration of current state
  // @return nothing
  void GetState(Eigen::Vector3f* anchor_point, Eigen::Vector3f* velocity,
                Eigen::Vector3f* accelaration);

  void GetAccelerationGain(Eigen::Vector3f* acceleration_gain);

 protected:
  // @brief propagate covariance of filter
  // @params[IN] time_diff: time interval from last updating
  // @return nothing
  void Propagate(const double& time_diff);

  // @brief compute measured velocity
  // @params[IN] new_object: new object for current updating
  // @params[IN] old_object: old object for last updating
  // @params[IN] time_diff: time interval from last updating
  // @return measured velocity
  Eigen::VectorXf ComputeMeasuredVelocity(
      const std::shared_ptr<TrackedObject>& new_object,
      const std::shared_ptr<TrackedObject>& old_object,
      const double& time_diff);

  // @brief compute measured anchor point velocity
  // @params[IN] new_object: new object for current updating
  // @params[IN] old_object: old object for last updating
  // @params[IN] time_diff: time interval from last updating
  // @return measured anchor point elocity
  Eigen::VectorXf ComputeMeasuredAnchorPointVelocity(
      const std::shared_ptr<TrackedObject>& new_object,
      const std::shared_ptr<TrackedObject>& old_object,
      const double& time_diff);

  // @brief compute measured bbox center velocity
  // @params[IN] new_object: new object for current updating
  // @params[IN] old_object: old object for last updating
  // @params[IN] time_diff: time interval from last updating
  // @return measured bbox center velocity
  Eigen::VectorXf ComputeMeasuredBboxCenterVelocity(
      const std::shared_ptr<TrackedObject>& new_object,
      const std::shared_ptr<TrackedObject>& old_object,
      const double& time_diff);

  // @brief compute measured bbox corner velocity
  // @params[IN] new_object: new object for current updating
  // @params[IN] old_object: old object for last updating
  // @params[IN] time_diff: time interval from last updating
  // @return measured bbox corner velocity
  Eigen::VectorXf ComputeMeasuredBboxCornerVelocity(
      const std::shared_ptr<TrackedObject>& new_object,
      const std::shared_ptr<TrackedObject>& old_object,
      const double& time_diff);

  // @brief select measured velocity among candidates
  // @params[IN] candidates: candidates of measured velocity
  // @return selected measurement of velocity
  Eigen::Vector3f SelectMeasuredVelocity(
      const std::vector<Eigen::Vector3f>& candidates);

  // @brief select measured velocity among candidates according motion
  // consistency
  // @params[IN] candidates: candidates of measured velocity
  // @return selected measurement of velocity
  Eigen::Vector3f SelectMeasuredVelocityAccordingMotionConsistency(
      const std::vector<Eigen::Vector3f>& candidates);

  // @brief update filter
  // @params[IN] measured_anchor_point: anchor point of given measurement
  // @params[IN] measured_velocity: velocity of given measurement
  // @params[IN] time_diff: time interval from last updating
  // @return nothing
  void UpdateVelocity(const Eigen::VectorXf& measured_anchor_point,
                      const Eigen::VectorXf& measured_velocity,
                      const double& time_diff);

  // @brief compute update quality for adaptive filtering
  // @params[IN] new_object: new object for current updating
  // @params[IN] old_object: old object for last updating
  // @reutrn nothing
  void ComputeUpdateQuality(const std::shared_ptr<TrackedObject>& new_object,
                            const std::shared_ptr<TrackedObject>& old_object);

  // @brief compute update quality by using association score
  // @params[IN] new_object: new object for current updating
  // @return upate quality according association score
  float ComputeUpdateQualityAccordingAssociationScore(
      const std::shared_ptr<TrackedObject>& new_object);

  // @brief compute update quality by using association score
  // @params[IN] old_object: old object for last updaitng
  // @params[IN] new_object: new object for current updating
  // @return update quality according point number change
  float ComputeUpdateQualityAccordingPointNumChange(
      const std::shared_ptr<TrackedObject>& new_object,
      const std::shared_ptr<TrackedObject>& old_object);

  // @brief compute breakdown threshold
  // @return nothing
  void ComputeBreakdownThreshold();

  // @brief get online covariance of filter
  // @params[OUT] online_covariance: online covariance
  // @return noting
  void GetOnlineCovariance(Eigen::Matrix3f* online_covariance);

 protected:
  void EvaluateOnlineCovariance();
  Eigen::Vector3f ComputeMeasuredAcceleration(
      const Eigen::Vector3f& measured_velocity, const double& time_diff);
  void UpdateAcceleration(const Eigen::VectorXf& measured_acceleration);

  // adaptive
  static bool s_use_adaptive_;
  static double s_association_score_maximum_;

  // parameters
  static Eigen::Matrix3d s_propagation_noise_;
  static double s_measurement_noise_;
  static double s_initial_velocity_noise_;
  static double s_breakdown_threshold_maximum_;

  static size_t s_measurement_cached_history_size_minimum_;
  static size_t s_measurement_cached_history_size_maximum_;
  size_t measurement_cached_history_size_;

  // filter history
  int age_;

  std::deque<Eigen::Vector3f> history_measured_velocity_;
  std::deque<double> history_time_diff_;

  // filter covariances
  Eigen::Matrix3d velocity_covariance_;
  Eigen::Matrix3d online_velocity_covariance_;

  // filter states
  Eigen::Vector3d belief_anchor_point_;
  Eigen::Vector3d belief_velocity_;
  Eigen::Vector3d belief_acceleration_;
  Eigen::Vector3d belief_acceleration_gain_;
  double update_quality_;
  double breakdown_threshold_;
};  // class KalmanFilter

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_LIDAR_TRACKER_HM_TRACKER_KALMAN_FILTER_H_
