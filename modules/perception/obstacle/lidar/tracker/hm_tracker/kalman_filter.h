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

#include <vector>
#include "modules/perception/obstacle/base/object.h"
#include "modules/perception/obstacle/lidar/tracker/hm_tracker/base_filter.h"

namespace apollo {
namespace perception {

class KalmanFilter : public BaseFilter {
 public:
  KalmanFilter();
  ~KalmanFilter();

  // @brief set use adaptive for all the filter objects
  // @params[IN] use_adaptive: flag of whether use adaptive version or not
  // @return nothing
  static void SetUseAdaptive(const bool use_adaptive);

  // @brief set max adaptive score for computing update qaulity
  // @params[IN] max_adaptive_score: adaptive score upperbound
  // @return nothing
  static void SetMaxAdaptiveScore(const double max_adaptive_score);

  // @brief init core parameters of all the filter objects
  // @params[IN] centroid_measurement_noise: noise of centroid measurement
  // @params[IN] init_velocity_variance: initial variance of velocity
  // @params[IN] propagation_variance_xy: propagation uncertainty of xy
  // @params[IN] propagation_variance_z: propagation uncertainty of z
  // @return nothing
  static void InitParams(const double measurement_noise,
                         const double init_velocity_variance,
                         const double propagation_variance_xy,
                         const double propagation_variance_z);

  // @brief initialize the state of filter
  // @params[IN] anchor_point: initial anchor point for filtering
  // @params[IN] velocity: initial velocity for filtering
  // @return nothing
  void Initialize(const Eigen::Vector3f& anchor_point,
                  const Eigen::Vector3f& velocity);

  // @brief predict the state of filter
  // @params[IN] time_diff: time interval for predicting
  // @return predicted states of filtering
  Eigen::VectorXf Predict(const double time_diff);

  // @brief update filter with object
  // @params[IN] new_object: new object for current updating
  // @params[IN] old_object: old object for last updating
  // @params[IN] time_diff: time interval from last updating
  // @return nothing
  void UpdateWithObject(const TrackedObjectPtr& new_object,
                        const TrackedObjectPtr& old_object,
                        const double time_diff);

  // @brief update filter without object
  // @params[IN] time_diff: time interval from last updating
  // @return nothing
  void UpdateWithoutObject(const double time_diff);

  // @brief get state of filter
  // @params[OUT] anchor_point: anchor point of current state
  // @params[OUT] velocity: velocity of current state
  // @return nothing
  void GetState(Eigen::Vector3f* anchor_point,
                Eigen::Vector3f* velocity);

  // @brief get state of filter with accelaration
  // @params[OUT] anchor_point: anchor point of current state
  // @params[OUT] velocity: velocity of current state
  // @params[OUT] velocity_accelaration: accelaration of current state
  // @return nothing
  void GetState(Eigen::Vector3f* anchor_point,
                Eigen::Vector3f* velocity,
                Eigen::Vector3f* velocity_accelaration);

 protected:
  // @brief propagate covariance of filter
  // @params[IN] time_diff: time interval from last updating
  // @return nothing
  void Propagate(const double time_diff);

  // @brief compute measured velocity
  // @params[IN] new_object: new object for current updating
  // @params[IN] old_object: old object for last updating
  // @params[IN] time_diff: time interval from last updating
  // @return measured velocity
  Eigen::VectorXf ComputeMeasuredVelocity(
    const TrackedObjectPtr& new_object,
    const TrackedObjectPtr& old_object,
    const double time_diff);

  // @brief compute measured anchor point velocity
  // @params[IN] new_object: new object for current updating
  // @params[IN] old_object: old object for last updating
  // @params[IN] time_diff: time interval from last updating
  // @return measured anchor point elocity
  Eigen::VectorXf ComputeMeasuredAnchorPointVelocity(
    const TrackedObjectPtr& new_object,
    const TrackedObjectPtr& old_object,
    const double time_diff);

  // @brief compute measured bbox center velocity
  // @params[IN] new_object: new object for current updating
  // @params[IN] old_object: old object for last updating
  // @params[IN] time_diff: time interval from last updating
  // @return measured bbox center velocity
  Eigen::VectorXf ComputeMeasuredBboxCenterVelocity(
    const TrackedObjectPtr& new_object,
    const TrackedObjectPtr& old_object,
    const double time_diff);

  // @brief compute measured bbox corner velocity
  // @params[IN] new_object: new object for current updating
  // @params[IN] old_object: old object for last updating
  // @params[IN] time_diff: time interval from last updating
  // @return measured bbox corner velocity
  Eigen::VectorXf ComputeMeasuredBboxCornerVelocity(
    const TrackedObjectPtr& new_object,
    const TrackedObjectPtr& old_object,
    const double time_diff);

  // @brief select measured velocity among candidates
  // @params[IN] candidates: candidates of measured velocity
  // @return measured velocity
  Eigen::Vector3f SelectMeasuredVelocity(
    const std::vector<Eigen::Vector3f>& candidates);

  // @brief select measured velocity among candidates according motion
  // consistency
  // @params[IN] candidates: candidates of measured velocity
  // @return measured velocity
  Eigen::Vector3f SelectMeasuredVelocityAccordingMotionConsistency(
    const std::vector<Eigen::Vector3f>& candidates);

  // @brief update filter
  // @params[IN] measured_anchor_point: anchor point of given measurement
  // @params[IN] measured_velocity: velocity of given measurement
  // @params[IN] time_diff: time interval from last updating
  // @return nothing
  void UpdateModel(const Eigen::VectorXf measured_anchor_point,
                   const Eigen::VectorXf measured_velocity,
                   const double time_diff);

  // @brief compute update quality for adaptive filtering
  // @params[IN] new_object: new object for current updating
  // @params[IN] old_object: old object for last updating
  // @reutrn nothing
  void ComputeUpdateQuality(const TrackedObjectPtr& new_object,
                            const TrackedObjectPtr& old_object);

  // @brief compute update quality by using association score
  // @params[IN] new_object: new object for current updating
  // @return upate quality according association score
  float ComputeUpdateQualityAccordingAssociationScore(
    const TrackedObjectPtr& new_object);

  // @brief compute update quality by using association score
  // @params[IN] new_object: new object for current updating
  // @return update quality according point number change
  float ComputeUpdateQualityAccordingPointNumChange(
    const TrackedObjectPtr& new_object,
    const TrackedObjectPtr& old_object);

  // @brief compute breakdown threshold
  // @return breakdown threshold
  float ComputeBreakdownThreshold();

 protected:
  // adaptive
  static bool             s_use_adaptive_;
  static double           s_max_adaptive_score_;

  // parameters
  static double           s_measurement_noise_;
  static double           s_init_velocity_variance_;
  static double           s_propagation_variance_xy_;
  static double           s_propagation_variance_z_;

  // filter history
  int                     age_;

  // filter covariances
  Eigen::Matrix3d         covariance_velocity_;
  Eigen::Matrix3d         covariance_propagation_uncertainty_;

  // filter states
  double                  update_quality_;
  Eigen::Vector3d         belief_anchor_point_;
  Eigen::Vector3d         belief_velocity_;
  Eigen::Vector3d         belief_velocity_accelaration_;
};  // class KalmanFilter

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_LIDAR_TRACKER_HM_TRACKER_KALMAN_FILTER_H_
