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

#ifndef MODULES_PERCEPTION_OBSTACLE_FUSION_PROBABILISTIC_FUSION_PBF_KALMAN_MOTION_FUSION_H_  // NOLINT
#define MODULES_PERCEPTION_OBSTACLE_FUSION_PROBABILISTIC_FUSION_PBF_KALMAN_MOTION_FUSION_H_  // NOLINT

#include <deque>
#include <memory>
#include <utility>
#include <vector>

#include "modules/common/macro.h"
#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_base_motion_fusion.h"
#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_sensor_object.h"
#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_track.h"

namespace apollo {
namespace perception {

class PbfKalmanMotionFusion : public PbfBaseMotionFusion {
 public:
  PbfKalmanMotionFusion();
  ~PbfKalmanMotionFusion();

  // @brief initialize the state of filter
  // @params[IN] anchor_point: initial anchor point for filtering
  // @params[IN] velocity: initial velocity for filtering
  // @return nothing
  void Initialize(const Eigen::Vector3d &anchor_point,
                  const Eigen::Vector3d &velocity);

  // @brief initialize state of the filter
  // @params[IN] new_object: initial object for filtering
  // @return nothing
  void Initialize(const std::shared_ptr<PbfSensorObject> new_object);

  // @brief predict the state of filter
  // @params[OUT] anchor_point:  predicted anchor point for filtering
  // @params[OUT] velocity: predicted velocity
  // @params[IN] time_diff: time interval from last update
  // @return nothing
  void Predict(Eigen::Vector3d *anchor_point, Eigen::Vector3d *velocity,
               const double time_diff);

  // @brief update with measurements
  // @params[IN] new_object: new object for current update
  // @params[IN] time_diff: time interval from last update;
  // @return nothing
  void UpdateWithObject(const std::shared_ptr<PbfSensorObject> new_object,
                        const double time_diff);

  // @brief update without measurements
  // @params[IN] time_diff: time interval from last update
  // @return nothing
  void UpdateWithoutObject(const double time_diff);

  // @brief get current state of the filter
  // @params[OUT] anchor_point: current anchor_point
  // @params[OUT] velocity: current velocity
  // @return nothing
  void GetState(Eigen::Vector3d *anchor_point, Eigen::Vector3d *velocity);

  // @brief set current state of the filter
  // @params[IN] anchor_point: updated anchor_point
  // @params[IN] velocity: updated velocity
  // @return nothing
  void SetState(const Eigen::Vector3d &anchor_point,
                const Eigen::Vector3d &velocity);

 protected:
  int GetRadarHistoryLength();

  int GetLidarHistoryLength();

  int GetLidarHistoryIndex(const int &history_seq);

  int GetRadarHistoryIndex(const int &history_seq);

  double GetHistoryTimediff(const int &history_index,
                            const double &current_timestamp);

  void UpdateAcceleration(const Eigen::VectorXd &measured_acceleration);

  Eigen::Vector3d belief_anchor_point_;
  Eigen::Vector3d belief_velocity_;
  Eigen::Vector3d belief_acceleration_;

  Eigen::Vector4d priori_state_;
  Eigen::Vector4d posteriori_state_;

  Eigen::Matrix4d p_matrix_;
  // the state-transition matrix
  Eigen::Matrix4d a_matrix_;
  // the observation mode
  Eigen::Matrix4d c_matrix_;

  // the covariance of the process noise
  Eigen::Matrix4d q_matrix_;
  //  the covariance of the observation noise
  Eigen::Matrix4d r_matrix_;

  // Optimal Kalman gain
  Eigen::Matrix4d k_matrix_;

  std::deque<bool> history_lidar_radar_consistency_;
  std::deque<Eigen::Vector3d> history_velocity_;
  std::deque<double> history_time_diff_;
  std::deque<bool> history_velocity_is_radar_;
};

}  // namespace perception
}  // namespace apollo

// clang-format off
#endif  // MODULES_PERCEPTION_OBSTACLE_FUSION_PROBABILISTIC_FUSION_PBF_KALMAN_MOTION_FUSION_H_ // NOLINT
// clang-format on
