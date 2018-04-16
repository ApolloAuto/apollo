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

#ifndef MODULES_PERCEPTION_OBSTACLE_FUSION_PROBABILISTIC_FUSION_IMF_FUSION_H_  // NOLINT
#define MODULES_PERCEPTION_OBSTACLE_FUSION_PROBABILISTIC_FUSION_IMF_FUSION_H_  // NOLINT

#include <map>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "modules/common/macro.h"
#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_base_motion_fusion.h"
#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_sensor_object.h"
#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_track.h"

namespace apollo {
namespace perception {

class PbfIMFFusion : public PbfBaseMotionFusion {
 public:
  PbfIMFFusion();
  ~PbfIMFFusion();

  // @brief initialize the state of filter
  // @params[IN] anchor_point: initial anchor point for filtering
  // @params[IN] velocity: initial velocity for filtering
  // @return nothing
  virtual void Initialize(const Eigen::Vector3d& anchor_point,
                          const Eigen::Vector3d& velocity);

  // @brief initialize state of the filter
  // @params[IN] new_object: initial object for filtering
  // @return nothing
  virtual void Initialize(const std::shared_ptr<PbfSensorObject> new_object);

  // @brief predict the state of filter
  // @params[OUT] anchor_point:  predicted anchor point for filtering
  // @params[OUT] velocity: predicted velocity
  // @params[IN] time_diff: time interval from last update
  // @return nothing
  virtual void Predict(Eigen::Vector3d* anchor_point, Eigen::Vector3d* velocity,
                       const double time_diff);

  // @brief update with measurements
  // @params[IN] new_object: new object for current update
  // @params[IN] time_diff: time interval from last update;
  // @return nothing
  virtual void UpdateWithObject(
      const std::shared_ptr<PbfSensorObject> new_object,
      const double time_diff);

  // @brief update without measurements
  // @params[IN] time_diff: time interval from last update
  // @return nothing
  virtual void UpdateWithoutObject(const double time_diff);

  // @brief get current state of the filter
  // @params[OUT] anchor_point: current anchor_point
  // @params[OUT] velocity: current velocity
  // @return nothing
  virtual void GetState(Eigen::Vector3d* anchor_point,
                        Eigen::Vector3d* velocity);

  void SetState(const Eigen::Vector3d& anchor_point,
                const Eigen::Vector3d& velocity);

 protected:
  // @brief cache sensor objects in history
  // @param[IN] new object: new object for current update
  // @return nothing
  void CacheSensorObjects(const std::shared_ptr<PbfSensorObject> new_object);

  // @brief remove outdated sensor objects from cache
  // @param[IN] timestamp: current sensor timestamp
  // @return nothing
  void RemoveOutdatedSensorObjects(const double timestamp);

  // @brief get latest sensor cache
  // @param[IN] type: requested sensor type
  // @return latest sensor object
  std::shared_ptr<PbfSensorObject> GetSensorLatestCache(const SensorType type);

  void GetUncertainty(Eigen::Matrix3d* position_uncertainty,
                      Eigen::Matrix3d* velocity_uncertainty);

  bool ObtainSensorPrediction(std::shared_ptr<Object> obj,
                              double sensor_timestamp,
                              const Eigen::Matrix4d& process_noise,
                              const Eigen::Matrix4d& trans_matrix,
                              Eigen::Vector4d* state_pre,
                              Eigen::Matrix4d* cov_pre);
  // global
  Eigen::Vector3d belief_anchor_point_;
  Eigen::Vector3d belief_velocity_;
  Eigen::Vector3d belief_acceleration_;
  Eigen::Matrix<double, 4, 1> priori_state_;
  Eigen::Matrix<double, 4, 1> posteriori_state_;
  std::map<SensorType, std::queue<std::shared_ptr<PbfSensorObject>>>
      cached_sensor_objects_;

  // the omega matrix
  Eigen::Matrix<double, 4, 4> omega_matrix_;
  // the state vector is information matrix: cov.inverse() * state
  Eigen::Matrix<double, 4, 1> xi_;
  // the state-transition matrix
  Eigen::Matrix<double, 4, 4> a_matrix_;
  // the observation mode
  Eigen::Matrix<double, 4, 4> c_matrix_;
  // the covariance of the process noise
  Eigen::Matrix<double, 4, 4> q_matrix_;
};

}  // namespace perception
}  // namespace apollo

// clang-format off
#endif  // MODULES_PERCEPTION_OBSTACLE_FUSION_PROBABILISTIC_FUSION_IMF_FUSION_H_ // NOLINT
// clang-format on
