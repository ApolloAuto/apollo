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

#ifndef MODULES_PERCEPTION_OBSTACLE_FUSION_PROBABILISTIC_FUSION_PBF_BASE_MOTION_FUSION_H_
#define MODULES_PERCEPTION_OBSTACLE_FUSION_PROBABILISTIC_FUSION_PBF_BASE_MOTION_FUSION_H_
#include <utility>
#include <vector>
#include "modules/common/macro.h"
#include "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_sensor_object.h"
// #include
// "modules/perception/obstacle/fusion/probabilistic_fusion/pbf_track.h"

namespace apollo {
namespace perception {

class PbfBaseMotionFusion {
 public:
  PbfBaseMotionFusion() : name_("PbfBaseMotionFusion"), initialized_(false) {}
  virtual ~PbfBaseMotionFusion() {}

  // @brief initialize the state of filter
  // @params[IN] anchor_point: initial anchor point for filtering
  // @params[IN] velocity: initial velocity for filtering
  // @return nothing
  virtual void Initialize(const Eigen::Vector3d &anchor_point,
                          const Eigen::Vector3d &velocity) = 0;

  // @brief initialize state of the filter
  // @params[IN] new_object: initial object for filtering
  // @return nothing
  virtual void Initialize(const PbfSensorObjectPtr new_object) = 0;

  // @brief predict the state of filter
  // @params[OUT] anchor_point:  predicted anchor point for filtering
  // @params[OUT] velocity: predicted velocity
  // @params[IN] time_diff: time interval from last update
  // @return nothing
  virtual void Predict(Eigen::Vector3d &anchor_point, Eigen::Vector3d &velocity,
                       const double time_diff) = 0;

  // @brief update with measurements
  // @params[IN] new_object: new object for current update
  // @params[IN] time_diff: time interval from last update;
  // @return nothing
  virtual void UpdateWithObject(const PbfSensorObjectPtr new_object,
                                const double time_diff) = 0;

  // @brief update without measurements
  // @params[IN] time_diff: time interval from last update
  // @return nothing
  virtual void UpdateWithoutObject(const double time_diff) = 0;

  // @brief get current state of the filter
  // @params[OUT] anchor_point: current anchor_point
  // @params[OUT] velocity: current velocity
  // @return nothing
  virtual void GetState(Eigen::Vector3d &anchor_point,
                        Eigen::Vector3d &velocity) = 0;

  std::string name() {
    return name_;
  }

  // @brief check if filter has been initialized
  // @return initialization status
  bool Initialized() const {
    return initialized_;
  }

 protected:
  std::string name_;
  bool initialized_;
};

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_FUSION_PROBABILISTIC_FUSION_PBF_BASE_MOTION_FUSION_H
