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

#include <string>

#include "modules/perception/lidar_tracking/tracker/multi_lidar_fusion/proto/multi_lidar_fusion_config.pb.h"

#include "modules/perception/common/lib/interface/base_init_options.h"
#include "modules/perception/lidar_tracking/tracker/common/mlf_track_data.h"

using apollo::perception::BaseInitOptions;

namespace apollo {
namespace perception {
namespace lidar {

struct MlfMotionRefinerInitOptions : public BaseInitOptions {};

struct MlfMotionRefinerOptions {};

class MlfMotionRefiner {
 public:
  MlfMotionRefiner() = default;
  ~MlfMotionRefiner() = default;

  /**
   * @brief Init motion refiner
   *
   * @param options
   * @return true
   * @return false
   */
  bool Init(const MlfMotionRefinerInitOptions& options =
                MlfMotionRefinerInitOptions());

  /**
   * @brief Refine velocity of new object
   *
   * @param track_data history track data
   * @param new_object new object to be updated
   * @return true if velocity is changed
   * @return false
   */
  bool Refine(const MlfTrackDataConstPtr& track_data,
              TrackedObjectPtr new_object);
  /**
   * @brief Get class name
   *
   * @return std::string
   */
  std::string Name() const { return "MlfMotionRefiner"; }

 protected:
  /**
   * @brief Check whether new observed object is static or not
   *
   * @param latest_object latest history object
   * @param new_object new object to be updated
   * @return true if new object is static
   * @return false
   */
  bool CheckStaticHypothesisByState(
      const TrackedObjectConstPtr& latest_object,
      const TrackedObjectConstPtr& new_object) const;

  /**
   * @brief Check whether new observed object is static or not via
   *        considering the angle velocity change
   *
   * @param latest_object latest history object
   * @param new_object new object to be updated
   * @param reasonable_angle_change_maximum max allowed angle change threshold
   * @return true if new object is static
   * @return false
   */
  bool CheckStaticHypothesisByVelocityAngleChange(
      const TrackedObjectConstPtr& latest_object,
      const TrackedObjectConstPtr& new_object,
      double reasonable_angle_change_maximum) const;

  bool CalculateVelocityAngleChange(
        const TrackedObjectConstPtr& latest_object,
        const TrackedObjectConstPtr& new_object,
        double* vel_change_angle,
        double* velocity_heading_angle) const;

  bool NeedAssignMovableObjectSpeed(
        const TrackedObjectConstPtr& latest_object,
        const TrackedObjectConstPtr& new_object) const;

 protected:
  double claping_acceleration_threshold_ = 10;
  double claping_speed_threshold_ = 1.0;
  double cyc_refine_speed_ = 1.2;
  double car_refine_speed_ = 1.0;
  const double EPSION_TIME = 1e-3;
  bool use_movable_state_check_ = false;
};  // class MlfMotionRefiner

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
