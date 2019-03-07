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

#include "modules/perception/lidar/lib/tracker/common/mlf_track_data.h"

namespace apollo {
namespace perception {
namespace lidar {

struct MlfMotionRefinerInitOptions {};

struct MlfMotionRefinerOptions {};

class MlfMotionRefiner {
 public:
  MlfMotionRefiner() = default;
  ~MlfMotionRefiner() = default;

  bool Init(const MlfMotionRefinerInitOptions& options =
                MlfMotionRefinerInitOptions());

  // @brief: refine velocity of new object
  // @params [in]: history track data
  // @params [in]: new object to be updated
  // @return: true if velocity is changed
  bool Refine(const MlfTrackDataConstPtr& track_data,
              TrackedObjectPtr new_object);

  std::string Name() const { return "MlfMotionRefiner"; }

 protected:
  // @brief: check whether new observed object is static or not
  // @params[in]: latest history object
  // @params[in]: new object to be updated
  // @return: true if new object is static
  bool CheckStaticHypothesisByState(
      const TrackedObjectConstPtr& latest_object,
      const TrackedObjectConstPtr& new_object) const;

  // @brief: check whether new observed object is static or not via
  //         considering the angle velocity change
  // @params [in]: latest history object
  // @params [in]: new object to be updated
  // @params [in]: max allowed angle change threshold
  // @return: true if new object is static
  bool CheckStaticHypothesisByVelocityAngleChange(
      const TrackedObjectConstPtr& latest_object,
      const TrackedObjectConstPtr& new_object,
      double reasonable_angle_change_maximum) const;

 protected:
  double claping_acceleration_threshold_ = 10;
  double claping_speed_threshold_ = 1.0;
  const double EPSION_TIME = 1e-3;
};  // class MlfMotionRefiner

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
