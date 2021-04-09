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

#ifndef MODULES_PERCEPTION_OBSTACLE_RADAR_MODEST_OBJECT_BUILDER_H_
#define MODULES_PERCEPTION_OBSTACLE_RADAR_MODEST_OBJECT_BUILDER_H_

#include <memory>
#include <unordered_map>

#include "Eigen/Core"

#include "modules/common/log.h"
#include "modules/perception/obstacle/base/object.h"
#include "modules/perception/obstacle/base/types.h"
#include "modules/perception/obstacle/radar/interface/base_radar_detector.h"
#include "modules/perception/obstacle/radar/modest/radar_define.h"

namespace apollo {
namespace perception {

class ObjectBuilder {
 public:
  ObjectBuilder() = default;
  ~ObjectBuilder() = default;

  // @brief: build radar objects
  // @param [in]: raw obstacles from radar driver.
  // @param [in]: radar pose from localization
  // @param [in]: host car velocity from localization
  // @param [out]: built radar objects
  // @return nothing
  bool Build(const ContiRadar &raw_obstacles, const Eigen::Matrix4d &radar_pose,
             const Eigen::Vector2d &main_velocity,
             SensorObjects *radar_objects);

  void SetDelayFrame(int delay_frames) { delay_frames_ = delay_frames; }

  void SetUseFpFilter(bool use_fp_filter) { use_fp_filter_ = use_fp_filter; }

  void SetContiParams(const ContiParams &conti_params) {
    conti_params_ = conti_params;
  }

 private:
  std::unordered_map<int, int> continuous_ids_;
  int delay_frames_ = 4;
  bool use_fp_filter_ = true;
  ContiParams conti_params_;
};

}  // namespace perception
}  // namespace apollo
#endif  // MODULES_PERCEPTION_OBSTACLE_RADAR_MODEST_OBJECT_BUILDER_H_
