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

#ifndef MODULES_PERCEPTION_OBSTACLE_RADAR_OBJECT_BUILDER_H_
#define MODULES_PERCEPTION_OBSTACLE_RADAR_OBJECT_BUILDER_H_

#include <memory>
#include <Eigen/Core>
#include "modules/perception/obstacle/base/types.h"
#include "modules/perception/obstacle/base/object.h"
#include "modules/common/log.h"
#include "modules/perception/obstacle/radar/interface/base_radar_detector.h"
#include "modules/perception/obstacle/radar/detector/modest/radar_define.h"
#include "modules/perception/obstacle/radar/interface/base_filter.h"
#include "modules/perception/obstacle/radar/filter/akf/adaptive_kalman_filter.h"

namespace apollo {
namespace perception {

class ObjectBuilder {
public:
void build(const RadarObsArray& raw_obstacles,
        const Eigen::Matrix4d& radar_pose,
        const Eigen::Vector2d& main_velocity,
        SensorObjects & radar_objects);

static void SetDelayFrame(int delay_frames) {
  delay_frames_ = delay_frames;
}

static void SetUseFpFilter(bool use_fp_filter) {
  use_fp_filter_ = use_fp_filter;
}

static void SetContiParams(ContiParams conti_params) {
  conti_params_ = conti_params;
}

private:
std::map<int, int> continuous_ids_;
static int delay_frames_;
static bool use_fp_filter_;
static ContiParams conti_params_;
};

} // namespace perception
} // namespace apollo
#endif // MODULES_PERCEPTION_OBSTACLE_RADAR_OBJECT_BUILDER_H_