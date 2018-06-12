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

#include "modules/perception/obstacle/base/object_supplement.h"

namespace apollo {
namespace perception {

TrackStateVars LidarFrameSupplement::state_vars;
TrackStateVars RadarFrameSupplement::state_vars;
TrackStateVars CameraFrameSupplement::state_vars;

/**RadarFrameSupplement implementation*/
RadarFrameSupplement::RadarFrameSupplement() {}

RadarFrameSupplement::~RadarFrameSupplement() {}

RadarFrameSupplement::RadarFrameSupplement(const RadarFrameSupplement& rhs) {}

RadarFrameSupplement& RadarFrameSupplement::operator=(
    const RadarFrameSupplement& rhs) {
  return (*this);
}

void RadarFrameSupplement::clone(const RadarFrameSupplement& rhs) {}

/**CameraFrameSupplement implementation*/
CameraFrameSupplement::CameraFrameSupplement() {}

CameraFrameSupplement::~CameraFrameSupplement() {}

CameraFrameSupplement::CameraFrameSupplement(const CameraFrameSupplement& rhs) {
  rhs.depth_map.copyTo(depth_map);
  rhs.label_map.copyTo(label_map);
  rhs.lane_map.copyTo(lane_map);
}

CameraFrameSupplement& CameraFrameSupplement::operator=(
    const CameraFrameSupplement& rhs) {
  rhs.depth_map.copyTo(depth_map);
  rhs.label_map.copyTo(label_map);
  rhs.lane_map.copyTo(lane_map);
  return (*this);
}

void CameraFrameSupplement::clone(const CameraFrameSupplement& rhs) {
  rhs.depth_map.copyTo(depth_map);
  rhs.label_map.copyTo(label_map);
  rhs.lane_map.copyTo(lane_map);
}

/**RadarSupplement implementation*/
RadarSupplement::RadarSupplement() {}

RadarSupplement::~RadarSupplement() {}

RadarSupplement::RadarSupplement(const RadarSupplement& rhs) {
  range = rhs.range;
  angle = rhs.angle;
  relative_radial_velocity = rhs.relative_radial_velocity;
  relative_tangential_velocity = rhs.relative_tangential_velocity;
  radial_velocity = rhs.radial_velocity;
}

RadarSupplement& RadarSupplement::operator=(const RadarSupplement& rhs) {
  range = rhs.range;
  angle = rhs.angle;
  relative_radial_velocity = rhs.relative_radial_velocity;
  relative_tangential_velocity = rhs.relative_tangential_velocity;
  radial_velocity = rhs.radial_velocity;
  return (*this);
}

void RadarSupplement::clone(const RadarSupplement& rhs) {
  range = rhs.range;
  angle = rhs.angle;
  relative_radial_velocity = rhs.relative_radial_velocity;
  relative_tangential_velocity = rhs.relative_tangential_velocity;
  radial_velocity = rhs.radial_velocity;
}

/**CameraSupplement implementation*/
CameraSupplement::CameraSupplement() {}

CameraSupplement::~CameraSupplement() {}

CameraSupplement::CameraSupplement(const CameraSupplement& rhs) {
  upper_left = rhs.upper_left;
  lower_right = rhs.lower_right;
  local_track_id = rhs.local_track_id;
  alpha = rhs.alpha;
  pts8.assign(rhs.pts8.begin(), rhs.pts8.end());
  front_upper_left = rhs.front_upper_left;
  front_lower_right = rhs.front_lower_right;
  back_upper_left = rhs.back_upper_left;
  back_lower_right = rhs.back_lower_right;
}

CameraSupplement& CameraSupplement::operator=(const CameraSupplement& rhs) {
  upper_left = rhs.upper_left;
  lower_right = rhs.lower_right;
  local_track_id = rhs.local_track_id;
  alpha = rhs.alpha;
  pts8.assign(rhs.pts8.begin(), rhs.pts8.end());
  front_upper_left = rhs.front_upper_left;
  front_lower_right = rhs.front_lower_right;
  back_upper_left = rhs.back_upper_left;
  back_lower_right = rhs.back_lower_right;

  return (*this);
}

void CameraSupplement::clone(const CameraSupplement& rhs) {
  upper_left = rhs.upper_left;
  lower_right = rhs.lower_right;
  local_track_id = rhs.local_track_id;
  alpha = rhs.alpha;
  pts8.assign(rhs.pts8.begin(), rhs.pts8.end());
  front_upper_left = rhs.front_upper_left;
  front_lower_right = rhs.front_lower_right;
  back_upper_left = rhs.back_upper_left;
  back_lower_right = rhs.back_lower_right;
}

}  // namespace perception
}  // namespace apollo
