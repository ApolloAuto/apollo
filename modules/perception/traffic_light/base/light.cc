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
#include "modules/perception/traffic_light/base/light.h"

#include <cmath>

#include "modules/common/log.h"

namespace apollo {
namespace perception {
namespace traffic_light {

std::ostream &operator<<(std::ostream &os, const Light &light) {
  os << "Light id:" << light.info.id().id() << " status:" << light.status.color;
  return os;
}

double Distance2Stopline(
    const Eigen::Matrix4d &car_pose,
    const google::protobuf::RepeatedPtrField<hdmap::Curve> &stoplines) {
  if (stoplines.size() == 0) {
    AWARN << "compute car to stopline's distance failed(no stopline). "
          << "car_pose:" << car_pose;
    return -1;
  }
  const hdmap::Curve &stopline = stoplines.Get(0);
  if (stopline.segment_size() == 0) {
    AWARN
        << "compute distance to stopline failed(stopline has no segment line)."
        << "car_pose:" << car_pose
        << " stopline:" << stopline.ShortDebugString();
    return -1;
  }
  if (!stopline.segment(0).has_line_segment()) {
    AWARN << "compute distance to stopline failed(stopline has no segment)."
          << "car_pose:" << car_pose
          << " stopline:" << stopline.ShortDebugString();
    return -1;
  }

  if (stopline.segment(0).line_segment().point_size() == 0) {
    AWARN << "compute distance to stopline failed(stopline has no point). "
          << "car_pose:" << car_pose
          << " stopline:" << stopline.ShortDebugString();
    return -1;
  }

  double car_x = car_pose(0, 3);
  double car_y = car_pose(1, 3);
  double stopline_x = stopline.segment(0).line_segment().point(0).x();
  double stopline_y = stopline.segment(0).line_segment().point(0).y();

  return sqrt(std::pow(car_x - stopline_x, 2) +
              std::pow(car_y - stopline_y, 2));
}

}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
