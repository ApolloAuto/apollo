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

#ifndef INCLUDE_POINT_TYPES_H_
#define INCLUDE_POINT_TYPES_H_

#include <pcl/point_types.h>

namespace apollo {
namespace drivers {
namespace hesai {

struct PointXYZIT {
  PCL_ADD_POINT4D
  uint8_t intensity;
  double timestamp;
  uint16_t ring;                   ///< laser ring number
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
} EIGEN_ALIGN16;

}  // namespace hesai
}  // namespace drivers
}  // namespace apollo

POINT_CLOUD_REGISTER_POINT_STRUCT(
    apollo::drivers::hesai::PointXYZIT,
    (float, x, x)(float, y, y)(float, z, z)(uint8_t, intensity, intensity)(
        double, timestamp, timestamp)(uint16_t, ring, ring))

typedef apollo::drivers::hesai::PointXYZIT PPoint;
typedef pcl::PointCloud<PPoint> PPointCloud;

#endif  // INCLUDE_POINT_TYPES_H_
