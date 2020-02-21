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

/**
 * @file pcl_point_types.h
 * @brief The pcl types.
 */

#pragma once

#include "pcl/point_types.h"

namespace apollo {
namespace localization {
namespace msf {
namespace velodyne {

struct PointXYZIRT {
  float x;
  float y;
  float z;
  unsigned char intensity;
  unsigned char ring;
  double timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
} EIGEN_ALIGN16;  // enforce SSE padding for correct memory alignment

struct PointXYZIT {
  float x;
  float y;
  float z;
  unsigned char intensity;
  double timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
} EIGEN_ALIGN16;  // enforce SSE padding for correct memory alignment

struct PointXYZIRTd {
  double x;
  double y;
  double z;
  unsigned char intensity;
  unsigned char ring;
  double timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
} EIGEN_ALIGN16;  // enforce SSE padding for correct memory alignment

struct PointXYZITd {
  double x;
  double y;
  double z;
  unsigned char intensity;
  double timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
} EIGEN_ALIGN16;  // enforce SSE padding for correct memory alignment

}  // namespace velodyne
}  // namespace msf
}  // namespace localization
}  // namespace apollo

POINT_CLOUD_REGISTER_POINT_STRUCT(
    apollo::localization::msf::velodyne::PointXYZIT,
    (float, x, x)(float, y, y)(float, z, z)(uint8_t, intensity,
                                            intensity)(double, timestamp,
                                                       timestamp))

POINT_CLOUD_REGISTER_POINT_STRUCT(
    apollo::localization::msf::velodyne::PointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)(uint8_t, intensity, intensity)(
        uint8_t, ring, ring)(double, timestamp, timestamp))

POINT_CLOUD_REGISTER_POINT_STRUCT(
    apollo::localization::msf::velodyne::PointXYZITd,
    (double, x, x)(double, y, y)(double, z, z)(uint8_t, intensity,
                                               intensity)(double, timestamp,
                                                          timestamp))

POINT_CLOUD_REGISTER_POINT_STRUCT(
    apollo::localization::msf::velodyne::PointXYZIRTd,
    (double, x, x)(double, y, y)(double, z, z)(uint8_t, intensity, intensity)(
        uint8_t, ring, ring)(double, timestamp, timestamp))
