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

/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2011, 2012 Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 *
 *  Id:database.h15542011−06−1422:11:17Zjack.oquin
 */

/** \file
 *
 *  Point Cloud Library point structures for Velodyne data.
 *
 *  @author Jesse Vera
 *  @author Jack O'Quin
 *  @author Piyush Khandelwal
 */

#ifndef MODULES_DRIVERS_LIDAR_VELODYNE_POINTCLOUD_POINT_TYPES_H_
#define MODULES_DRIVERS_LIDAR_VELODYNE_POINTCLOUD_POINT_TYPES_H_

#include <pcl/point_types.h>

namespace apollo {
namespace drivers {
namespace lidar_velodyne {

/** Euclidean Velodyne coordinate, including intensity and ring number. */
struct PointXYZIR {
  PCL_ADD_POINT4D;                 // quad-word XYZ
  float intensity;                 ///< laser intensity reading
  uint16_t ring;                   ///< laser ring number
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // ensure proper alignment
} EIGEN_ALIGN16;

}  // namespace lidar_velodyne
}  // namespace drivers
}  // namespace apollo

POINT_CLOUD_REGISTER_POINT_STRUCT(
    apollo::drivers::lidar_velodyne::PointXYZIR,
    (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                            intensity)(uint16_t, ring, ring))

#endif  // MODULES_DRIVERS_LIDAR_VELODYNE_POINTCLOUD_POINT_TYPES_H_
