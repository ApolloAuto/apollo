/******************************************************************************
 * Copyright 2019 The Apollo Authors. All Rights Reserved.
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

#include "pcl/PointIndices.h"
#include "pcl/io/pcd_io.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/search/impl/kdtree.hpp"

namespace apollo {
namespace perception {
namespace benchmark {

typedef ::pcl::PointIndices PointIndices;
typedef ::pcl::PointIndices::Ptr PointIndicesPtr;
typedef ::pcl::PointXY Point2d;
typedef ::pcl::PointCloud<Point2d> PointCloud2d;
typedef ::pcl::PointCloud<Point2d>::Ptr PointCloud2dPtr;
typedef ::pcl::PointCloud<Point2d>::ConstPtr PointCloud2dConstPtr;

struct PointXYZIT {
  float x;
  float y;
  float z;
  uint8_t intensity;
  double timestamp;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

typedef ::pcl::PointCloud<PointXYZIT> PointXYZITCloud;
typedef ::pcl::PointCloud<PointXYZIT>::Ptr PointXYZITCloudPtr;
typedef ::pcl::PointCloud<PointXYZIT>::ConstPtr PointXYZITCloudConstPtr;

struct PointXYZL {
  float x;
  float y;
  float z;
  uint32_t label;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

typedef ::pcl::PointCloud<PointXYZL> PointXYZLCloud;
typedef ::pcl::PointCloud<PointXYZL>::Ptr PointXYZLCloudPtr;
typedef ::pcl::PointCloud<PointXYZL>::ConstPtr PointXYZLCloudConstPtr;

struct PointXYZIL;
// typedef PointXYZIHT Point;
typedef PointXYZIL Point;
typedef pcl::PointCloud<Point> PointCloud;
typedef pcl::PointCloud<Point>::Ptr PointCloudPtr;
typedef pcl::PointCloud<Point>::ConstPtr PointCloudConstPtr;

struct PointXYZIL {
  float x = 0.f;
  float y = 0.f;
  float z = 0.f;
  float intensity = 0.f;
  uint32_t label = 0;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

}  // namespace benchmark
}  // namespace perception
}  // namespace apollo

POINT_CLOUD_REGISTER_POINT_STRUCT(
        apollo::perception::benchmark::PointXYZIT,
        (float, x, x)
        (float, y, y)
        (float, z, z)
        (std::uint8_t, intensity, intensity)
        (double, timestamp, timestamp)
)

POINT_CLOUD_REGISTER_POINT_STRUCT(
        apollo::perception::benchmark::PointXYZL,
        (float, x, x)
        (float, y, y)
        (float, z, z)
        (std::uint32_t, label, label)
)

POINT_CLOUD_REGISTER_POINT_STRUCT(
        apollo::perception::benchmark::PointXYZIL,
        (float, x, x)
        (float, y, y)
        (float, z, z)
        (float, intensity, intensity)
        (std::uint32_t, label, label)
)
