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

#ifndef MODULES_PERCEPTION_LIB_PCL_UTIL_PCL_TYPES_H_
#define MODULES_PERCEPTION_LIB_PCL_UTIL_PCL_TYPES_H_

#include "pcl/common/time.h"
#include "pcl/common/transforms.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/search/impl/kdtree.hpp"

namespace apollo {
namespace perception {
namespace pcl_util {

typedef pcl::PointIndices PointIndices;
typedef pcl::PointIndices::Ptr PointIndicesPtr;
typedef pcl::PointXY Point2d;
typedef pcl::PointCloud<Point2d> PointCloud2d;
typedef pcl::PointCloud<Point2d>::Ptr PointCloud2dPtr;
typedef pcl::PointCloud<Point2d>::ConstPtr PointCloud2dConstPtr;

struct PointXYZIH;

typedef PointXYZIH Point;
typedef pcl::PointCloud<Point> PointCloud;
typedef pcl::PointCloud<Point>::Ptr PointCloudPtr;
typedef pcl::PointCloud<Point>::ConstPtr PointCloudConstPtr;
typedef pcl::PointXYZRGB CPoint;
typedef pcl::PointCloud<CPoint> CPointCloud;
typedef pcl::PointCloud<CPoint>::Ptr CPointCloudPtr;
typedef pcl::PointCloud<CPoint>::ConstPtr CPointCloudConstPtr;
typedef pcl::KdTreeFLANN<Point> KdTree;

struct BoundingCube {
  float x;  // center of box
  float y;  // center of box
  float z;  // center of box
  float length;
  float width;
  float height;
  float yaw;

  float trans_x;  // center of points
  float trans_y;  // center of points
  float trans_z;  // center of points
};

// using double type to define x, y, z.
struct PointD {
  double x;
  double y;
  double z;
  uint8_t intensity;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

typedef ::pcl::PointCloud<PointD> PointDCloud;
typedef ::pcl::PointCloud<PointD>::Ptr PointDCloudPtr;
typedef ::pcl::PointCloud<PointD>::ConstPtr PointDCloudConstPtr;

struct PointXYZIH {
  PCL_ADD_POINT4D;
  float intensity;
  float h;  // height from ground

  PointXYZIH() {
    x = 0.0f;
    y = 0.0f;
    z = 0.0f;
    h = 0.0f;
    intensity = 0.0f;
    data[3] = 1.0f;
  }

  explicit PointXYZIH(float _intensity) {
    x = 0.0f;
    y = 0.0f;
    z = 0.0f;
    data[3] = 1.0f;
    intensity = _intensity;
  }
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

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

struct RawPointXYZIT {
  PCL_ADD_POINT4D
  uint8_t intensity;
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

}  // namespace pcl_util
}  // namespace perception
}  // namespace apollo

POINT_CLOUD_REGISTER_POINT_STRUCT(
    apollo::perception::pcl_util::PointD,
    (double, x, x)(double, y, y)(double, z, z)(uint8_t, intensity, intensity))

POINT_CLOUD_REGISTER_POINT_STRUCT(apollo::perception::pcl_util::PointXYZIH,
                                  (float, x, x)(float, y, y)(float, z, z)(
                                      float, intensity, intensity)(float, h, h))

POINT_CLOUD_REGISTER_POINT_STRUCT(apollo::perception::pcl_util::PointXYZIT,
                                  (float, x, x)(float, y, y)(float, z, z)(
                                      uint8_t, intensity,
                                      intensity)(double, timestamp, timestamp))
POINT_CLOUD_REGISTER_POINT_STRUCT(apollo::perception::pcl_util::RawPointXYZIT,
                                  (float, x, x)(float, y, y)(float, z, z)(
                                      uint8_t, intensity,
                                      intensity)(double, timestamp, timestamp))

POINT_CLOUD_REGISTER_POINT_STRUCT(
    apollo::perception::pcl_util::PointXYZIRT,
    (float, x, x)(float, y, y)(float, z, z)(uint8_t, intensity, intensity)(
        uint8_t, ring, ring)(double, timestamp, timestamp))

POINT_CLOUD_REGISTER_POINT_STRUCT(apollo::perception::pcl_util::PointXYZITd,
                                  (double, x, x)(double, y, y)(double, z, z)(
                                      uint8_t, intensity,
                                      intensity)(double, timestamp, timestamp))

POINT_CLOUD_REGISTER_POINT_STRUCT(
    apollo::perception::pcl_util::PointXYZIRTd,
    (double, x, x)(double, y, y)(double, z, z)(uint8_t, intensity, intensity)(
        uint8_t, ring, ring)(double, timestamp, timestamp))

#endif  // MODULES_PERCEPTION_LIB_PCL_UTIL_PCL_TYPES_H_
