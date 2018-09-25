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
#include "modules/perception/common/geometry/convex_hull_2d.h"

#include <limits>
#include <cfloat>
#include "gtest/gtest.h"

#include "modules/perception/base/point_cloud_types.h"
#include "modules/perception/base/point_cloud.h"
#include "modules/perception/base/point.h"

namespace apollo {
namespace perception {
namespace common {

using PointF = apollo::perception::base::PointXYZIT<float>;
using PointFCloud = apollo::perception::base::PointCloud<PointF>;

TEST(ConvexHull2DTest, convex_hull_2d) {
  ConvexHull2D<PointFCloud, PointFCloud> convex_hull_2d;
  PointFCloud pointcloud_in, pointcloud_out;
  bool flag = true;
  flag = convex_hull_2d.GetConvexHull(pointcloud_in, &pointcloud_out);
  EXPECT_FALSE(flag);
  // float heigh = 0.0f;
  for (size_t i = 0; i < 10; i++) {
    for (size_t j = 0; j < 10; j++) {
      PointF pt;
      pt.x = i;
      pt.y = j;
      pt.z = 0.0;
      pointcloud_in.push_back(pt, 0.0, i * 1.0f);
    }
  }
  flag = convex_hull_2d.GetConvexHull(pointcloud_in, &pointcloud_out);
  EXPECT_TRUE(flag);
  EXPECT_EQ(pointcloud_out.size(), 4);
  flag = false;
  pointcloud_out.clear();
  float distance_above_ground_thres = -1.0f;
  flag = convex_hull_2d.GetConvexHullWithoutGround(
      pointcloud_in, distance_above_ground_thres, &pointcloud_out);
  EXPECT_TRUE(flag);
  EXPECT_EQ(pointcloud_out.size(), 4);
  pointcloud_out.clear();
  float distance_beneath_head_thres = 10.0f;
  flag = convex_hull_2d.GetConvexHullWithoutGroundAndHead(
      pointcloud_in, distance_above_ground_thres, distance_beneath_head_thres,
      &pointcloud_out);
  EXPECT_TRUE(flag);
  EXPECT_EQ(pointcloud_out.size(), 4);
  distance_above_ground_thres = 10.0f;
  distance_beneath_head_thres = -10.0f;
  pointcloud_out.clear();
  flag = convex_hull_2d.GetConvexHullWithoutGround(
      pointcloud_in, distance_above_ground_thres, &pointcloud_out);
  EXPECT_TRUE(flag);
  EXPECT_EQ(pointcloud_out.size(), 4);
  pointcloud_out.clear();
  flag = convex_hull_2d.GetConvexHullWithoutGroundAndHead(
      pointcloud_in, distance_above_ground_thres, distance_beneath_head_thres,
      &pointcloud_out);
  EXPECT_TRUE(flag);
  EXPECT_EQ(pointcloud_out.size(), 4);

  distance_above_ground_thres = -10.0f;
  distance_beneath_head_thres = 10.0f;
  pt.x = 15.5f;
  pt.y = 5.5f;
  pt.z = 0.f;
  pointcloud_in.push_back(pt, 0.0, FLT_MAX);
  pointcloud_out.clear();
  flag = convex_hull_2d.GetConvexHullWithoutGround(
      pointcloud_in, distance_above_ground_thres, &pointcloud_out);
  EXPECT_TRUE(flag);
  EXPECT_EQ(pointcloud_out.size(), 5);
  pointcloud_out.clear();
  flag = convex_hull_2d.GetConvexHullWithoutGroundAndHead(
      pointcloud_in, distance_above_ground_thres, distance_beneath_head_thres,
      &pointcloud_out);
  EXPECT_TRUE(flag);
  EXPECT_EQ(pointcloud_out.size(), 5);
}

TEST(ConvexHull2DTest1, convex_hull_2d1) {
  ConvexHull2D<PointFCloud, PointFCloud> convex_hull_2d;
  PointFCloudPtr pointcloud_in_ptr =
      PointFCloudPtr(new PointFCloud(128, 128, PointF()));
  PointFCloud pointcloud_out;
  PointF tmp_pt;
  // float heigh = 0.0f;
  for (size_t i = 0; i < 128; i++) {
    for (size_t j = 0; j < 128; j++) {
      tmp_pt.x = 1.f * i;
      tmp_pt.y = 1.f * j;
      tmp_pt.z = 1.f * i;
      float heigh = i * 1.0f;
      *(pointcloud_in_ptr->at(i, j)) = tmp_pt;
      *(pointcloud_in_ptr->points_height(i, j)) = heigh;
    }
  }
  bool flag = false;
  float distance_above_ground_thres = 63.f;
  flag = convex_hull_2d.GetConvexHullWithoutGround(
      *pointcloud_in_ptr, distance_above_ground_thres, &pointcloud_out);
  EXPECT_TRUE(flag);
  EXPECT_EQ(pointcloud_out.size(), 4);
  distance_above_ground_thres = 20.0f;
  float distance_beneath_head_thres = 30.0f;
  pointcloud_out.clear();
  flag = convex_hull_2d.GetConvexHullWithoutGroundAndHead(
      *pointcloud_in_ptr, distance_above_ground_thres,
      distance_beneath_head_thres, &pointcloud_out);
  EXPECT_TRUE(flag);
  EXPECT_EQ(pointcloud_out.size(), 4);
}

}  // namespace common
}  // namespace perception
}  // namespace apollo
