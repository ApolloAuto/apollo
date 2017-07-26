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
#include "modules/perception/obstacle/common/geometry_util.h"
#include <gtest/gtest.h>
#include <vector>
#include <cfloat>
#include "modules/common/log.h"

namespace apollo {
namespace perception {

using pcl_util::Point;
using pcl_util::PointCloud;
using pcl_util::PointCloudPtr;
using pcl_util::PointDCloud;

TEST(GeometryUtilTest, TransAffineToMatrix4) {
  Eigen::Vector3d translation(-0.0189, 1.0061, 1);
  Eigen::Vector4d rotation(0.0099, -0.0029, 0.6989, 0.7151);
  Eigen::Matrix4d trans_matrix;
  TransAffineToMatrix4(translation, rotation, &trans_matrix);
  AINFO << "trans_matrix: " << trans_matrix;
}

TEST(GeometryUtilTest, GetCloudMinMax3D) {
  pcl_util::PointCloudPtr in_cloud(new pcl_util::PointCloud);
  in_cloud->is_dense = true;
  for (int i = 0; i < 10; ++i) {
    Point pt;
    pt.x = float(i);
    pt.y = float(i);
    pt.z = float(i);
    pt.intensity = 123.0f;
    in_cloud->push_back(pt);
  }
  Eigen::Vector4f min_point;
  Eigen::Vector4f max_point;
  GetCloudMinMax3D<Point>(in_cloud, &min_point, &max_point);
  EXPECT_NEAR(min_point.x(), 0.0, 1e-6);
  EXPECT_NEAR(min_point.y(), 0.0, 1e-6);
  EXPECT_NEAR(min_point.z(), 0.0, 1e-6);
  EXPECT_NEAR(max_point.x(), 9.0, 1e-6);
  EXPECT_NEAR(max_point.y(), 9.0, 1e-6);
  EXPECT_NEAR(max_point.z(), 9.0, 1e-6);

  in_cloud->is_dense = false;
  Point pt;
  pt.x = NAN;
  pt.y = NAN;
  pt.z = NAN;
  in_cloud->push_back(pt);
  GetCloudMinMax3D<Point>(in_cloud, &min_point, &max_point);
  EXPECT_NEAR(min_point.x(), 0.0, 1e-6);
  EXPECT_NEAR(min_point.y(), 0.0, 1e-6);
  EXPECT_NEAR(min_point.z(), 0.0, 1e-6);
  EXPECT_NEAR(max_point.x(), 9.0, 1e-6);
  EXPECT_NEAR(max_point.y(), 9.0, 1e-6);
  EXPECT_NEAR(max_point.z(), 9.0, 1e-6);
}

TEST(GeometryUtilTest, TransformCloud) {
  pcl_util::PointCloudPtr in_cloud(new pcl_util::PointCloud);
  for (int i = 0; i < 10; ++i) {
    Point pt;
    pt.x = float(i);
    pt.y = float(i);
    pt.z = float(i);
    pt.intensity = 123.0f;
    in_cloud->push_back(pt);
  }

  pcl_util::PointDCloud out_cloud;
  std::vector<int> indices{0, 2, 5, 9};
  TransformCloud(in_cloud, indices, &out_cloud);

  EXPECT_EQ(out_cloud.points.size(), 4);
  EXPECT_NEAR(out_cloud.points[0].x, 0.0, 1e-6);
  EXPECT_NEAR(out_cloud.points[1].y, 2.0, 1e-6);
  EXPECT_NEAR(out_cloud.points[2].z, 5.0, 1e-6);
  EXPECT_EQ(out_cloud.points[3].intensity, 123u);
}

}  // namespace perception
}  // namespace apollo
