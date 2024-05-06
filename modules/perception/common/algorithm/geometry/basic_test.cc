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
#include "modules/perception/common/algorithm/geometry/basic.h"

#include "gtest/gtest.h"
#include "modules/common/util/eigen_defs.h"
#include "modules/perception/common/base/point_cloud.h"
#include "modules/perception/common/algorithm/geometry/common.h"

namespace apollo {
namespace perception {
namespace algorithm {

using base::PointCloud;
using base::PointF;

TEST(GeometryBasicTest, cross_product_test) {
  Eigen::Vector2f p1(0.0, 0.0);
  Eigen::Vector2f p2(1.0, 0.0);
  Eigen::Vector2f p3(0.0, 1.0);
  float result = CrossProduct<float>(p1, p2, p3);
  EXPECT_NEAR(result, 1.f, std::numeric_limits<float>::epsilon());
}

TEST(GeometryBasicTest, cross_product_test2) {
  base::PointF pt1, pt2, pt3;
  pt1.x = 0.0;
  pt1.y = 0.0;
  pt2.x = 1.0;
  pt2.y = 0.0;
  pt3.x = 0.0;
  pt3.y = 1.0;
  float result_pt = CrossProduct<base::PointF>(pt1, pt2, pt3);
  EXPECT_NEAR(result_pt, 1.f, std::numeric_limits<float>::epsilon());
}

TEST(GeometryBasicTest, calculate_eucliden_dist_test) {
  base::PointF pt1, pt2;
  pt1.x = 1.f;
  pt1.y = 1.f;
  pt1.z = 1.f;
  pt2.x = 4.f;
  pt2.y = 5.f;
  pt2.z = 13.f;
  float distance = CalculateEuclidenDist<base::PointF>(pt1, pt2);
  EXPECT_NEAR(distance, 13.f, std::numeric_limits<float>::epsilon());
}

TEST(GeometryBasicTest, calculate_eucliden_dist_2d_xy_test) {
  base::PointF pt1, pt2;
  pt1.x = 1.f;
  pt1.y = 1.f;
  pt1.z = 1.f;
  pt2.x = 4.f;
  pt2.y = 5.f;
  pt2.z = 13.f;
  float distance_2d = CalculateEuclidenDist2DXY<base::PointF>(pt1, pt2);
  EXPECT_NEAR(distance_2d, 5.f, std::numeric_limits<float>::epsilon());
}

TEST(GeometryBasicTest, calculate_cos_theta_2d_xy_test) {
  Eigen::Vector3f v1(1.0, 0.0, 0.0);
  Eigen::Vector3f v2(0.0, 1.0, 0.0);
  float result = CalculateCosTheta2DXY<float>(v1, v2);
  EXPECT_NEAR(result, 0.f, std::numeric_limits<float>::epsilon());
  Eigen::Vector3f v3(0.0, 0.0, 0.0);
  result = CalculateCosTheta2DXY<float>(v3, v3);
  EXPECT_NEAR(result, 0.f, std::numeric_limits<float>::epsilon());
}

TEST(GeometryBasicTest, calculate_theta_2d_xy_test) {
  Eigen::Vector3f v1(1.0, 0.0, 0.0);
  Eigen::Vector3f v2(2.0, 0.0, 0.0);
  float result = CalculateTheta2DXY<float>(v1, v2);
  EXPECT_NEAR(result, 0.f, std::numeric_limits<float>::epsilon());
  Eigen::Vector3f v3(0.0, 0.0, 0.0);
  result = CalculateTheta2DXY<float>(v3, v3);
  EXPECT_NEAR(result, 0.f, std::numeric_limits<float>::epsilon());
  Eigen::Vector3f v4(-1.0, -1.0, 0.0);
  Eigen::Vector3f v5(1.0, -1.0, 0.0);
  result = CalculateTheta2DXY<float>(v1, v4);
  EXPECT_NEAR(result, -2.35619, 0.00001);
}

TEST(GeometryBasicTest, calculate_rotation_mat_2d_xy) {
  Eigen::Vector3f v1(1.0, 0.0, 0.0);
  Eigen::Vector3f v2(2.0, 0.0, 0.0);
  Eigen::Matrix3f result = CalculateRotationMat2DXY<float>(v1, v2);
  EXPECT_NEAR(result(0, 0), 1.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(result(0, 1), 0.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(result(0, 2), 0.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(result(1, 0), 0.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(result(1, 1), 1.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(result(1, 2), 0.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(result(2, 0), 0.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(result(2, 1), 0.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(result(2, 2), 1.f, std::numeric_limits<float>::epsilon());
  Eigen::Vector3f v3(0.0, 0.0, 0.0);
  result = CalculateRotationMat2DXY<float>(v3, v3);
  EXPECT_NEAR(result(0, 0), 0.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(result(0, 1), 0.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(result(0, 2), 0.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(result(1, 0), 0.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(result(1, 1), 0.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(result(1, 2), 0.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(result(2, 0), 0.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(result(2, 1), 0.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(result(2, 2), 0.f, std::numeric_limits<float>::epsilon());
}

TEST(GeometryBasicTest, calculate_2d_xy_project_vector) {
  Eigen::Vector3d v1(1.0, 0.0, 0.0);
  Eigen::Vector3d v2(0.0, 1.0, 0.0);
  Eigen::Vector3d result = Calculate2DXYProjectVector(v1, v2);
  EXPECT_NEAR(result(0), 0.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(result(1), 0.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(result(2), 0.f, std::numeric_limits<float>::epsilon());
  Eigen::Vector3d v3(0.0, 0.0, 0.0);
  result = Calculate2DXYProjectVector(v3, v3);
  EXPECT_NEAR(result(0), 0.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(result(1), 0.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(result(2), 0.f, std::numeric_limits<float>::epsilon());
}

TEST(GeometryBasicTest, convert_cartesian_to_polar_coordinate) {
  base::PointF pt;
  pt.x = 10.f;
  pt.y = 10.f;
  pt.z = 10.f;
  float h_angle, v_angle, dist;
  ConvertCartesiantoPolarCoordinate<base::PointF>(pt, &h_angle, &v_angle,
                                                  &dist);
  EXPECT_NEAR(h_angle, 45, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(v_angle, 35.2644, 0.0001);
  EXPECT_NEAR(dist, 17.3205, 0.0001);
}

TEST(GeometryCommonTest, is_point_xy_in_polygon_2d_xy) {
  base::PointF pt, temp;
  PointCloud<PointF> polygon;
  temp.x = 10.f;
  temp.y = 10.f;
  temp.z = 10.f;
  polygon.push_back(temp);
  temp.x = 20.f;
  temp.y = -10.f;
  temp.z = 5.f;
  polygon.push_back(temp);
  temp.x = -10.f;
  temp.y = 0.f;
  temp.z = 15.f;
  polygon.push_back(temp);
  temp.x = -10.f;
  temp.y = 0.f;
  temp.z = 10.f;
  polygon.push_back(temp);
  bool flag = false;
  pt.x = 0.f;
  pt.y = 0.f;
  pt.z = 0.f;
  flag = IsPointXYInPolygon2DXY(pt, polygon);
  EXPECT_TRUE(flag);
  pt.x = 100.f;
  pt.y = 0.f;
  pt.z = 0.f;
  flag = IsPointXYInPolygon2DXY(pt, polygon);
  EXPECT_FALSE(flag);
  pt.x = 0.f;
  pt.y = 5.f;
  pt.z = 0.f;
  flag = IsPointXYInPolygon2DXY(pt, polygon);
  EXPECT_TRUE(flag);
}

TEST(GeometryCommonTest, is_point_in_bbox) {
  Eigen::Vector3f gnd_c(0.0, 0.0, 0.0);
  Eigen::Vector3f dir_x(1.0, 0.0, 0.0);
  Eigen::Vector3f dir_y(0.0, 1.0, 0.0);
  Eigen::Vector3f dir_z(0.0, 0.0, 1.0);
  Eigen::Vector3f size(2.0, 2.0, 2.0);
  base::PointF pt_1, pt_2, pt_3, pt_4;
  pt_1.x = 0;
  pt_1.y = 0;
  pt_1.z = 0;
  pt_2.x = 10;
  pt_2.y = 0;
  pt_2.z = 0;
  pt_3.x = 0;
  pt_3.y = 10;
  pt_3.z = 0;
  pt_4.x = 0;
  pt_4.y = 0;
  pt_4.z = 10;
  bool flag = false;
  flag = IsPointInBBox<base::PointF>(gnd_c, dir_x, dir_y, dir_z, size, pt_1);
  EXPECT_TRUE(flag);
  flag = IsPointInBBox<base::PointF>(gnd_c, dir_x, dir_y, dir_z, size, pt_2);
  EXPECT_FALSE(flag);
  flag = IsPointInBBox<base::PointF>(gnd_c, dir_x, dir_y, dir_z, size, pt_3);
  EXPECT_FALSE(flag);
  flag = IsPointInBBox<base::PointF>(gnd_c, dir_x, dir_y, dir_z, size, pt_4);
  EXPECT_FALSE(flag);
}

TEST(GeometryCommonTest, calculate_bbox_size_center_2d_xy) {
  Eigen::Vector3f size;
  Eigen::Vector3d center;
  PointCloud<PointF> cloud;
  base::PointF temp;
  temp.x = 10.f;
  temp.y = 10.f;
  temp.z = 5.f;
  cloud.push_back(temp);
  temp.x = 20.f;
  temp.y = -10.f;
  temp.z = 15.f;
  cloud.push_back(temp);
  temp.x = -10.f;
  temp.y = 15.f;
  temp.z = -5.f;
  cloud.push_back(temp);

  Eigen::Vector3f direction(1.0, 0.0, 0.0);
  CalculateBBoxSizeCenter2DXY<PointCloud<PointF>>(cloud, direction, &size,
                                                  &center);
  constexpr float kFloatEpsilon = std::numeric_limits<float>::epsilon();
  EXPECT_NEAR(size(0), 30.f, kFloatEpsilon);
  EXPECT_NEAR(size(1), 25.f, kFloatEpsilon);
  EXPECT_NEAR(size(2), 20.f, kFloatEpsilon);
  constexpr double kDoubleEpsilon = std::numeric_limits<double>::epsilon();
  EXPECT_NEAR(center(0), 5.0, kDoubleEpsilon);
  EXPECT_NEAR(center(1), 2.5, kDoubleEpsilon);
  EXPECT_NEAR(center(2), -5.0, kDoubleEpsilon);
}

TEST(GeometryCommonTest, calculate_most_consistent_bbox_direction) {
  Eigen::Vector3f previous_dir(1.0, 0.0, 0.0);
  Eigen::Vector3f current_dir(0.0, 1.0, 0.0);
  CalculateMostConsistentBBoxDir2DXY(previous_dir, &current_dir);
  EXPECT_NEAR(current_dir(0), -1.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(current_dir(1), 0.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(current_dir(2), 0.f, std::numeric_limits<float>::epsilon());
  CalculateMostConsistentBBoxDir2DXY(current_dir, &previous_dir);
  EXPECT_NEAR(previous_dir(0), -1.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(previous_dir(1), 0.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(previous_dir(2), 0.f, std::numeric_limits<float>::epsilon());
  Eigen::Vector3f previous_dir2(1.0, 0.0, 0.0);
  Eigen::Vector3f current_dir2(0.0, -1.0, 0.0);
  CalculateMostConsistentBBoxDir2DXY(previous_dir2, &current_dir2);
  EXPECT_NEAR(current_dir2(0), -1.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(current_dir2(1), 0.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(current_dir2(2), 0.f, std::numeric_limits<float>::epsilon());
}

TEST(GeometryCommonTest, calculate_iou_2d_xy) {
  Eigen::Vector3f center0(0.0, 0.0, 0.0);
  Eigen::Vector3f center1(0.0, 1.0, 0.0);
  Eigen::Vector3f size0(2.0, 2.0, 2.0);
  Eigen::Vector3f size1(2.0, 2.0, 2.0);
  float result = CalculateIou2DXY(center0, size0, center1, size1);
  EXPECT_NEAR(result, 1.f / 3.f, std::numeric_limits<float>::epsilon());
  Eigen::Vector3f center2(0.0, 0.0, 0.0);
  Eigen::Vector3f center3(0.0, 0.0, 0.0);
  Eigen::Vector3f size2(0.0, 0.0, 0.0);
  Eigen::Vector3f size3(0.0, 0.0, 0.0);
  result = CalculateIou2DXY(center2, size2, center3, size3);
  EXPECT_NEAR(result, 0.f, std::numeric_limits<float>::epsilon());
}

TEST(GeometryCommonTest, calculate_dist_and_dir_to_segs) {
  Eigen::Vector3f pt(0.0, 0.0, 0.0);
  PointCloud<PointF> cloud;
  base::PointF temp;
  temp.x = 10.f;
  temp.y = 0.f;
  temp.z = 0.f;
  cloud.push_back(temp);
  temp.x = 0.f;
  temp.y = -10.f;
  temp.z = 0.f;
  cloud.push_back(temp);
  temp.x = 0.f;
  temp.y = 0.f;
  temp.z = -5.f;
  cloud.push_back(temp);
  Eigen::Vector3f direction;
  float distance = 0.f;
  CalculateDistAndDirToSegs<PointF>(pt, cloud, &distance, &direction);
  // EXPECT_NEAR(direction(0), -1.f, std::numeric_limits<float>::epsilon());
  // EXPECT_NEAR(direction(1), 0.f, std::numeric_limits<float>::epsilon());
  // EXPECT_NEAR(direction(2), 0.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(distance, 0.f, std::numeric_limits<float>::epsilon());
  cloud.clear();
  cloud.push_back(temp);
  EXPECT_FALSE(
      CalculateDistAndDirToSegs<PointF>(pt, cloud, &distance, &direction));
  // EXPECT_NEAR(direction(0), -1.f, std::numeric_limits<float>::epsilon());
  // EXPECT_NEAR(direction(1), 0.f, std::numeric_limits<float>::epsilon());
  // EXPECT_NEAR(direction(2), 0.f, std::numeric_limits<float>::epsilon());
  // EXPECT_NEAR(distance, 0.f, std::numeric_limits<float>::epsilon());
}

TEST(GeometryCommonTest, calculate_dist_and_dir_to_boundary) {
  Eigen::Vector3f pt(0.0, 0.0, 0.0);
  PointCloud<PointF> left, right;
  base::PointF temp;
  temp.x = 10.f;
  temp.y = 0.f;
  temp.z = 0.f;
  left.push_back(temp);
  temp.x = 0.f;
  temp.y = -10.f;
  temp.z = 0.f;
  left.push_back(temp);
  temp.x = 0.f;
  temp.y = 0.f;
  temp.z = -5.f;
  right.push_back(temp);
  temp.x = -20.f;
  temp.y = 0.f;
  temp.z = 0.f;
  right.push_back(temp);
  Eigen::Vector3f direction;
  float distance;
  CalculateDistAndDirToBoundary<PointF>(pt, left, right, &distance, &direction);
  // EXPECT_NEAR(direction(0), -1.f, std::numeric_limits<float>::epsilon());
  // EXPECT_NEAR(direction(1), 0.f, std::numeric_limits<float>::epsilon());
  // EXPECT_NEAR(direction(2), 0.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(distance, 0.f, std::numeric_limits<float>::epsilon());
  CalculateDistAndDirToBoundary<PointF>(pt, right, left, &distance, &direction);
  // EXPECT_NEAR(direction(0), -1.f, std::numeric_limits<float>::epsilon());
  // EXPECT_NEAR(direction(1), 0.f, std::numeric_limits<float>::epsilon());
  // EXPECT_NEAR(direction(2), 0.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(distance, 0.f, std::numeric_limits<float>::epsilon());
}

TEST(GeometryCommonTest, calculate_dist_and_dir_to_boundary_lists) {
  Eigen::Vector3f pt(0.0, 0.0, 0.0);
  PointCloud<PointF> left, right;
  apollo::common::EigenVector<PointCloud<PointF>> left_list, right_list;
  base::PointF temp;
  temp.x = 10.f;
  temp.y = 0.f;
  temp.z = 0.f;
  left.push_back(temp);
  temp.x = 0.f;
  temp.y = -10.f;
  temp.z = 0.f;
  left.push_back(temp);
  left_list.push_back(left);
  temp.x = 0.f;
  temp.y = 0.f;
  temp.z = -5.f;
  right.push_back(temp);
  temp.x = -20.f;
  temp.y = 0.f;
  temp.z = 0.f;
  right.push_back(temp);
  right_list.push_back(right);
  Eigen::Vector3f direction;
  float distance;
  CalculateDistAndDirToBoundary<PointF>(pt, left_list, right_list, &distance,
                                        &direction);
  // EXPECT_NEAR(direction(0), -1.f, std::numeric_limits<float>::epsilon());
  // EXPECT_NEAR(direction(1), 0.f, std::numeric_limits<float>::epsilon());
  // EXPECT_NEAR(direction(2), 0.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(distance, 0.f, std::numeric_limits<float>::epsilon());
  CalculateDistAndDirToBoundary<PointF>(pt, right_list, left_list, &distance,
                                        &direction);
  // EXPECT_NEAR(direction(0), -1.f, std::numeric_limits<float>::epsilon());
  // EXPECT_NEAR(direction(1), 0.f, std::numeric_limits<float>::epsilon());
  // EXPECT_NEAR(direction(2), 0.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(distance, 0.f, std::numeric_limits<float>::epsilon());
}

}  // namespace algorithm
}  // namespace perception
}  // namespace apollo
