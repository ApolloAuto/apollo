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
#include "modules/perception/common/point_cloud_processing/common.h"
#include <gtest/gtest.h>
#include <limits>
namespace apollo {
namespace perception {
namespace common {

TEST(PointCloudProcessingCommonTest, transform_point_test) {
  base::PointF pt_in;
  pt_in.x = 10.f;
  pt_in.y = 10.f;
  pt_in.z = 5.f;
  base::PointF pt_out;
  Eigen::Affine3d pose = Eigen::Affine3d::Identity();
  TransformPoint(pt_in, pose, &pt_out);
  EXPECT_NEAR(pt_in.x, 10.f, FLT_EPSILON);
  EXPECT_NEAR(pt_in.y, 10.f, FLT_EPSILON);
  EXPECT_NEAR(pt_in.z, 5.f, FLT_EPSILON);
}

TEST(PointCloudProcessingCommonTest, transform_point_cloud_test1) {
  base::PointFCloud cloud_in, cloud_out;
  base::PointF temp;
  temp.x = 10.f;
  temp.y = 10.f;
  temp.z = 5.f;
  cloud_in.push_back(temp);
  temp.x = 20.f;
  temp.y = -10.f;
  temp.z = 15.f;
  cloud_in.push_back(temp);
  Eigen::Affine3d pose = Eigen::Affine3d::Identity();
  TransformPointCloud<base::PointFCloud>(cloud_in, pose, &cloud_out);
  EXPECT_NEAR(cloud_out[0].x, 10.f, FLT_EPSILON);
  EXPECT_NEAR(cloud_out[0].y, 10.f, FLT_EPSILON);
  EXPECT_NEAR(cloud_out[0].z, 5.f, FLT_EPSILON);
  EXPECT_NEAR(cloud_out[1].x, 20.f, FLT_EPSILON);
  EXPECT_NEAR(cloud_out[1].y, -10.f, FLT_EPSILON);
  EXPECT_NEAR(cloud_out[1].z, 15.f, FLT_EPSILON);
}

TEST(PointCloudProcessingCommonTest, transform_point_cloud_test2) {
  base::PointFCloudPtr cloud_in(new base::PointFCloud);
  base::PointFCloudPtr cloud_out(new base::PointFCloud);
  base::PointF temp;
  temp.x = 10.f;
  temp.y = 10.f;
  temp.z = 5.f;
  cloud_in->push_back(temp);
  temp.x = 20.f;
  temp.y = -10.f;
  temp.z = 15.f;
  cloud_in->push_back(temp);
  Eigen::Affine3d pose = Eigen::Affine3d::Identity();
  TransformPointCloud<base::PointFCloud>(cloud_in, pose, cloud_out);
  EXPECT_NEAR(cloud_out->at(0).x, 10.f, FLT_EPSILON);
  EXPECT_NEAR(cloud_out->at(0).y, 10.f, FLT_EPSILON);
  EXPECT_NEAR(cloud_out->at(0).z, 5.f, FLT_EPSILON);
  EXPECT_NEAR(cloud_out->at(1).x, 20.f, FLT_EPSILON);
  EXPECT_NEAR(cloud_out->at(1).y, -10.f, FLT_EPSILON);
  EXPECT_NEAR(cloud_out->at(1).z, 15.f, FLT_EPSILON);
}

TEST(PointCloudProcessingCommonTest, transform_point_cloud_test3) {
  base::PointFCloud cloud_in_out;
  base::PointF temp;
  temp.x = 10.f;
  temp.y = 10.f;
  temp.z = 5.f;
  cloud_in_out.push_back(temp);
  temp.x = 20.f;
  temp.y = -10.f;
  temp.z = 15.f;
  cloud_in_out.push_back(temp);
  Eigen::Affine3d pose = Eigen::Affine3d::Identity();
  TransformPointCloud<base::PointFCloud>(pose, &cloud_in_out);
  EXPECT_NEAR(cloud_in_out[0].x, 10.f, FLT_EPSILON);
  EXPECT_NEAR(cloud_in_out[0].y, 10.f, FLT_EPSILON);
  EXPECT_NEAR(cloud_in_out[0].z, 5.f, FLT_EPSILON);
  EXPECT_NEAR(cloud_in_out[1].x, 20.f, FLT_EPSILON);
  EXPECT_NEAR(cloud_in_out[1].y, -10.f, FLT_EPSILON);
  EXPECT_NEAR(cloud_in_out[1].z, 15.f, FLT_EPSILON);
}

TEST(PointCloudProcessingCommonTest, transform_point_cloud_test4) {
  base::PointFCloudPtr cloud_in_out(new base::PointFCloud);
  base::PointF temp;
  temp.x = 10.f;
  temp.y = 10.f;
  temp.z = 5.f;
  cloud_in_out->push_back(temp);
  temp.x = 20.f;
  temp.y = -10.f;
  temp.z = 15.f;
  cloud_in_out->push_back(temp);
  Eigen::Affine3d pose = Eigen::Affine3d::Identity();
  TransformPointCloud<base::PointFCloud>(pose, cloud_in_out);
  EXPECT_NEAR(cloud_in_out->at(0).x, 10.f, FLT_EPSILON);
  EXPECT_NEAR(cloud_in_out->at(0).y, 10.f, FLT_EPSILON);
  EXPECT_NEAR(cloud_in_out->at(0).z, 5.f, FLT_EPSILON);
  EXPECT_NEAR(cloud_in_out->at(1).x, 20.f, FLT_EPSILON);
  EXPECT_NEAR(cloud_in_out->at(1).y, -10.f, FLT_EPSILON);
  EXPECT_NEAR(cloud_in_out->at(1).z, 15.f, FLT_EPSILON);
}

TEST(PointCloudProcessingCommonTest, extract_indiced_cloud_test) {
  base::PointFCloudPtr cloud_in(new base::PointFCloud);
  base::PointFCloudPtr cloud_out(new base::PointFCloud);
  base::PointF temp;
  temp.x = 10.f;
  temp.y = 10.f;
  temp.z = 5.f;
  cloud_in->push_back(temp);
  temp.x = 20.f;
  temp.y = -10.f;
  temp.z = 15.f;
  cloud_in->push_back(temp);
  std::vector<int> indices;
  indices.push_back(1);
  ExtractIndicedCloud<base::PointFCloud>(cloud_in, indices, cloud_out);
  EXPECT_NEAR(cloud_out->at(0).x, 20.f, FLT_EPSILON);
  EXPECT_NEAR(cloud_out->at(0).y, -10.f, FLT_EPSILON);
  EXPECT_NEAR(cloud_out->at(0).z, 15.f, FLT_EPSILON);
}

TEST(PointCloudProcessingCommonTest, get_min_max_in_3d_test) {
  base::PointF temp;
  base::PointFCloudPtr cloud(new base::PointFCloud);
  temp.x = 10.f;
  temp.y = 10.f;
  temp.z = 10.f;
  cloud->push_back(temp);
  temp.x = 20.f;
  temp.y = -10.f;
  temp.z = 5.f;
  cloud->push_back(temp);
  temp.x = -10.f;
  temp.y = 0.f;
  temp.z = 15.f;
  cloud->push_back(temp);
  base::PointIndices indices;
  indices.indices.push_back(0);
  indices.indices.push_back(2);
  Eigen::Vector4f min_pt, max_pt;
  GetMinMaxIn3D<base::PointFCloud>(cloud, indices, &min_pt, &max_pt);
  EXPECT_NEAR(min_pt(0), -10.f, FLT_EPSILON);
  EXPECT_NEAR(min_pt(1), 0.f, FLT_EPSILON);
  EXPECT_NEAR(min_pt(2), 10.f, FLT_EPSILON);
  EXPECT_NEAR(min_pt(3), 0.f, FLT_EPSILON);
  EXPECT_NEAR(max_pt(0), 10.f, FLT_EPSILON);
  EXPECT_NEAR(max_pt(1), 10.f, FLT_EPSILON);
  EXPECT_NEAR(max_pt(2), 15.f, FLT_EPSILON);
  EXPECT_NEAR(max_pt(3), 0.f, FLT_EPSILON);
  temp.x = std::numeric_limits<float>::quiet_NaN();
  temp.y = 10.f;
  temp.z = 10.f;
  cloud->push_back(temp);
  temp.x = 10.f;
  temp.y = std::numeric_limits<float>::quiet_NaN();
  temp.z = 10.f;
  cloud->push_back(temp);
  temp.x = 10.f;
  temp.y = 10.f;
  temp.z = std::numeric_limits<float>::quiet_NaN();
  cloud->push_back(temp);
  indices.indices.push_back(3);
  indices.indices.push_back(4);
  indices.indices.push_back(5);
  GetMinMaxIn3D<base::PointFCloud>(cloud, indices, &min_pt, &max_pt);
  EXPECT_NEAR(min_pt(0), -10.f, FLT_EPSILON);
  EXPECT_NEAR(min_pt(1), 0.f, FLT_EPSILON);
  EXPECT_NEAR(min_pt(2), 10.f, FLT_EPSILON);
  EXPECT_NEAR(min_pt(3), 0.f, FLT_EPSILON);
  EXPECT_NEAR(max_pt(0), 10.f, FLT_EPSILON);
  EXPECT_NEAR(max_pt(1), 10.f, FLT_EPSILON);
  EXPECT_NEAR(max_pt(2), 15.f, FLT_EPSILON);
  EXPECT_NEAR(max_pt(3), 0.f, FLT_EPSILON);
}

TEST(PointCloudProcessingCommonTest, get_min_max_in_3d_test2) {
  base::PointF temp;
  base::PointFCloudPtr cloud(new base::PointFCloud);
  temp.x = 10.f;
  temp.y = 10.f;
  temp.z = 10.f;
  cloud->push_back(temp);
  temp.x = 20.f;
  temp.y = -10.f;
  temp.z = 5.f;
  cloud->push_back(temp);
  temp.x = -10.f;
  temp.y = 0.f;
  temp.z = 15.f;
  cloud->push_back(temp);
  base::PointIndices indices;
  indices.indices.push_back(0);
  indices.indices.push_back(2);
  Eigen::Vector4f min_pt, max_pt;
  GetMinMaxIn3D<base::PointFCloud>(cloud, &min_pt, &max_pt);
  EXPECT_NEAR(min_pt(0), -10.f, FLT_EPSILON);
  EXPECT_NEAR(min_pt(1), -10.f, FLT_EPSILON);
  EXPECT_NEAR(min_pt(2), 5.f, FLT_EPSILON);
  EXPECT_NEAR(min_pt(3), 0.f, FLT_EPSILON);
  EXPECT_NEAR(max_pt(0), 20.f, FLT_EPSILON);
  EXPECT_NEAR(max_pt(1), 10.f, FLT_EPSILON);
  EXPECT_NEAR(max_pt(2), 15.f, FLT_EPSILON);
  EXPECT_NEAR(max_pt(3), 0.f, FLT_EPSILON);
  temp.x = std::numeric_limits<float>::quiet_NaN();
  temp.y = 10.f;
  temp.z = 10.f;
  cloud->push_back(temp);
  temp.x = 10.f;
  temp.y = std::numeric_limits<float>::quiet_NaN();
  temp.z = 10.f;
  cloud->push_back(temp);
  temp.x = 10.f;
  temp.y = 10.f;
  temp.z = std::numeric_limits<float>::quiet_NaN();
  cloud->push_back(temp);
  GetMinMaxIn3D<base::PointFCloud>(cloud, &min_pt, &max_pt);
  EXPECT_NEAR(min_pt(0), -10.f, FLT_EPSILON);
  EXPECT_NEAR(min_pt(1), -10.f, FLT_EPSILON);
  EXPECT_NEAR(min_pt(2), 5.f, FLT_EPSILON);
  EXPECT_NEAR(min_pt(3), 0.f, FLT_EPSILON);
  EXPECT_NEAR(max_pt(0), 20.f, FLT_EPSILON);
  EXPECT_NEAR(max_pt(1), 10.f, FLT_EPSILON);
  EXPECT_NEAR(max_pt(2), 15.f, FLT_EPSILON);
  EXPECT_NEAR(max_pt(3), 0.f, FLT_EPSILON);
}

TEST(PointCloudProcessingCommonTest, get_min_max_in_3d_test3) {
  base::PointF temp;
  base::PointFCloud cloud;
  temp.x = 10.f;
  temp.y = 10.f;
  temp.z = 10.f;
  cloud.push_back(temp);
  temp.x = 20.f;
  temp.y = -10.f;
  temp.z = 5.f;
  cloud.push_back(temp);
  temp.x = -10.f;
  temp.y = 0.f;
  temp.z = 15.f;
  cloud.push_back(temp);
  base::PointIndices indices;
  indices.indices.push_back(0);
  indices.indices.push_back(2);
  Eigen::Vector4f min_pt, max_pt;
  GetMinMaxIn3D<base::PointFCloud>(cloud, indices, &min_pt, &max_pt);
  EXPECT_NEAR(min_pt(0), -10.f, FLT_EPSILON);
  EXPECT_NEAR(min_pt(1), 0.f, FLT_EPSILON);
  EXPECT_NEAR(min_pt(2), 10.f, FLT_EPSILON);
  EXPECT_NEAR(min_pt(3), 0.f, FLT_EPSILON);
  EXPECT_NEAR(max_pt(0), 10.f, FLT_EPSILON);
  EXPECT_NEAR(max_pt(1), 10.f, FLT_EPSILON);
  EXPECT_NEAR(max_pt(2), 15.f, FLT_EPSILON);
  EXPECT_NEAR(max_pt(3), 0.f, FLT_EPSILON);
  temp.x = std::numeric_limits<float>::quiet_NaN();
  temp.y = 10.f;
  temp.z = 10.f;
  cloud.push_back(temp);
  temp.x = 10.f;
  temp.y = std::numeric_limits<float>::quiet_NaN();
  temp.z = 10.f;
  cloud.push_back(temp);
  temp.x = 10.f;
  temp.y = 10.f;
  temp.z = std::numeric_limits<float>::quiet_NaN();
  cloud.push_back(temp);
  indices.indices.push_back(3);
  indices.indices.push_back(4);
  indices.indices.push_back(5);
  GetMinMaxIn3D<base::PointFCloud>(cloud, indices, &min_pt, &max_pt);
  EXPECT_NEAR(min_pt(0), -10.f, FLT_EPSILON);
  EXPECT_NEAR(min_pt(1), 0.f, FLT_EPSILON);
  EXPECT_NEAR(min_pt(2), 10.f, FLT_EPSILON);
  EXPECT_NEAR(min_pt(3), 0.f, FLT_EPSILON);
  EXPECT_NEAR(max_pt(0), 10.f, FLT_EPSILON);
  EXPECT_NEAR(max_pt(1), 10.f, FLT_EPSILON);
  EXPECT_NEAR(max_pt(2), 15.f, FLT_EPSILON);
  EXPECT_NEAR(max_pt(3), 0.f, FLT_EPSILON);
}

TEST(PointCloudProcessingCommonTest, get_min_max_in_3d_test4) {
  base::PointF temp;
  base::PointFCloud cloud;
  temp.x = 10.f;
  temp.y = 10.f;
  temp.z = 10.f;
  cloud.push_back(temp);
  temp.x = 20.f;
  temp.y = -10.f;
  temp.z = 5.f;
  cloud.push_back(temp);
  temp.x = -10.f;
  temp.y = 0.f;
  temp.z = 15.f;
  cloud.push_back(temp);
  base::PointIndices indices;
  indices.indices.push_back(0);
  indices.indices.push_back(2);
  Eigen::Vector4f min_pt, max_pt;
  GetMinMaxIn3D<base::PointFCloud>(cloud, &min_pt, &max_pt);
  EXPECT_NEAR(min_pt(0), -10.f, FLT_EPSILON);
  EXPECT_NEAR(min_pt(1), -10.f, FLT_EPSILON);
  EXPECT_NEAR(min_pt(2), 5.f, FLT_EPSILON);
  EXPECT_NEAR(min_pt(3), 0.f, FLT_EPSILON);
  EXPECT_NEAR(max_pt(0), 20.f, FLT_EPSILON);
  EXPECT_NEAR(max_pt(1), 10.f, FLT_EPSILON);
  EXPECT_NEAR(max_pt(2), 15.f, FLT_EPSILON);
  EXPECT_NEAR(max_pt(3), 0.f, FLT_EPSILON);
  temp.x = std::numeric_limits<float>::quiet_NaN();
  temp.y = 10.f;
  temp.z = 10.f;
  cloud.push_back(temp);
  temp.x = 10.f;
  temp.y = std::numeric_limits<float>::quiet_NaN();
  temp.z = 10.f;
  cloud.push_back(temp);
  temp.x = 10.f;
  temp.y = 10.f;
  temp.z = std::numeric_limits<float>::quiet_NaN();
  cloud.push_back(temp);
  GetMinMaxIn3D<base::PointFCloud>(cloud, &min_pt, &max_pt);
  EXPECT_NEAR(min_pt(0), -10.f, FLT_EPSILON);
  EXPECT_NEAR(min_pt(1), -10.f, FLT_EPSILON);
  EXPECT_NEAR(min_pt(2), 5.f, FLT_EPSILON);
  EXPECT_NEAR(min_pt(3), 0.f, FLT_EPSILON);
  EXPECT_NEAR(max_pt(0), 20.f, FLT_EPSILON);
  EXPECT_NEAR(max_pt(1), 10.f, FLT_EPSILON);
  EXPECT_NEAR(max_pt(2), 15.f, FLT_EPSILON);
  EXPECT_NEAR(max_pt(3), 0.f, FLT_EPSILON);
}

TEST(PointCloudProcessingCommonTest, calculate_centroid_test) {
  base::PointFCloudPtr cloud(new base::PointFCloud);
  base::PointF temp;
  temp.x = 10.f;
  temp.y = 10.f;
  temp.z = 5.f;
  cloud->push_back(temp);
  temp.x = 15.f;
  temp.y = -10.f;
  temp.z = 15.f;
  cloud->push_back(temp);
  temp.x = -10.f;
  temp.y = 15.f;
  temp.z = -5.f;
  cloud->push_back(temp);
  Eigen::Vector3f result = CalculateCentroid<base::PointFCloud>(*cloud);
  EXPECT_NEAR(result(0), 5.f, FLT_EPSILON);
  EXPECT_NEAR(result(1), 5.f, FLT_EPSILON);
  EXPECT_NEAR(result(2), 5.f, FLT_EPSILON);
}

}  // namespace common
}  // namespace perception
}  // namespace apollo
