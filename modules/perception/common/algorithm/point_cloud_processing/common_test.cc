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

#include "modules/perception/common/algorithm/point_cloud_processing/common.h"

#include "gtest/gtest.h"

namespace apollo {
namespace perception {
namespace algorithm {

using base::AttributePointCloud;
using base::PointF;

TEST(PointCloudProcessingCommonTest, transform_point_test) {
  PointF pt_in;
  pt_in.x = 10.f;
  pt_in.y = 10.f;
  pt_in.z = 5.f;
  PointF pt_out;
  Eigen::Affine3d pose = Eigen::Affine3d::Identity();
  TransformPoint(pt_in, pose, &pt_out);
  EXPECT_NEAR(pt_in.x, 10.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(pt_in.y, 10.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(pt_in.z, 5.f, std::numeric_limits<float>::epsilon());
}

TEST(PointCloudProcessingCommonTest, transform_point_cloud_test1) {
  AttributePointCloud<PointF> cloud_in, cloud_out;
  PointF temp;
  temp.x = 10.f;
  temp.y = 10.f;
  temp.z = 5.f;
  cloud_in.push_back(temp);
  temp.x = 20.f;
  temp.y = -10.f;
  temp.z = 15.f;
  cloud_in.push_back(temp);
  Eigen::Affine3d pose = Eigen::Affine3d::Identity();
  TransformPointCloud<PointF>(cloud_in, pose, &cloud_out);
  EXPECT_NEAR(cloud_out[0].x, 10.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(cloud_out[0].y, 10.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(cloud_out[0].z, 5.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(cloud_out[1].x, 20.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(cloud_out[1].y, -10.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(cloud_out[1].z, 15.f, std::numeric_limits<float>::epsilon());
}

TEST(PointCloudProcessingCommonTest, transform_point_cloud_test2) {
  std::shared_ptr<AttributePointCloud<PointF>> cloud_in(
      new AttributePointCloud<PointF>);
  std::shared_ptr<AttributePointCloud<PointF>> cloud_out(
      new AttributePointCloud<PointF>);
  PointF temp;
  temp.x = 10.f;
  temp.y = 10.f;
  temp.z = 5.f;
  cloud_in->push_back(temp);
  temp.x = 20.f;
  temp.y = -10.f;
  temp.z = 15.f;
  cloud_in->push_back(temp);
  Eigen::Affine3d pose = Eigen::Affine3d::Identity();
  TransformPointCloud<PointF>(*cloud_in, pose, cloud_out.get());
  EXPECT_NEAR(cloud_out->at(0).x, 10.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(cloud_out->at(0).y, 10.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(cloud_out->at(0).z, 5.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(cloud_out->at(1).x, 20.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(cloud_out->at(1).y, -10.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(cloud_out->at(1).z, 15.f, std::numeric_limits<float>::epsilon());
}

TEST(PointCloudProcessingCommonTest, transform_point_cloud_test3) {
  AttributePointCloud<PointF> cloud_in_out;
  PointF temp;
  temp.x = 10.f;
  temp.y = 10.f;
  temp.z = 5.f;
  cloud_in_out.push_back(temp);
  temp.x = 20.f;
  temp.y = -10.f;
  temp.z = 15.f;
  cloud_in_out.push_back(temp);
  Eigen::Affine3d pose = Eigen::Affine3d::Identity();
  TransformPointCloud<PointF>(pose, &cloud_in_out);
  EXPECT_NEAR(cloud_in_out[0].x, 10.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(cloud_in_out[0].y, 10.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(cloud_in_out[0].z, 5.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(cloud_in_out[1].x, 20.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(cloud_in_out[1].y, -10.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(cloud_in_out[1].z, 15.f, std::numeric_limits<float>::epsilon());
}

TEST(PointCloudProcessingCommonTest, transform_point_cloud_test4) {
  std::shared_ptr<AttributePointCloud<PointF>> cloud_in_out(
      new AttributePointCloud<PointF>);
  PointF temp;
  temp.x = 10.f;
  temp.y = 10.f;
  temp.z = 5.f;
  cloud_in_out->push_back(temp);
  temp.x = 20.f;
  temp.y = -10.f;
  temp.z = 15.f;
  cloud_in_out->push_back(temp);
  Eigen::Affine3d pose = Eigen::Affine3d::Identity();
  TransformPointCloud<PointF>(pose, cloud_in_out.get());
  EXPECT_NEAR(cloud_in_out->at(0).x, 10.f,
              std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(cloud_in_out->at(0).y, 10.f,
              std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(cloud_in_out->at(0).z, 5.f,
              std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(cloud_in_out->at(1).x, 20.f,
              std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(cloud_in_out->at(1).y, -10.f,
              std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(cloud_in_out->at(1).z, 15.f,
              std::numeric_limits<float>::epsilon());
}

TEST(PointCloudProcessingCommonTest, extract_indiced_cloud_test) {
  std::shared_ptr<AttributePointCloud<PointF>> cloud_in(
      new AttributePointCloud<PointF>);
  std::shared_ptr<AttributePointCloud<PointF>> cloud_out(
      new AttributePointCloud<PointF>);
  PointF temp;
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
  ExtractIndicedCloud<AttributePointCloud<PointF>>(cloud_in, indices,
                                                   cloud_out);
  EXPECT_NEAR(cloud_out->at(0).x, 20.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(cloud_out->at(0).y, -10.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(cloud_out->at(0).z, 15.f, std::numeric_limits<float>::epsilon());
}

TEST(PointCloudProcessingCommonTest, get_min_max_in_3d_test) {
  PointF temp;
  std::shared_ptr<AttributePointCloud<PointF>> cloud(
      new AttributePointCloud<PointF>);
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
  GetMinMaxIn3D<PointF>(*cloud, indices, &min_pt, &max_pt);
  EXPECT_NEAR(min_pt(0), -10.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(min_pt(1), 0.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(min_pt(2), 10.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(min_pt(3), 0.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(max_pt(0), 10.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(max_pt(1), 10.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(max_pt(2), 15.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(max_pt(3), 0.f, std::numeric_limits<float>::epsilon());
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
  GetMinMaxIn3D<PointF>(*cloud, indices, &min_pt, &max_pt);
  EXPECT_NEAR(min_pt(0), -10.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(min_pt(1), 0.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(min_pt(2), 10.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(min_pt(3), 0.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(max_pt(0), 10.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(max_pt(1), 10.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(max_pt(2), 15.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(max_pt(3), 0.f, std::numeric_limits<float>::epsilon());
}

TEST(PointCloudProcessingCommonTest, get_min_max_in_3d_test2) {
  PointF temp;
  std::shared_ptr<AttributePointCloud<PointF>> cloud(
      new AttributePointCloud<PointF>);
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
  GetMinMaxIn3D<PointF>(*cloud, &min_pt, &max_pt);
  EXPECT_NEAR(min_pt(0), -10.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(min_pt(1), -10.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(min_pt(2), 5.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(min_pt(3), 0.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(max_pt(0), 20.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(max_pt(1), 10.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(max_pt(2), 15.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(max_pt(3), 0.f, std::numeric_limits<float>::epsilon());
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
  GetMinMaxIn3D<PointF>(*cloud, &min_pt, &max_pt);
  EXPECT_NEAR(min_pt(0), -10.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(min_pt(1), -10.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(min_pt(2), 5.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(min_pt(3), 0.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(max_pt(0), 20.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(max_pt(1), 10.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(max_pt(2), 15.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(max_pt(3), 0.f, std::numeric_limits<float>::epsilon());
}

TEST(PointCloudProcessingCommonTest, get_min_max_in_3d_test3) {
  PointF temp;
  AttributePointCloud<PointF> cloud;
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
  GetMinMaxIn3D<PointF>(cloud, indices, &min_pt, &max_pt);
  EXPECT_NEAR(min_pt(0), -10.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(min_pt(1), 0.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(min_pt(2), 10.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(min_pt(3), 0.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(max_pt(0), 10.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(max_pt(1), 10.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(max_pt(2), 15.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(max_pt(3), 0.f, std::numeric_limits<float>::epsilon());
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
  GetMinMaxIn3D<PointF>(cloud, indices, &min_pt, &max_pt);
  EXPECT_NEAR(min_pt(0), -10.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(min_pt(1), 0.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(min_pt(2), 10.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(min_pt(3), 0.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(max_pt(0), 10.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(max_pt(1), 10.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(max_pt(2), 15.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(max_pt(3), 0.f, std::numeric_limits<float>::epsilon());
}

TEST(PointCloudProcessingCommonTest, get_min_max_in_3d_test4) {
  PointF temp;
  AttributePointCloud<PointF> cloud;
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
  GetMinMaxIn3D<PointF>(cloud, &min_pt, &max_pt);
  EXPECT_NEAR(min_pt(0), -10.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(min_pt(1), -10.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(min_pt(2), 5.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(min_pt(3), 0.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(max_pt(0), 20.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(max_pt(1), 10.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(max_pt(2), 15.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(max_pt(3), 0.f, std::numeric_limits<float>::epsilon());
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
  GetMinMaxIn3D<PointF>(cloud, &min_pt, &max_pt);
  EXPECT_NEAR(min_pt(0), -10.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(min_pt(1), -10.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(min_pt(2), 5.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(min_pt(3), 0.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(max_pt(0), 20.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(max_pt(1), 10.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(max_pt(2), 15.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(max_pt(3), 0.f, std::numeric_limits<float>::epsilon());
}

TEST(PointCloudProcessingCommonTest, calculate_centroid_test) {
  std::shared_ptr<AttributePointCloud<PointF>> cloud(
      new AttributePointCloud<PointF>);
  PointF temp;
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
  Eigen::Matrix<float, 3, 1> result = CalculateCentroid<float>(*cloud);
  EXPECT_NEAR(result(0), 5.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(result(1), 5.f, std::numeric_limits<float>::epsilon());
  EXPECT_NEAR(result(2), 5.f, std::numeric_limits<float>::epsilon());
}

}  // namespace algorithm
}  // namespace perception
}  // namespace apollo
