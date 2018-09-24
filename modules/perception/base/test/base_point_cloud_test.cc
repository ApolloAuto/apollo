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
#include <gtest/gtest.h>
#include <limits>
#include "modules/perception/base/point_cloud_types.h"

namespace apollo {
namespace perception {
namespace base {

TEST(PointTest, point_test) {
  {
    PointXYZIF point;
    EXPECT_EQ(point.x, 0.f);
    EXPECT_EQ(point.y, 0.f);
    EXPECT_EQ(point.z, 0.f);
    EXPECT_EQ(point.intensity, 0.f);
  }
  {
    PointXYZID point;
    EXPECT_EQ(point.x, 0.0);
    EXPECT_EQ(point.y, 0.0);
    EXPECT_EQ(point.z, 0.0);
    EXPECT_EQ(point.intensity, 0.0);
  }
  {
    PointXYZITF point;
    EXPECT_EQ(point.x, 0.f);
    EXPECT_EQ(point.timestamp, 0.0);
  }
  {
    PointXYZITD point;
    EXPECT_EQ(point.x, 0.0);
    EXPECT_EQ(point.timestamp, 0.0);
  }
  {
    PointXYZITHF point;
    EXPECT_EQ(point.x, 0.f);
    EXPECT_EQ(point.timestamp, 0.0);
    EXPECT_EQ(point.height, FLT_MAX);
  }
  {
    PointXYZITHD point;
    EXPECT_EQ(point.x, 0.0);
    EXPECT_EQ(point.timestamp, 0.0);
    EXPECT_EQ(point.height, FLT_MAX);
  }
  {
    PointXYZITHBF point;
    EXPECT_EQ(point.x, 0.f);
    EXPECT_EQ(point.timestamp, 0.0);
    EXPECT_EQ(point.height, FLT_MAX);
    EXPECT_EQ(point.beam_id, -1);
  }
  {
    PointXYZITHBD point;
    EXPECT_EQ(point.x, 0.0);
    EXPECT_EQ(point.timestamp, 0.0);
    EXPECT_EQ(point.height, FLT_MAX);
    EXPECT_EQ(point.beam_id, -1);
  }
  {
    PointXYZITHBLF point;
    EXPECT_EQ(point.x, 0.f);
    EXPECT_EQ(point.timestamp, 0.0);
    EXPECT_EQ(point.height, FLT_MAX);
    EXPECT_EQ(point.beam_id, -1);
    EXPECT_EQ(point.label, 0);
  }
  {
    PointXYZITHBLD point;
    EXPECT_EQ(point.x, 0.0);
    EXPECT_EQ(point.timestamp, 0.0);
    EXPECT_EQ(point.height, FLT_MAX);
    EXPECT_EQ(point.beam_id, -1);
    EXPECT_EQ(point.label, 0);
  }
}

TEST(PointIndicesTest, point_test) {
  PointIndices indices;
  EXPECT_EQ(indices.indices.capacity(), kDefaultReservePointNum);
}

TEST(PointCloudTest, point_cloud_constructor_test) {
  {
    typedef PointCloud<PointF> TestPointCloud;
    TestPointCloud cloud1;
    EXPECT_EQ(cloud1.width_, 0);
    EXPECT_EQ(cloud1.height_, 0);
    EXPECT_EQ(cloud1.size(), 0);
    EXPECT_TRUE(cloud1.empty());
    TestPointCloud cloud2(cloud1);
    EXPECT_EQ(cloud2.width_, 0);
    EXPECT_EQ(cloud2.height_, 0);
    EXPECT_EQ(cloud2.size(), 0);
    cloud2.resize(10);
    EXPECT_EQ(cloud2.width_, 10);
    EXPECT_EQ(cloud2.height_, 1);
    EXPECT_EQ(cloud2.size(), 10);
    PointIndices indices;
    indices.indices = {0, 3, 5};
    EXPECT_EQ(indices.indices.size(), 3);
    TestPointCloud cloud3(cloud2, indices);
    EXPECT_EQ(cloud3.width(), 3);
    EXPECT_EQ(cloud3.height(), 1);
    TestPointCloud cloud4(cloud2, indices.indices);
    EXPECT_EQ(cloud4.width(), 3);
    EXPECT_EQ(cloud4.height(), 1);
    TestPointCloud cloud5(6, 7);
    EXPECT_TRUE(cloud5.IsOrganized());
    EXPECT_EQ(cloud5.width(), 6);
    EXPECT_EQ(cloud5.height(), 7);
    cloud5 += cloud4;
    EXPECT_EQ(cloud5.width(), 45);
    EXPECT_EQ(cloud5.height(), 1);
  }
  {
    typedef AttributePointCloud<PointF> TestPointCloud;
    TestPointCloud cloud1;
    EXPECT_EQ(cloud1.width_, 0);
    EXPECT_EQ(cloud1.height_, 0);
    EXPECT_EQ(cloud1.size(), 0);
    EXPECT_TRUE(cloud1.empty());
    TestPointCloud cloud2(cloud1);
    EXPECT_EQ(cloud2.width_, 0);
    EXPECT_EQ(cloud2.height_, 0);
    EXPECT_EQ(cloud2.size(), 0);
    cloud2.resize(10);
    EXPECT_EQ(cloud2.width_, 10);
    EXPECT_EQ(cloud2.height_, 1);
    EXPECT_EQ(cloud2.size(), 10);
    PointIndices indices;
    indices.indices = {0, 3, 5};
    EXPECT_EQ(indices.indices.size(), 3);
    TestPointCloud cloud3(cloud2, indices);
    EXPECT_EQ(cloud3.width(), 3);
    EXPECT_EQ(cloud3.height(), 1);
    TestPointCloud cloud4(cloud2, indices.indices);
    EXPECT_EQ(cloud4.width(), 3);
    EXPECT_EQ(cloud4.height(), 1);
    TestPointCloud cloud5(6, 7);
    EXPECT_TRUE(cloud5.IsOrganized());
    EXPECT_EQ(cloud5.width(), 6);
    EXPECT_EQ(cloud5.height(), 7);
    cloud5 += cloud4;
    EXPECT_EQ(cloud5.width(), 45);
    EXPECT_EQ(cloud5.height(), 1);
  }
}

TEST(PointCloudTest, point_cloud_interface_test) {
  typedef PointCloud<PointF> TestPointCloud;
  TestPointCloud cloud;
  cloud.reserve(2);
  cloud.resize(2);
  EXPECT_FALSE(cloud.IsOrganized());
  TestPointCloud organized_cloud(2, 2);
  organized_cloud.resize(4);
  EXPECT_TRUE(organized_cloud.IsOrganized());
  auto const_check_null = [](const TestPointCloud& cloud) {
    EXPECT_EQ(cloud.at(0, 0), nullptr);
    EXPECT_EQ(cloud(0, 0), nullptr);
  };
  auto const_check_not_null = [](const TestPointCloud& cloud) {
    EXPECT_NE(cloud.at(0, 0), nullptr);
    EXPECT_NE(cloud(0, 0), nullptr);
  };
  EXPECT_NE(organized_cloud.at(0, 0), nullptr);
  EXPECT_NE(organized_cloud(0, 0), nullptr);
  const_check_not_null(organized_cloud);
  cloud.push_back(PointF());
  EXPECT_EQ(cloud.at(0, 0), nullptr);
  EXPECT_EQ(cloud(0, 0), nullptr);
  const_check_null(cloud);

  EXPECT_EQ(cloud.size(), 3);
  auto const_check_eq = [](const TestPointCloud& cloud) {
    EXPECT_EQ(&(cloud[0]), &(cloud.front()));
    EXPECT_EQ(&(cloud[2]), &(cloud.back()));
  };
  EXPECT_EQ(&(cloud[0]), &(cloud.front()));
  EXPECT_EQ(&(cloud[2]), &(cloud.back()));
  const_check_eq(cloud);

  for (TestPointCloud::iterator it = cloud.begin(); it != cloud.end(); ++it) {
    EXPECT_EQ(it->x, 0.f);
  }
  for (TestPointCloud::const_iterator it = cloud.begin(); it != cloud.end();
       ++it) {
    EXPECT_EQ(it->x, 0.f);
  }

  cloud.clear();
  EXPECT_EQ(cloud.size(), 0);

  cloud.resize(1);
  EXPECT_FALSE(cloud.SwapPoint(0, 3));
  EXPECT_FALSE(cloud.SwapPoint(3, 0));
  cloud.resize(2);
  cloud[0].x = 1.f;
  cloud[1].x = 2.f;
  EXPECT_TRUE(cloud.SwapPoint(0, 1));
  EXPECT_EQ(cloud[0].x, 2.f);
  EXPECT_EQ(cloud[1].x, 1.f);

  TestPointCloud cloud2;
  cloud2.resize(1);
  cloud2[0].x = 10.f;
  EXPECT_FALSE(cloud.CopyPoint(0, 10, cloud2));
  EXPECT_TRUE(cloud.CopyPoint(0, 0, cloud2));
  EXPECT_EQ(cloud[0].x, 10.f);

  cloud.resize(1);
  cloud.set_timestamp(10.0);
  Eigen::Affine3d pose = Eigen::Affine3d::Identity();
  pose.translate(Eigen::Vector3d(20.0, 0, 0));
  cloud.set_sensor2world_pose(pose);

  cloud2.clear();
  cloud2.SwapPointCloud(&cloud);

  EXPECT_EQ(cloud2.size(), 1);
  EXPECT_NEAR(cloud2.get_timestamp(), 10.0, 1e-9);
  EXPECT_NEAR(cloud2.get_sensor2world_pose().translation()(0), 20.0, 1e-9);
}

TEST(PointCloudTest, attribute_point_cloud_interface_test) {
  typedef AttributePointCloud<PointF> TestPointCloud;
  TestPointCloud cloud;
  cloud.reserve(2);
  cloud.resize(2);
  cloud.push_back(PointF());
  cloud.push_back(PointF(), 1.0, 0.5f, 1, 2);
  auto const_check_eq = [](const TestPointCloud& cloud) {
    EXPECT_EQ(cloud.points_timestamp(3), 1.0);
    EXPECT_EQ(cloud.points_height(3), 0.5f);
    EXPECT_EQ(cloud.points_beam_id(3), 1);
    EXPECT_EQ(cloud.points_timestamp(0, 0), nullptr);
    EXPECT_EQ(cloud.points_height(0, 0), nullptr);
    EXPECT_EQ(cloud.points_beam_id(0, 0), nullptr);
    EXPECT_EQ(cloud.points_label(0, 0), nullptr);
  };
  EXPECT_EQ(cloud.points_timestamp(3), 1.0);
  EXPECT_EQ(cloud.points_height(3), 0.5f);
  EXPECT_EQ(cloud.points_beam_id(3), 1);
  EXPECT_EQ(cloud.points_label(3), 2);
  const_check_eq(cloud);

  TestPointCloud organized_cloud(2, 2);
  organized_cloud.reserve(4);
  organized_cloud.resize(4);
  EXPECT_TRUE(organized_cloud.IsOrganized());
  organized_cloud.points_timestamp(3) = 1.0;
  organized_cloud.points_height(3) = 0.5f;
  organized_cloud.points_beam_id(3) = 1;
  organized_cloud.points_label(3) = 2;
  auto const_check_eq_organized = [](const TestPointCloud& cloud) {
    EXPECT_EQ(cloud.points_timestamp(3), 1.0);
    EXPECT_EQ(cloud.points_height(3), 0.5f);
    EXPECT_EQ(cloud.points_beam_id(3), 1);
    EXPECT_EQ(cloud.points_label(3), 2);
    EXPECT_NE(cloud.points_timestamp(0, 0), nullptr);
    EXPECT_NE(cloud.points_height(0, 0), nullptr);
    EXPECT_NE(cloud.points_beam_id(0, 0), nullptr);
    EXPECT_NE(cloud.points_label(0, 0), nullptr);
  };
  EXPECT_EQ(organized_cloud.points_timestamp(3), 1.0);
  EXPECT_EQ(organized_cloud.points_height(3), 0.5f);
  EXPECT_EQ(organized_cloud.points_beam_id(3), 1);
  EXPECT_EQ(organized_cloud.points_label(3), 2);
  const_check_eq_organized(organized_cloud);

  cloud.clear();
  EXPECT_EQ(cloud.size(), 0);

  cloud.resize(1);
  EXPECT_FALSE(cloud.SwapPoint(0, 3));
  EXPECT_FALSE(cloud.SwapPoint(3, 0));

  cloud.resize(2);
  cloud[0].x = 1.f;
  cloud.points_timestamp(0) = 0.0;
  cloud.points_height(0) = 0.f;
  cloud.points_beam_id(0) = 0;
  cloud.points_label(0) = 0;
  cloud[1].x = 2.f;
  cloud.points_timestamp(1) = 1.0;
  cloud.points_height(1) = 1.f;
  cloud.points_beam_id(1) = 1;
  cloud.points_label(1) = 1;

  EXPECT_TRUE(cloud.SwapPoint(0, 1));
  EXPECT_EQ(cloud[0].x, 2.f);
  EXPECT_EQ(cloud.points_timestamp(0), 1.0);
  EXPECT_EQ(cloud.points_height(0), 1.f);
  EXPECT_EQ(cloud.points_beam_id(0), 1);
  EXPECT_EQ(cloud.points_label(0), 1);
  EXPECT_EQ(cloud[1].x, 1.f);
  EXPECT_EQ(cloud.points_timestamp(1), 0.0);
  EXPECT_EQ(cloud.points_height(1), 0.f);
  EXPECT_EQ(cloud.points_beam_id(1), 0);
  EXPECT_EQ(cloud.points_label(1), 0);

  TestPointCloud cloud2;
  cloud2.resize(1);
  cloud2[0].x = 10.f;
  cloud2.points_timestamp(0) = 10.0;
  cloud2.points_height(0) = 10.f;
  cloud2.points_beam_id(0) = 10;
  cloud2.points_label(0) = 10;

  EXPECT_FALSE(cloud.CopyPoint(0, 10, cloud2));
  EXPECT_TRUE(cloud.CopyPoint(0, 0, cloud2));
  EXPECT_EQ(cloud[0].x, 10.f);
  EXPECT_EQ(cloud.points_timestamp(0), 10.0);
  EXPECT_EQ(cloud.points_height(0), 10.f);
  EXPECT_EQ(cloud.points_beam_id(0), 10);
  EXPECT_EQ(cloud.points_label(0), 10);

  cloud.resize(1);
  cloud.set_timestamp(10.0);
  Eigen::Affine3d pose = Eigen::Affine3d::Identity();
  pose.translate(Eigen::Vector3d(20.0, 0, 0));
  cloud.set_sensor2world_pose(pose);
  cloud.points_timestamp(1) = 2.0;
  cloud.points_height(1) = 3.f;
  cloud.points_beam_id(1) = 4;
  cloud.points_label(1) = 5;

  cloud2.clear();
  cloud2.SwapPointCloud(&cloud);

  EXPECT_EQ(cloud2.size(), 1);
  EXPECT_NEAR(cloud2.get_timestamp(), 10.0, 1e-9);
  EXPECT_NEAR(cloud2.get_sensor2world_pose().translation()(0), 20.0, 1e-9);
  EXPECT_NEAR(cloud2.points_timestamp(1), 2.0, 1e-9);
  EXPECT_NEAR(cloud2.points_height(1), 3.f, 1e-9);
  EXPECT_EQ(cloud2.points_beam_id(1), 4);
  EXPECT_EQ(cloud2.points_label(1), 5);
}

TEST(PointCloudTest, transform_test) {
  Eigen::Affine3d affine = Eigen::Affine3d::Identity();
  affine.rotate(Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d(1, 0, 0)));
  affine.translate(Eigen::Vector3d(1, 1, 1));
  typedef AttributePointCloud<PointF> TestPointCloud;
  TestPointCloud cloud(4, 4);
  cloud.set_timestamp(1.0);
  EXPECT_EQ(cloud.get_timestamp(), 1.0);
  cloud.set_sensor2world_pose(affine);
  EXPECT_EQ((cloud.get_sensor2world_pose().matrix() - affine.matrix()).trace(),
            0.0);
  cloud.RotatePointCloud(false);
  EXPECT_EQ(cloud.get_sensor2world_pose().linear().trace(), 3.0);
  cloud.TransformPointCloud(false);
  EXPECT_EQ(cloud.get_sensor2world_pose().translation().x(), 0.0);
  EXPECT_EQ(cloud.get_sensor2world_pose().translation().y(), 0.0);
  EXPECT_EQ(cloud.get_sensor2world_pose().translation().z(), 0.0);
  EXPECT_EQ(cloud.get_sensor2world_pose().matrix().determinant(), 1.0);

  cloud[0].x = std::numeric_limits<float>::quiet_NaN();
  cloud[1].y = std::numeric_limits<float>::quiet_NaN();
  cloud[2].z = std::numeric_limits<float>::quiet_NaN();
  cloud.RotatePointCloud(true);
  EXPECT_EQ(cloud.get_sensor2world_pose().linear().trace(), 3.0);
  cloud.TransformPointCloud(true);
  EXPECT_EQ(cloud.get_sensor2world_pose().translation().x(), 0.0);
  EXPECT_EQ(cloud.get_sensor2world_pose().translation().y(), 0.0);
  EXPECT_EQ(cloud.get_sensor2world_pose().translation().z(), 0.0);
  EXPECT_EQ(cloud.get_sensor2world_pose().matrix().determinant(), 1.0);
}

template <typename PointT>
void cloud_check(const std::shared_ptr<const PointCloud<PointT>> cloud) {
  for (auto& point : *cloud) {
    EXPECT_EQ(point.x, 0.f);
    EXPECT_EQ(point.y, 0.f);
    EXPECT_EQ(point.z, 0.f);
  }
}

template <typename PointT>
void resize_cloud(const std::shared_ptr<PointCloud<PointT>> cloud) {
  cloud->resize(cloud->size() * 2);
}

TEST(PointCloudTest, dynamic_binding_test) {
  std::shared_ptr<PointCloud<PointF>> cloud;
  cloud.reset(new PointCloud<PointF>);
  cloud->resize(10);
  cloud_check<PointF>(cloud);
  EXPECT_TRUE(cloud->CheckConsistency());

  std::shared_ptr<AttributePointCloud<PointF>> attribute_cloud;
  attribute_cloud.reset(new AttributePointCloud<PointF>);
  attribute_cloud->resize(10);
  cloud_check<PointF>(attribute_cloud);
  resize_cloud<PointF>(attribute_cloud);
  EXPECT_EQ(attribute_cloud->size(), 20);
  EXPECT_TRUE(attribute_cloud->CheckConsistency());
}

}  // namespace base
}  // namespace perception
}  // namespace apollo
