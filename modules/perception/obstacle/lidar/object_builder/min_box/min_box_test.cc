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

#include "modules/perception/obstacle/lidar/object_builder/min_box/min_box.h"

#include <fstream>

#include "gtest/gtest.h"

namespace apollo {
namespace perception {

class MinBoxObjectBuilderTest : public testing::Test {
 protected:
  MinBoxObjectBuilderTest() {}
  ~MinBoxObjectBuilderTest() {}
  void SetUp() {
    min_box_object_builder_ = new MinBoxObjectBuilder();
  }
  void TearDown() {
    delete min_box_object_builder_;
    min_box_object_builder_ = nullptr;
  }

 protected:
  MinBoxObjectBuilder* min_box_object_builder_ = nullptr;
};

bool ConstructPointCloud(std::vector<ObjectPtr>* objects) {
  std::string pcd_data(
      "modules/perception/data/min_box_object_builder_test/"
      "QB9178_3_1461381834_1461382134_30651.pcd");
  std::ifstream cluster_ifs(pcd_data.c_str(), std::ifstream::in);
  std::string point_buf;
  while (cluster_ifs.good()) {
    getline(cluster_ifs, point_buf);
    std::stringstream ss;
    ss << point_buf;
    int point_num = 0;
    ss >> point_num;
    if (point_num <= 0) {
      continue;
    }
    uint64_t intensity;
    pcl_util::PointCloudPtr cluster_cloud(new pcl_util::PointCloud);
    for (int i = 0; i < point_num; ++i) {
      pcl_util::Point p;
      ss >> p.x >> p.y >> p.z >> intensity;
      p.intensity = static_cast<uint8_t>(intensity);
      cluster_cloud->points.push_back(p);
    }
    ObjectPtr object(new Object);
    object->cloud = cluster_cloud;
    objects->push_back(object);
  }
  return true;
}

TEST_F(MinBoxObjectBuilderTest, build) {
  std::vector<ObjectPtr> objects;
  ConstructPointCloud(&objects);
  EXPECT_EQ(5, objects.size());
  ObjectBuilderOptions options;
  EXPECT_TRUE(min_box_object_builder_->Build(options, &objects));
  const double EPSILON = 1e-6;
  // obj 1
  EXPECT_EQ(3, objects[0]->cloud->points.size());
  EXPECT_NEAR(0.1499978, objects[0]->length, EPSILON);
  EXPECT_NEAR(0.0560999, objects[0]->width, EPSILON);
  EXPECT_NEAR(0.7320720, objects[0]->height, EPSILON);
  EXPECT_NEAR(0.0, objects[0]->direction[0], EPSILON);
  EXPECT_NEAR(1.0, objects[0]->direction[1], EPSILON);
  EXPECT_NEAR(0.0, objects[0]->direction[2], EPSILON);

  // obj 2
  EXPECT_EQ(76, objects[1]->cloud->points.size());
  EXPECT_NEAR(3.740892, objects[1]->length, EPSILON);
  EXPECT_NEAR(0.387848, objects[1]->width, EPSILON);
  EXPECT_NEAR(1.855707, objects[1]->height, EPSILON);
  EXPECT_NEAR(-0.9987907, objects[1]->direction[0], EPSILON);
  EXPECT_NEAR(-0.0491651, objects[1]->direction[1], EPSILON);
  EXPECT_NEAR(0.0, objects[1]->direction[2], EPSILON);

  // obj 3
  EXPECT_EQ(4, objects[2]->cloud->points.size());
  EXPECT_NEAR(0.693386, objects[2]->length, EPSILON);
  EXPECT_NEAR(0.278326, objects[2]->width, EPSILON);
  EXPECT_NEAR(0.0199099, objects[2]->height, EPSILON);
  EXPECT_NEAR(0.743031, objects[2]->direction[0], EPSILON);
  EXPECT_NEAR(-0.669257, objects[2]->direction[1], EPSILON);
  EXPECT_NEAR(0.0, objects[2]->direction[2], EPSILON);

  // obj 4
  EXPECT_EQ(7, objects[3]->cloud->points.size());
  EXPECT_NEAR(0.688922, objects[3]->length, EPSILON);
  EXPECT_NEAR(0.356031, objects[3]->width, EPSILON);
  EXPECT_NEAR(0.311353, objects[3]->height, EPSILON);
  EXPECT_NEAR(0.992437, objects[3]->direction[0], EPSILON);
  EXPECT_NEAR(0.122754, objects[3]->direction[1], EPSILON);
  EXPECT_NEAR(0.0, objects[3]->direction[2], EPSILON);

  // obj 5
  EXPECT_EQ(11, objects[4]->cloud->points.size());
  EXPECT_NEAR(1.147774, objects[4]->length, EPSILON);
  EXPECT_NEAR(0.326984, objects[4]->width, EPSILON);
  EXPECT_NEAR(0.977219, objects[4]->height, EPSILON);
  EXPECT_NEAR(0.979983, objects[4]->direction[0], EPSILON);
  EXPECT_NEAR(-0.199081, objects[4]->direction[1], EPSILON);
  EXPECT_NEAR(0.0, objects[4]->direction[2], EPSILON);
}

}  // namespace perception
}  // namespace apollo
