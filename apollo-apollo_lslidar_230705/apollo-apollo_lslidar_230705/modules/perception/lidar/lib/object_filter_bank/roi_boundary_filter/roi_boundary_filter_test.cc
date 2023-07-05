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
#include "gtest/gtest.h"

#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/lidar/lib/object_filter_bank/roi_boundary_filter/roi_boundary_filter.h"

namespace apollo {
namespace perception {
namespace lidar {

class ROIBoundaryFilterTest : public testing::Test {
 protected:
  void SetUp() {
    char cyber_path[100] = "CYBER_PATH=";
    putenv(cyber_path);
    char module_path[100] = "MODULE_PATH=";
    putenv(module_path);
    FLAGS_work_root =
        "/apollo/modules/perception/testdata/lidar/lib/object_filter_bank/"
        "roi_boundary";
  }
  void TearDown() {}

 protected:
  ROIBoundaryFilter filter_;
};

void MockLidarFrame(LidarFrame* frame) {
  frame->hdmap_struct.reset(new base::HdmapStruct);
  frame->hdmap_struct->road_polygons.resize(1);
  frame->hdmap_struct->road_boundary.resize(1);
  auto& polygon = frame->hdmap_struct->road_polygons[0];
  auto& boundary = frame->hdmap_struct->road_boundary[0];
  base::PolygonDType::PointType point;
  point.x = -20.0;
  point.y = -20.0;
  polygon.push_back(point);
  boundary.right_boundary.push_back(point);
  point.x = 20.0;
  point.y = -20.0;
  polygon.push_back(point);
  boundary.right_boundary.push_back(point);
  point.x = 20.0;
  point.y = 20.0;
  polygon.push_back(point);
  boundary.left_boundary.push_back(point);
  point.x = -20.0;
  point.y = 20.0;
  polygon.push_back(point);
  boundary.left_boundary.push_back(point);

  frame->segmented_objects.resize(3);

  frame->segmented_objects[2].reset(new base::Object);
  point.x = -5.0;
  point.y = 2.0;
  frame->segmented_objects[2]->polygon.push_back(point);
  point.x = -2.0;
  point.y = 1.0;
  frame->segmented_objects[2]->polygon.push_back(point);
  point.x = -2.0;
  point.y = -2.0;
  frame->segmented_objects[2]->polygon.push_back(point);
  point.x = -5.0;
  point.y = -2.0;
  frame->segmented_objects[2]->polygon.push_back(point);
  frame->segmented_objects[2]->id = 2;
  frame->segmented_objects[2]->lidar_supplement.cloud.resize(1);
  frame->segmented_objects[2]->lidar_supplement.num_points_in_roi = 1;

  frame->segmented_objects[1].reset(new base::Object);
  point.x = -5.0;
  point.y = 21.0;
  frame->segmented_objects[1]->polygon.push_back(point);
  point.x = -2.0;
  point.y = 21.0;
  frame->segmented_objects[1]->polygon.push_back(point);
  point.x = -2.0;
  point.y = 19.0;
  frame->segmented_objects[1]->polygon.push_back(point);
  point.x = -5.0;
  point.y = 19.0;
  frame->segmented_objects[1]->polygon.push_back(point);
  frame->segmented_objects[1]->id = 1;
  frame->segmented_objects[1]->lidar_supplement.cloud.resize(2);
  frame->segmented_objects[1]->lidar_supplement.num_points_in_roi = 1;

  frame->segmented_objects[0].reset(new base::Object);
  point.x = -5.0;
  point.y = 23;
  frame->segmented_objects[0]->polygon.push_back(point);
  point.x = -2.0;
  point.y = 23;
  frame->segmented_objects[0]->polygon.push_back(point);
  point.x = -2.0;
  point.y = 21;
  frame->segmented_objects[0]->polygon.push_back(point);
  point.x = -5.0;
  point.y = 21;
  frame->segmented_objects[0]->polygon.push_back(point);
  frame->segmented_objects[0]->id = 0;
  frame->segmented_objects[0]->lidar_supplement.cloud.resize(1);
  frame->segmented_objects[0]->lidar_supplement.num_points_in_roi = 0;
}

void ExtendLidarFrame(LidarFrame* frame) {
  frame->segmented_objects.resize(6);
  frame->segmented_objects[3].reset(new base::Object);
  base::PolygonDType::PointType point;
  point.x = 19.9;
  point.y = 19.9;
  frame->segmented_objects[3]->polygon.push_back(point);
  point.x = 19.1;
  point.y = 19.1;
  frame->segmented_objects[3]->polygon.push_back(point);
  point.x = 19.9;
  point.y = 19.1;
  frame->segmented_objects[3]->polygon.push_back(point);
  frame->segmented_objects[3]->id = 3;
  frame->segmented_objects[3]->lidar_supplement.cloud.resize(1);
  frame->segmented_objects[3]->lidar_supplement.num_points_in_roi = 1;
  frame->segmented_objects[3]->confidence = 0.11f;
  frame->segmented_objects[3]->lidar_supplement.is_background = false;

  frame->segmented_objects[4].reset(new base::Object);
  point.x = 19.9;
  point.y = 19.9;
  frame->segmented_objects[4]->polygon.push_back(point);
  point.x = 19.1;
  point.y = 19.1;
  frame->segmented_objects[4]->polygon.push_back(point);
  point.x = 19.9;
  point.y = 19.1;
  frame->segmented_objects[4]->polygon.push_back(point);
  frame->segmented_objects[4]->id = 4;
  frame->segmented_objects[4]->lidar_supplement.cloud.resize(1);
  frame->segmented_objects[4]->lidar_supplement.num_points_in_roi = 1;
  frame->segmented_objects[4]->confidence = 0.12f;
  frame->segmented_objects[4]->lidar_supplement.is_background = false;

  frame->segmented_objects[5].reset(new base::Object);
  point.x = -19.9;
  point.y = -19.9;
  frame->segmented_objects[5]->polygon.push_back(point);
  point.x = -19.1;
  point.y = -19.1;
  frame->segmented_objects[5]->polygon.push_back(point);
  point.x = -18.9;
  point.y = -18.9;
  frame->segmented_objects[5]->polygon.push_back(point);
  frame->segmented_objects[5]->id = 5;
  frame->segmented_objects[5]->lidar_supplement.cloud.resize(1);
  frame->segmented_objects[5]->lidar_supplement.num_points_in_roi = 1;
  frame->segmented_objects[5]->confidence = 0.11f;
  frame->segmented_objects[5]->lidar_supplement.is_background = false;
}

TEST_F(ROIBoundaryFilterTest, roi_boundary_filter_test) {
  EXPECT_TRUE(filter_.Init());
  LidarFrame frame;
  MockLidarFrame(&frame);

  ObjectFilterOptions filter_option;

  LidarFrame empty_frame;
  EXPECT_FALSE(filter_.Filter(filter_option, nullptr));
  EXPECT_TRUE(filter_.Filter(filter_option, &empty_frame));

  EXPECT_EQ(frame.segmented_objects.size(), 3);
  for (auto& obj : frame.segmented_objects) {
    obj->lidar_supplement.is_background = true;
  }
  EXPECT_TRUE(filter_.Filter(filter_option, &frame));
  EXPECT_EQ(frame.segmented_objects.size(), 3);
  for (auto& obj : frame.segmented_objects) {
    obj->lidar_supplement.is_background = false;
    obj->confidence = 1.0;
  }

  filter_.distance_to_boundary_threshold_ = 2.0;
  EXPECT_TRUE(filter_.Filter(filter_option, &frame));
  EXPECT_EQ(frame.segmented_objects.size(), 3);
  filter_.distance_to_boundary_threshold_ = 0.6;
  EXPECT_TRUE(filter_.Filter(filter_option, &frame));
  EXPECT_EQ(frame.segmented_objects.size(), 2);

  frame.hdmap_struct->road_polygons.clear();
  frame.hdmap_struct->junction_polygons.clear();
  EXPECT_TRUE(filter_.Filter(filter_option, &frame));

  MockLidarFrame(&frame);
  EXPECT_EQ(frame.segmented_objects.size(), 3);
  for (auto& obj : frame.segmented_objects) {
    obj->lidar_supplement.is_background = false;
  }
  frame.segmented_objects[0]->confidence = 0.8f;
  frame.segmented_objects[1]->confidence = 0.4f;
  filter_.distance_to_boundary_threshold_ = -1.f;
  EXPECT_TRUE(filter_.Filter(filter_option, &frame));
  EXPECT_EQ(frame.segmented_objects.size(), 2);

  frame.segmented_objects[0]->confidence = 0.4f;
  EXPECT_TRUE(filter_.Filter(filter_option, &frame));
  EXPECT_EQ(frame.segmented_objects.size(), 1);

  MockLidarFrame(&frame);
  ExtendLidarFrame(&frame);
  EXPECT_EQ(frame.segmented_objects.size(), 6);
  frame.segmented_objects[0]->lidar_supplement.is_background = false;
  frame.segmented_objects[1]->lidar_supplement.is_background = false;
  frame.segmented_objects[2]->lidar_supplement.is_background = false;
  frame.segmented_objects[0]->confidence = 0.8f;
  frame.segmented_objects[1]->confidence = 0.4f;
  filter_.distance_to_boundary_threshold_ = -1.f;
  EXPECT_TRUE(filter_.Filter(filter_option, &frame));
  EXPECT_EQ(frame.segmented_objects.size(), 4);
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
