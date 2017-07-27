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
#include <gtest/gtest.h>
#include <vector>
#include "modules/common/log.h"
#include "modules/perception/common/perception_gflags.h"

#define private public
#include "modules/perception/obstacle/onboard/lidar_process.h"

namespace apollo {
namespace perception {

using std::vector;

class LidarProcessTest : public testing::Test {
 protected:
  LidarProcessTest() {}
  virtual ~LidarProcessTest() {}
  virtual void SetUp() {}

 private:
  LidarProcess lidar_process_;
};

TEST_F(LidarProcessTest, test_InitFrameDependence) {
  FLAGS_work_root = "modules/perception/testdata";
  FLAGS_enable_hdmap_input = false;
  EXPECT_FALSE(lidar_process_.InitFrameDependence());
  FLAGS_config_manager_path = "./config_manager_test/config_manager.config";
  EXPECT_TRUE(lidar_process_.InitFrameDependence());
}

TEST_F(LidarProcessTest, test_InitAlgorithmPlugin) {
  FLAGS_onboard_roi_filter = "not_exit_algo";
  FLAGS_onboard_segmentor = "not_exit_algo";
  FLAGS_onboard_object_builder = "not_exit_algo";
  FLAGS_onboard_tracker = "not_exit_algo";
  EXPECT_FALSE(lidar_process_.InitAlgorithmPlugin());
  FLAGS_onboard_roi_filter = "DummyROIFilter";
  EXPECT_FALSE(lidar_process_.InitAlgorithmPlugin());

  FLAGS_onboard_segmentor = "DummySegmentation";
  EXPECT_FALSE(lidar_process_.InitAlgorithmPlugin());

  FLAGS_onboard_object_builder = "DummyObjectBuilder";
  EXPECT_FALSE(lidar_process_.InitAlgorithmPlugin());

  FLAGS_onboard_tracker = "DummyTracker";
  EXPECT_TRUE(lidar_process_.InitAlgorithmPlugin());
}

TEST_F(LidarProcessTest, test_Init) {
  lidar_process_.inited_ = true;
  EXPECT_TRUE(lidar_process_.Init());

  lidar_process_.inited_ = false;
  EXPECT_TRUE(lidar_process_.Init());
  EXPECT_TRUE(lidar_process_.inited_);
}

TEST_F(LidarProcessTest, test_GeneratePbMsg) {
  double timestamp = 1234.567;
  lidar_process_.timestamp_ = timestamp;
  vector<ObjectPtr> objs;
  ObjectPtr obj1 = std::make_shared<Object>();
  obj1->type = VEHICLE;
  objs.push_back(obj1);
  ObjectPtr obj2 = std::make_shared<Object>();
  obj2->type = PEDESTRIAN;
  objs.push_back(obj2);
  lidar_process_.objects_ = objs;

  PerceptionObstacles obstacles;
  EXPECT_TRUE(lidar_process_.GeneratePbMsg(&obstacles));
  EXPECT_EQ(obstacles.perception_obstacle_size(), 2);
  EXPECT_EQ(obstacles.perception_obstacle(0).type(),
            PerceptionObstacle::VEHICLE);
  EXPECT_EQ(obstacles.perception_obstacle(1).type(),
            PerceptionObstacle::PEDESTRIAN);
}

}  // namespace perception
}  // namespace apollo
