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

#include "modules/perception/obstacle/onboard/lidar_process.h"

#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "pcl/io/pcd_io.h"

#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/configs/config_gflags.h"
#include "modules/common/log.h"
#include "modules/perception/common/pcl_types.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/obstacle/lidar/dummy/dummy_algorithms.h"

namespace apollo {
namespace perception {

using Eigen::Matrix4d;
using pcl_util::Point;
using pcl_util::PointCloud;
using pcl_util::PointCloudPtr;
using std::vector;

class LidarProcessTest : public testing::Test {
 protected:
  LidarProcessTest() {
    common::adapter::AdapterManagerConfig config;
    config.set_is_ros(false);
    {
      auto *sub_config = config.add_config();
      sub_config->set_mode(common::adapter::AdapterConfig::PUBLISH_ONLY);
      sub_config->set_type(
          common::adapter::AdapterConfig::PERCEPTION_OBSTACLES);
    }

    common::adapter::AdapterManager::Init(config);
  }
  virtual ~LidarProcessTest() {}

  LidarProcess lidar_process_;
};

TEST_F(LidarProcessTest, test_Init) {
  lidar_process_.inited_ = true;
  EXPECT_TRUE(lidar_process_.Init());
  lidar_process_.inited_ = false;

  FLAGS_work_root = "modules/perception/data";
  FLAGS_enable_hdmap_input = false;
  EXPECT_FALSE(lidar_process_.InitFrameDependence());
  EXPECT_FALSE(lidar_process_.Init());
  FLAGS_config_manager_path = "./config_manager_test/config_manager.config";
  FLAGS_enable_hdmap_input = false;
  EXPECT_TRUE(lidar_process_.InitFrameDependence());

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
  EXPECT_FALSE(lidar_process_.Init());

  FLAGS_onboard_tracker = "DummyTracker";
  EXPECT_TRUE(lidar_process_.InitAlgorithmPlugin());

  EXPECT_TRUE(lidar_process_.Init());
  EXPECT_TRUE(lidar_process_.inited_);
}

TEST_F(LidarProcessTest, test_Process) {
  std::string pcd_file =
      "modules/perception/data/hm_tracker_test/"
      "QN68P2_12_1476265365_1476265665_2.pcd";
  PointCloudPtr point_cloud(new PointCloud);
  pcl::PointCloud<pcl_util::PointXYZIT>::Ptr org_cloud(
      new pcl::PointCloud<pcl_util::PointXYZIT>);
  pcl::io::loadPCDFile(pcd_file, *org_cloud);
  point_cloud->points.reserve(org_cloud->points.size());
  for (size_t i = 0; i < org_cloud->points.size(); ++i) {
    pcl_util::Point pt;
    pt.x = org_cloud->points[i].x;
    pt.y = org_cloud->points[i].y;
    pt.z = org_cloud->points[i].z;
    pt.intensity = org_cloud->points[i].intensity;
    if (std::isnan(org_cloud->points[i].x)) continue;
    point_cloud->push_back(pt);
  }
  std::shared_ptr<Matrix4d> velodyne_trans = std::make_shared<Matrix4d>();
  (*velodyne_trans) << 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1;
  lidar_process_.hdmap_input_ = HDMapInput::instance();
  EXPECT_TRUE(lidar_process_.Process(123.0, point_cloud, velodyne_trans));
}

TEST_F(LidarProcessTest, test_GeneratePbMsg) {
  double timestamp = 1234.567;
  lidar_process_.timestamp_ = timestamp;
  vector<std::shared_ptr<Object>> objs;
  std::shared_ptr<Object> obj1 = std::make_shared<Object>();
  obj1->type = ObjectType::VEHICLE;
  objs.push_back(obj1);
  std::shared_ptr<Object> obj2 = std::make_shared<Object>();
  obj2->type = ObjectType::PEDESTRIAN;
  objs.push_back(obj2);
  lidar_process_.objects_ = objs;

  PerceptionObstacles obstacles;
  lidar_process_.GeneratePbMsg(&obstacles);
  EXPECT_EQ(obstacles.perception_obstacle_size(), 2);
  EXPECT_EQ(obstacles.perception_obstacle(0).type(),
            PerceptionObstacle::VEHICLE);
  EXPECT_EQ(obstacles.perception_obstacle(1).type(),
            PerceptionObstacle::PEDESTRIAN);
}

}  // namespace perception
}  // namespace apollo
