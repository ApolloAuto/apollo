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

#include "modules/perception/pointcloud_ground_detection/ground_detector/ground_service_detector/ground_service_detector.h"

#include <fstream>
#include "gtest/gtest.h"

#include "modules/perception/common/algorithm/io/io_util.h"
#include "modules/perception/common/perception_gflags.h"

namespace apollo {
namespace perception {
namespace lidar {

class LidarLibGroundServiceDetectorTest : public testing::Test {
 protected:
  void SetUp() {
    char cyber_path[] = "CYBER_PATH=";
    putenv(cyber_path);
    char module_path[] = "MODULE_PATH=";
    putenv(module_path);
  }

  void TearDown() {}
};

void LoadPlanes(std::string path, GroundNode* node_ptr) {
  int index = 0;
  std::ifstream file(path.c_str(), std::ifstream::in);
  while (file.good()) {
    std::string buf;
    getline(file, buf);
    if (buf.empty()) {
      continue;
    }
    std::stringstream ss;
    ss << buf;
    ss >> index;
    GroundNode* node = node_ptr + index;
    ss >> node->params(0) >> node->params(1) >> node->params(2) >>
        node->params(3) >> node->confidence;
  }
}

void LoadPoints(const std::string path, std::vector<std::vector<double>>* pts,
                std::vector<float>* height_gts) {
  std::vector<double> pt;
  std::ifstream file(path.c_str(), std::ifstream::in);
  double temp;
  float height;
  while (file.good()) {
    pt.clear();
    std::string buf;
    getline(file, buf);
    if (buf.empty()) {
      continue;
    }
    std::stringstream ss;
    ss << buf;
    for (int i = 0; i < 3; ++i) {
      ss >> temp;
      pt.push_back(temp);
    }
    ss >> height;
    height_gts->push_back(height);
    pts->push_back(pt);
  }
}

TEST_F(LidarLibGroundServiceDetectorTest,
       lidar_lib_scene_manager_ground_service_test) {
  GroundServiceDetector ground_service_detector;
  EXPECT_EQ(ground_service_detector.Name(), "GroundServiceDetector");
  EXPECT_FALSE(ground_service_detector.Init(GroundDetectorInitOptions()));

  EXPECT_TRUE(SceneManager::Instance().Init());
  EXPECT_TRUE(ground_service_detector.Init(GroundDetectorInitOptions()));
  EXPECT_FALSE(
      ground_service_detector.Detect(GroundDetectorOptions(), nullptr));
  LidarFrame frame;
  EXPECT_FALSE(ground_service_detector.Detect(GroundDetectorOptions(), &frame));
  frame.world_cloud = base::PointDCloudPool::Instance().Get();
  frame.cloud = base::PointFCloudPool::Instance().Get();
  EXPECT_FALSE(ground_service_detector.Detect(GroundDetectorOptions(), &frame));

  GroundServiceContent ground_service_content;
  auto ground_service = SceneManager::Instance().Service("GroundService");
  ACHECK(ground_service);
  ground_service->GetServiceContentCopy(&ground_service_content);
  ground_service->UpdateServiceContent(ground_service_content);

  GroundServicePtr ground_service_cast =
      std::dynamic_pointer_cast<GroundService>(ground_service);

  // test query
  std::vector<std::vector<double>> world_pts;
  std::vector<float> height_gts;
  float out_gt = 0.f;
  float out = 0.f;
  GroundNode* node_ptr =
      ground_service_cast->GetGroundServiceContent()->grid_.DataPtr();
  ground_service_cast->GetGroundServiceContent()->grid_center_
      << 461957.33791688998,
      4404672.5859791003, 19.143968966679999;
  LoadPlanes(
      "/apollo/modules/perception/testdata/lidar/lib/ground_detector/"
      "ground_service_detector/data/resources/planes.txt",
      node_ptr);
  LoadPoints(
      "/apollo/modules/perception/testdata/lidar/lib/ground_detector/"
      "ground_service_detector/data/resources/points.txt",
      &world_pts, &height_gts);

  for (size_t i = 0; i < world_pts.size(); ++i) {
    base::PointD world_pt;
    base::PointF local_pt;
    world_pt.x = world_pts[i][0];
    world_pt.y = world_pts[i][1];
    world_pt.z = world_pts[i][2];
    local_pt.x = static_cast<float>(world_pts[i][0]);
    local_pt.y = static_cast<float>(world_pts[i][1]);
    local_pt.z = static_cast<float>(world_pts[i][2]);
    frame.world_cloud->push_back(world_pt);
    frame.cloud->push_back(local_pt);
  }
  EXPECT_TRUE(ground_service_detector.Detect(GroundDetectorOptions(), &frame));
  for (size_t i = 0; i < frame.world_cloud->size(); ++i) {
    out_gt = height_gts[i];
    out = frame.cloud->points_height(i);
    EXPECT_NEAR(out_gt, out, 1e-6);
  }
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
