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

#include "modules/perception/lidar/lib/scene_manager/scene_manager.h"

#include "gtest/gtest.h"
#include "modules/perception/base/hdmap_struct.h"
#include "modules/perception/common/io/io_util.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/lidar/common/lidar_log.h"
#include "modules/perception/lidar/common/pcl_util.h"
#include "modules/perception/lidar/lib/roi_filter/hdmap_roi_filter/hdmap_roi_filter.h"
#include "modules/perception/lidar/lib/scene_manager/ground_service/ground_service.h"
#include "modules/perception/lidar/lib/scene_manager/roi_service/roi_service.h"
#include "modules/perception/map/hdmap/hdmap_input.h"

namespace apollo {
namespace perception {
namespace lidar {

class LidarLibSceneManagerTest : public testing::Test {
 protected:
  void SetUp() {
    char cyber_path[] = "CYBER_PATH=";
    putenv(cyber_path);
    char module_path[] = "MODULE_PATH=";
    putenv(module_path);
    FLAGS_work_root =
        "/apollo/modules/perception/testdata/lidar/lib/scene_manager";
    FLAGS_config_manager_path = "./conf";
    lib::ConfigManager::Instance()->Reset();

    map::HDMapInput* hdmap_input = map::HDMapInput::Instance();
    hdmap_input->Reset();
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

TEST_F(LidarLibSceneManagerTest, lidar_lib_scene_manager_test) {
  EXPECT_TRUE(SceneManager::Instance().Init());
  EXPECT_TRUE(SceneManager::Instance().Reset());
  EXPECT_TRUE(SceneManager::Instance().Init());
  EXPECT_GT(SceneManager::Instance().GetServiceNum(), 0);
}

TEST_F(LidarLibSceneManagerTest, lidar_lib_scene_manager_ground_service_test) {
  EXPECT_TRUE(SceneManager::Instance().Init());
  auto ground_service_err = SceneManager::Instance().Service("GroundService1");
  EXPECT_EQ(ground_service_err.get(), nullptr);
  // auto ground_service = std::dynamic_pointer_cast<GroundServicePtr>(
  //    SceneManager::Instance().Service("GroundService"));
  auto ground_service = SceneManager::Instance().Service("GroundService");
  EXPECT_NE(ground_service.get(), nullptr);

  GroundServiceContent ground_service_content;
  ground_service_content.Init(120.0, 120.0, 16, 16);

  // test get content
  ground_service->GetServiceContentCopy(&ground_service_content);
  EXPECT_EQ(ground_service_content.grid_.size(), 256);
  EXPECT_EQ(ground_service_content.rows_, 16);
  EXPECT_EQ(ground_service_content.cols_, 16);
  EXPECT_EQ(ground_service_content.resolution_x_, 15.0);
  EXPECT_EQ(ground_service_content.resolution_y_, 15.0);

  GroundServicePtr ground_service_cast =
      std::dynamic_pointer_cast<GroundService>(ground_service);

  EXPECT_EQ(ground_service_cast->GetGroundServiceContent()->grid_.size(), 256);
  EXPECT_EQ(ground_service_cast->GetGroundServiceContent()->rows_, 16);
  EXPECT_EQ(ground_service_cast->GetGroundServiceContent()->cols_, 16);
  EXPECT_EQ(ground_service_cast->GetGroundServiceContent()->resolution_x_,
            15.0);
  EXPECT_EQ(ground_service_cast->GetGroundServiceContent()->resolution_y_,
            15.0);

  // test set content
  ground_service->UpdateServiceContent(ground_service_content);

  EXPECT_EQ(ground_service_cast->GetGroundServiceContent()->grid_.size(),
            ground_service_content.grid_.size());
  EXPECT_EQ(ground_service_cast->GetGroundServiceContent()->grid_center_(0),
            ground_service_content.grid_center_(0));
  EXPECT_EQ(ground_service_cast->GetGroundServiceContent()->rows_,
            ground_service_content.rows_);
  EXPECT_EQ(ground_service_cast->GetGroundServiceContent()->cols_,
            ground_service_content.cols_);
  EXPECT_EQ(ground_service_cast->GetGroundServiceContent()->resolution_x_,
            ground_service_content.resolution_x_);
  EXPECT_EQ(ground_service_cast->GetGroundServiceContent()->resolution_y_,
            ground_service_content.resolution_y_);
  EXPECT_EQ(
      ground_service_cast->GetGroundServiceContent()->grid_[1][1].confidence,
      0.f);
  EXPECT_EQ(
      ground_service_cast->GetGroundServiceContent()->grid_.IsInGrid(11, -1),
      false);

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
      "/apollo/modules/perception/testdata/lidar/lib/"
      "scene_manager/resources/planes.txt",
      node_ptr);
  LoadPoints(
      "/apollo/modules/perception/testdata/lidar/lib/"
      "scene_manager/resources/points.txt",
      &world_pts, &height_gts);
  Eigen::Vector3d world_point(0.0, 0.0, 0.0);

  ground_service_content.grid_ =
      ground_service_cast->GetGroundServiceContent()->grid_;
  ground_service_content.grid_center_ =
      ground_service_cast->GetGroundServiceContent()->grid_center_;

  for (size_t p = 0; p < world_pts.size(); ++p) {
    for (int j = 0; j < 3; ++j) {
      world_point(j) = world_pts[p][j];
    }
    out_gt = height_gts[p];
    out = ground_service_cast->QueryPointToGroundDistance(world_point);
    EXPECT_NEAR(out_gt, out, 1e-6);
    out = ground_service_cast->QueryPointToGroundDistance(
        world_point, ground_service_content);
    EXPECT_NEAR(out_gt, out, 1e-6);
  }
}

void MockData(LidarFrame* frame) {
  std::string pcd =
      "/apollo/modules/perception/testdata/lidar/lib/scene_manager/data/"
      "pcd/1532063882.176900.pcd";
  std::string pose =
      "/apollo/modules/perception/testdata/lidar/lib/scene_manager/data/"
      "pose/1532063882.176900.pose";
  // a. load pcd
  frame->cloud = base::PointFCloudPool::Instance().Get();
  EXPECT_TRUE(LoadPCLPCD(pcd, frame->cloud.get()));
  // b. load pose
  int idt = 0;
  double timestamp = 0.0;
  EXPECT_TRUE(
      common::ReadPoseFile(pose, &frame->lidar2world_pose, &idt, &timestamp));

  // c.update hdmap struct;
  base::PointD point;
  point.x = frame->lidar2world_pose.translation()(0);
  point.y = frame->lidar2world_pose.translation()(1);
  point.z = frame->lidar2world_pose.translation()(2);
  frame->hdmap_struct.reset(new base::HdmapStruct);
  CHECK(map::HDMapInput::Instance()->GetRoiHDMapStruct(point, 120.0,
                                                       frame->hdmap_struct));

  // d. trans points
  frame->world_cloud = base::PointDCloudPool::Instance().Get();
  frame->lidar2world_pose.translation();
  for (size_t i = 0; i < frame->cloud->size(); ++i) {
    auto& local_pt = frame->cloud->at(i);
    Eigen::Vector3d trans_pt(local_pt.x, local_pt.y, local_pt.z);
    trans_pt = frame->lidar2world_pose * trans_pt;
    base::PointD world_pt;
    world_pt.x = trans_pt(0);
    world_pt.y = trans_pt(1);
    world_pt.z = trans_pt(2);
    frame->world_cloud->push_back(world_pt);
  }
}

TEST_F(LidarLibSceneManagerTest, lidar_lib_roi_service_test) {
  // FIXME(perception): fix the missing data files.
  return;

  ROIServiceContent content, content1, content2;
  content.GetCopy(nullptr);
  EXPECT_EQ(content.Name(), "ROIServiceContent");

  content1.range_ = 100;
  content1.cell_size_ = 0.1;
  content1.transform_ = Eigen::Vector3d(0, 0, 0);
  EXPECT_FALSE(content.Check(Eigen::Vector3d(0, 0, 0)));
  content.SetContent(content1);
  EXPECT_EQ(content.range_, content1.range_);
  EXPECT_EQ(content.cell_size_, content1.cell_size_);
  EXPECT_FALSE(content.Check(Eigen::Vector3d(1000.0, 0, 0)));
  EXPECT_FALSE(content.Check(Eigen::Vector3d(-1000.0, 0, 0)));
  EXPECT_FALSE(content.Check(Eigen::Vector3d(0, 1000.0, 0)));
  EXPECT_FALSE(content.Check(Eigen::Vector3d(0, -1000.0, 0)));

  content.GetCopy(&content2);
  EXPECT_EQ(content.range_, content2.range_);
  EXPECT_EQ(content.cell_size_, content2.cell_size_);

  EXPECT_TRUE(SceneManager::Instance().Init());

  LidarFrame frame;
  MockData(&frame);

  HdmapROIFilter roi_filter;
  CHECK(roi_filter.Init(ROIFilterInitOptions()));
  CHECK(roi_filter.Filter(ROIFilterOptions(), &frame));

  auto roi_service = std::dynamic_pointer_cast<ROIService>(
      SceneManager::Instance().Service("ROIService"));
  CHECK(roi_service);
  EXPECT_EQ(roi_service->Name(), "ROIService");

  base::PointIndices filter_indices;
  for (size_t i = 0; i < frame.world_cloud->size(); ++i) {
    auto& pt = frame.world_cloud->at(i);
    Eigen::Vector3d world_pt(pt.x, pt.y, pt.z);
    if (roi_service->QueryIsPointInROI(world_pt)) {
      filter_indices.indices.push_back(static_cast<int>(i));
    }
  }

  EXPECT_EQ(filter_indices.indices.size(), frame.roi_indices.indices.size());
  for (size_t i = 0; i < filter_indices.indices.size(); ++i) {
    EXPECT_EQ(filter_indices.indices[i], frame.roi_indices.indices[i]);
  }
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
