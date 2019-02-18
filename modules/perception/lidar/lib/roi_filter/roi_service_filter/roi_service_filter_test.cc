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
#include "modules/perception/lidar/lib/roi_filter/roi_service_filter/roi_service_filter.h"

#include "gtest/gtest.h"

#include "modules/perception/base/hdmap_struct.h"
#include "modules/perception/common/io/io_util.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/lidar/common/pcl_util.h"
#include "modules/perception/lidar/lib/roi_filter/hdmap_roi_filter/hdmap_roi_filter.h"
#include "modules/perception/lidar/lib/scene_manager/scene_manager.h"
#include "modules/perception/map/hdmap/hdmap_input.h"

namespace apollo {
namespace perception {

namespace lib {
DECLARE_string(config_manager_path);
DECLARE_string(work_root);
}  // namespace lib

namespace lidar {

class LidarLibROIServiceFilterTest : public testing::Test {
 protected:
  void SetUp() {
    char cyber_path[100] = "CYBER_PATH=";
    putenv(cyber_path);
    char module_path[100] = "MODULE_PATH=";
    putenv(module_path);
    FLAGS_work_root =
        "/apollo/modules/perception/testdata/"
        "lidar/lib/roi_filter/roi_service_filter";
    FLAGS_config_manager_path = "./conf";
    lib::ConfigManager::Instance()->Reset();

    map::HDMapInput* hdmap_input = map::HDMapInput::Instance();
    hdmap_input->Reset();
  }

  void TearDown() {}
};

void MockData(LidarFrame* frame) {
  std::string pcd =
      "/apollo/modules/perception/testdata/lidar/lib/roi_filter/"
      "roi_service_filter/data/pcd/1532063882.176900.pcd";
  std::string pose =
      "/apollo/modules/perception/testdata/lidar/lib/roi_filter/"
      "roi_service_filter/data/pose/1532063882.176900.pose";

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
  // auto translation = frame->lidar2world_pose.translation();
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

TEST_F(LidarLibROIServiceFilterTest, lidar_lib_roi_service_filter_test) {
  ROIServiceFilter roi_service_filter;
  EXPECT_EQ(roi_service_filter.Name(), "ROIServiceFilter");
  EXPECT_FALSE(roi_service_filter.Init(ROIFilterInitOptions()));

  EXPECT_TRUE(SceneManager::Instance().Init());
  EXPECT_TRUE(roi_service_filter.Init(ROIFilterInitOptions()));
  EXPECT_FALSE(roi_service_filter.Filter(ROIFilterOptions(), nullptr));
  LidarFrame frame;
  EXPECT_FALSE(roi_service_filter.Filter(ROIFilterOptions(), &frame));
  MockData(&frame);
  EXPECT_FALSE(roi_service_filter.Filter(ROIFilterOptions(), &frame));

  HdmapROIFilter roi_filter;
  CHECK(roi_filter.Init(ROIFilterInitOptions()));
  // TODO(All): Add back tests when data is ready.
  /*
  CHECK(roi_filter.Filter(ROIFilterOptions(), &frame));

  base::PointIndices filter_indices = frame.roi_indices;

  EXPECT_TRUE(roi_service_filter.Filter(ROIFilterOptions(), &frame));
  EXPECT_EQ(filter_indices.indices.size(), frame.roi_indices.indices.size());
  for (size_t i = 0; i < filter_indices.indices.size(); ++i) {
    EXPECT_EQ(filter_indices.indices[i], frame.roi_indices.indices[i]);
  }
  */
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
