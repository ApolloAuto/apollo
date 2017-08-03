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

#include <memory>
#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include "modules/common/log.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/lib/pcl_util/pcl_types.h"
#include "modules/perception/obstacle/lidar/segmentation/cnnseg/cnn_segmentation.h"

#define VISUALIZE
#define POINT_CLOUD_FILE "modules/perception/data/cnnseg_test/uscar_12_1470770225_1470770492_1349.pcd"

namespace apollo {
namespace perception {

class CNNSegmentationTest : public testing::Test {
 protected:
  CNNSegmentationTest() {}
  ~CNNSegmentationTest() {}
  void SetUp() {
    cnn_segmentor_.reset(new CNNSegmentation());
  }
  void TearDown() {}

 protected:
  std::shared_ptr<CNNSegmentation> cnn_segmentor_;
};

bool GetPointCloudFromFile(const std::string& pcd_file,
                           pcl_util::PointCloudPtr cloud) {
  pcl::PointCloud<pcl_util::PointXYZIT> ori_cloud;
  if (pcl::io::loadPCDFile(pcd_file, ori_cloud) < 0) {
    AERROR << "Failed to load pcd file: " << pcd_file;
    return false;
  }

  cloud->points.reserve(ori_cloud.points.size());
  for (size_t i = 0; i < ori_cloud.points.size(); ++i) {
    pcl_util::Point point;
    point.x = ori_cloud.points[i].x;
    point.y = ori_cloud.points[i].y;
    point.z = ori_cloud.points[i].z;
    point.intensity = ori_cloud.points[i].intensity;
    if (isnan(ori_cloud.points[i].x)) {
      continue;
    }
    cloud->push_back(point);
  }

  return true;
}

TEST_F(CNNSegmentationTest, test_cnnseg_det) {
  FLAGS_work_root = "modules/perception";
  FLAGS_config_manager_path = "./conf/config_manager.config";

  // generate input point cloud data
  std::string in_pcd_file(POINT_CLOUD_FILE);
  pcl_util::PointCloudPtr in_pc;
  in_pc.reset(new pcl_util::PointCloud());
  EXPECT_TRUE(GetPointCloudFromFile(in_pcd_file, in_pc));

  pcl_util::PointIndices valid_idx;
  auto &indices = valid_idx.indices;
  indices.resize(in_pc->size());
  std::iota(indices.begin(), indices.end(), 0);

  SegmentationOptions options;
  options.origin_cloud = in_pc;

  std::vector<ObjectPtr> out_objects;

  // testing initialization function
  EXPECT_TRUE(cnn_segmentor_->Init());

  // testing segment function
  EXPECT_TRUE(cnn_segmentor_->Segment(in_pc, valid_idx, options, &out_objects));

  EXPECT_GE(out_objects.size(), 0);
}

}  // namespace perception
}  // namespace apollo
