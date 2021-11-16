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

#include "modules/perception/lidar/lib/ground_detector/spatio_temporal_ground_detector/spatio_temporal_ground_detector.h"

#include "gflags/gflags.h"
#include "gtest/gtest.h"

#include "pcl/io/pcd_io.h"

namespace apollo {
namespace perception {
namespace lib {

DECLARE_string(work_root);
DECLARE_string(config_manager_path);

}  // namespace lib

namespace lidar {

bool LoadPCDFile(const std::string& file_path, base::PointFCloudPtr cloud_out) {
  int ret = 0;
  pcl::PointCloud<pcl::PointXYZI> org_cloud;
  if ((ret = pcl::io::loadPCDFile(file_path, org_cloud)) < 0) {
    AERROR << "Failed to load pcd file: " << file_path << " " << ret;
    return false;
  }

  cloud_out->resize(org_cloud.size());
  int pid = 0;
  for (size_t i = 0; i < org_cloud.size(); ++i) {
    if (std::isnan(org_cloud.at(i).x) || std::isnan(org_cloud.at(i).y) ||
        std::isnan(org_cloud.at(i).z)) {
      continue;
    }
    base::PointF& pt = cloud_out->at(pid++);
    pt.x = org_cloud.at(i).x;
    pt.y = org_cloud.at(i).y;
    pt.z = org_cloud.at(i).z;
    pt.intensity = org_cloud.at(i).intensity;
  }
  cloud_out->resize(pid);

  return true;
}

TEST(SpatioTemporalGroundDetectorTest, test_spatio_temporal_ground_detector) {
  // char cyber_path[100] = "CYBER_PATH=";
  // putenv(cyber_path);
  // char module_path[100] = "MODULE_PATH=";
  // putenv(module_path);
  // EXPECT_TRUE(SceneManager::Instance().Init());

  // // load pcd data
  // base::PointFCloudPtr pc_ptr;
  // pc_ptr.reset(new base::PointFCloud);
  // std::string filename =
  //     "modules/perception/testdata/lidar/lib/"
  //     "ground_detector/spatio_temporal_ground_detector/"
  //     "pcd_data/QB9178_3_1461752790_1461753090_36701.pcd";
  // bool ret = LoadPCDFile(filename, pc_ptr);
  // ACHECK(ret) << "Failed to load " << filename;

  // // test init
  // std::shared_ptr<SpatioTemporalGroundDetector> detector(
  //     new SpatioTemporalGroundDetector);
  // EXPECT_TRUE(detector->Init());
  // EXPECT_STREQ("SpatioTemporalGroundDetector", detector->Name().c_str());

  // // test input
  // GroundDetectorOptions options;
  // std::shared_ptr<LidarFrame> frame_data = nullptr;
  // EXPECT_FALSE(detector->Detect(options, frame_data.get()));
  // frame_data = std::shared_ptr<LidarFrame>(new LidarFrame);
  // EXPECT_FALSE(detector->Detect(options, frame_data.get()));
  // frame_data->cloud = base::PointFCloudPool::Instance().Get();
  // frame_data->world_cloud = base::PointDCloudPool::Instance().Get();
  // EXPECT_FALSE(detector->Detect(options, frame_data.get()));

  // // test use_roi_ = false
  // frame_data->cloud = pc_ptr;
  // frame_data->lidar2world_pose = Eigen::Affine3d::Identity();
  // base::PointD temp;
  // for (size_t i = 0; i < frame_data->cloud->size(); ++i) {
  //   const auto& pt = frame_data->cloud->at(i);
  //   temp.x = pt.x;
  //   temp.y = pt.y;
  //   temp.z = pt.z;
  //   frame_data->world_cloud->push_back(temp);
  // }
  // EXPECT_TRUE(detector->Detect(options, frame_data.get()));
  // EXPECT_GT(frame_data->non_ground_indices.indices.size(), 0);

  // // test use_roi_ = true && default_point_size_ = 100
  // detector->use_roi_ = true;
  // detector->default_point_size_ = 10;
  // size_t roi_num = static_cast<size_t>(0.75 * pc_ptr->size());
  // std::vector<int> roi_indices(roi_num);
  // std::iota(roi_indices.begin(), roi_indices.end(), 0);
  // frame_data->roi_indices.indices = roi_indices;
  // EXPECT_TRUE(detector->Detect(options, frame_data.get()));
  // EXPECT_EQ(roi_num * 2, detector->default_point_size_);
  // EXPECT_EQ(roi_num * 2, detector->point_indices_temp_.size());
  // EXPECT_EQ(roi_num * 6, detector->data_.size());
  // EXPECT_GT(frame_data->non_ground_indices.indices.size(), 0);

  // detector->use_roi_ = false;
  // EXPECT_TRUE(detector->Detect(options, frame_data.get()));
  // EXPECT_GT(frame_data->non_ground_indices.indices.size(), 0);
  // auto ground_service = SceneManager::Instance().Service("GroundService");
  // GroundServicePtr ground_service_cast =
  //     std::dynamic_pointer_cast<GroundService>(ground_service);

  // Eigen::Vector3d world_point(0.0, 0.0, 0.0);
  // float out_query = 0.0f;
  // float out_detected = 0.0f;
  // for (size_t i = 0; i < 10; ++i) {
  //   const auto& index = frame_data->non_ground_indices.indices[i];
  //   const auto& pt = frame_data->world_cloud->at(index);
  //   out_detected = frame_data->world_cloud->points_height(index);
  //   world_point(0) = pt.x;
  //   world_point(1) = pt.y;
  //   world_point(2) = pt.z;
  //   out_query = ground_service_cast->QueryPointToGroundDistance(world_point);
  //   EXPECT_NEAR(out_query, out_detected, 1e-6);
  // }
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
