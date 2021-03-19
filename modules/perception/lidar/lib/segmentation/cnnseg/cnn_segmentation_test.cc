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
#include "modules/perception/lidar/lib/segmentation/cnnseg/cnn_segmentation.h"

#include <algorithm>
#include <limits>

#include "gtest/gtest.h"
#include "pcl/io/pcd_io.h"

#include "modules/perception/common/io/io_util.h"
#include "modules/perception/common/perception_gflags.h"

namespace {
constexpr float kFloatEpsilon = std::numeric_limits<float>::epsilon();
}  // namespace

namespace apollo {
namespace perception {
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

void PrintObjects(const std::vector<base::ObjectPtr>& objects) {
  AINFO << "Total objects num: " << objects.size();
  int obj_id = 0;
  for (auto object : objects) {
    unsigned cloud_size =
        static_cast<unsigned>(object->lidar_supplement.cloud.size());
    AINFO << "Point num of Segment: " << cloud_size;
    std::cout << "-- Object " << obj_id++ << " : ";
    std::cout << object->ToString() << ", type_probs: " << object->type_probs[0]
              << ", " << object->type_probs[1] << ", " << object->type_probs[2]
              << ", " << object->type_probs[3] << ", " << object->type_probs[4]
              << ", " << object->type_probs[5] << std::endl;
  }
}

TEST(CNNSegmentationTest, cnn_segmentation_sequence_test) {
  unsetenv("CYBER_PATH");
  unsetenv("MODULE_PATH");
  FLAGS_work_root =
      "/apollo/modules/perception/testdata/"
      "lidar/lib/segmentation/cnnseg/";

  auto segmentation = std::shared_ptr<CNNSegmentation>(new CNNSegmentation);
  SegmentationOptions options;
  EXPECT_FALSE(segmentation->Segment(options, nullptr));
  LidarFrame frame_data;
  EXPECT_FALSE(segmentation->Segment(options, &frame_data));
  frame_data.cloud = base::PointFCloudPool::Instance().Get();
  frame_data.world_cloud = base::PointDCloudPool::Instance().Get();
  EXPECT_FALSE(segmentation->Segment(options, &frame_data));

  EXPECT_TRUE(segmentation->Init());
  EXPECT_TRUE(segmentation->InitClusterAndBackgroundSegmentation());

  std::string pcd_path =
      "/apollo/modules/perception/testdata/lidar/app/data/";
  std::vector<std::string> pcd_file_names;
  common::GetFileList(pcd_path, ".pcd", &pcd_file_names);
  std::string file_name;
  std::sort(pcd_file_names.begin(), pcd_file_names.end(),
            [](const std::string& lhs, const std::string& rhs) {
              if (lhs.length() < rhs.length()) {
                return true;
              } else if (lhs.length() == rhs.length()) {
                return lhs <= rhs;
              } else {
                return false;
              }
            });
  for (size_t i = 0; i < pcd_file_names.size(); ++i) {
    std::shared_ptr<LidarFrame> frame(new LidarFrame);
    frame->cloud = base::PointFCloudPool::Instance().Get();
    frame->world_cloud = base::PointDCloudPool::Instance().Get();
    if (!LoadPCDFile(pcd_file_names[i], frame->cloud)) {
      continue;
    }
    frame->world_cloud->resize(frame->cloud->size());
    EXPECT_TRUE(segmentation->Segment(options, frame.get()));
  }
}

TEST(CNNSegmentationTest, cnn_segmentation_test) {
  unsetenv("CYBER_PATH");
  unsetenv("MODULE_PATH");
  FLAGS_work_root =
      "/apollo/modules/perception/testdata/"
      "lidar/lib/segmentation/cnnseg/";

  // load pcd data
  auto pcl_ptr = std::shared_ptr<base::PointFCloud>(new base::PointFCloud);
  std::string filename =
      "/apollo/modules/perception/testdata/lidar/app/data/0002_00.pcd";
  bool ret = LoadPCDFile(filename, pcl_ptr);
  ACHECK(ret) << "Failed to load " << filename;
  // load non ground indices
  base::PointIndices non_ground_indices;
  //  auto& indices = non_ground_indices.indices;
  //  std::ifstream in_file(
  //      "/apollo/modules/perception/testdata/lidar/app/data/perception/lidar/files/0002_00.txt");
  //  ASSERT_TRUE(in_file.good());
  //  std::string line;
  //  while (getline(in_file, line)) {
  //    indices.push_back(std::stoi(line));
  //  }

  // test init
  auto segmentation = std::shared_ptr<CNNSegmentation>(new CNNSegmentation);
  EXPECT_TRUE(segmentation->Init());

  // test segment
  using base::ObjectType;
  SegmentationOptions options;
  LidarFrame frame_data;
  frame_data.cloud = pcl_ptr;
  frame_data.world_cloud = base::PointDCloudPool::Instance().Get();
  frame_data.world_cloud->resize(pcl_ptr->size());
  frame_data.non_ground_indices = non_ground_indices;
  segmentation->Segment(options, &frame_data);
  std::vector<base::ObjectPtr>& objects = frame_data.segmented_objects;
  //  EXPECT_LE(4, objects.size());
  EXPECT_GT(objects[0]->lidar_supplement.cloud.size(), 0);
  EXPECT_GT(fabs(objects[3]->confidence), kFloatEpsilon);
  // test heading
  EXPECT_GT(fabs(objects[3]->theta), kFloatEpsilon);
  // test classification
  EXPECT_EQ(1, objects[1]->lidar_supplement.raw_classification_methods.size());
  EXPECT_EQ(1, objects[1]->lidar_supplement.raw_probs.size());
  EXPECT_EQ(static_cast<int>(ObjectType::MAX_OBJECT_TYPE),
            objects[2]->lidar_supplement.raw_probs[0].size());
  PrintObjects(objects);

  segmentation->cnnseg_param_.set_do_classification(false);
  segmentation->cnnseg_param_.set_do_heading(false);
  segmentation->Segment(options, &frame_data);
  objects = frame_data.segmented_objects;
  //  EXPECT_LE(4, objects.size());
  EXPECT_GT(objects[0]->lidar_supplement.cloud.size(), 0);
  EXPECT_GT(fabs(objects[3]->confidence), kFloatEpsilon);
  // test no heading
  EXPECT_LE(fabs(objects[3]->theta), kFloatEpsilon);
  // test no classification
  EXPECT_EQ(0, objects[1]->lidar_supplement.raw_probs.size());
  EXPECT_EQ(0, objects[1]->lidar_supplement.raw_classification_methods.size());
  PrintObjects(objects);

  segmentation->InitClusterAndBackgroundSegmentation();
  segmentation->Segment(options, &frame_data);
  objects = frame_data.segmented_objects;
  //  EXPECT_EQ(4, objects.size());
  EXPECT_GT(objects[0]->lidar_supplement.cloud.size(), 0);
  EXPECT_GT(fabs(objects[3]->confidence), kFloatEpsilon);
  PrintObjects(objects);

  EXPECT_TRUE(segmentation->InitClusterAndBackgroundSegmentation());
  EXPECT_TRUE(segmentation->Segment(options, &frame_data));
  objects = frame_data.segmented_objects;
  //  EXPECT_LE(4, objects.size());
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
