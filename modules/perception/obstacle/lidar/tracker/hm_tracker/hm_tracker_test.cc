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

#include "modules/perception/obstacle/lidar/tracker/hm_tracker/hm_tracker.h"

#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>

#include "gtest/gtest.h"

#include "modules/common/log.h"
#include "modules/common/util/file.h"
#include "modules/perception/common/pcl_types.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/obstacle/common/pose_util.h"
#include "modules/perception/obstacle/lidar/object_builder/min_box/min_box.h"

namespace apollo {
namespace perception {

class HmObjectTrackerTest : public testing::Test {
 protected:
  HmObjectTrackerTest() {}
  virtual ~HmObjectTrackerTest() {}
  void SetUp() {
    RegisterFactoryHmObjectTracker();
    FLAGS_work_root = "modules/perception";
    FLAGS_config_manager_path = "conf/config_manager.config";
    if (!ConfigManager::instance()->Init()) {
      AERROR << "failed to Init ConfigManager";
      return;
    }
    hm_tracker_ = new HmObjectTracker();
    object_builder_ = new MinBoxObjectBuilder();
    object_builder_->Init();
    object_builder_options_.ref_center =
        Eigen::Vector3d(0, 0, -1.7);  // velodyne height
    tracker_options_.velodyne_trans.reset(new Eigen::Matrix4d);
  }
  void TearDown() {
    delete hm_tracker_;
    hm_tracker_ = nullptr;
    delete object_builder_;
    object_builder_ = nullptr;
  }

 protected:
  HmObjectTracker* hm_tracker_ = nullptr;
  MinBoxObjectBuilder* object_builder_ = nullptr;
  ObjectBuilderOptions object_builder_options_;
  TrackerOptions tracker_options_;
};

bool ConstructObjects(const std::string& filename,
                      std::vector<std::shared_ptr<Object>>* objects) {
  std::ifstream ifs(filename);
  if (!ifs.is_open()) {
    AERROR << "failed to open file" << filename;
    return false;
  }
  std::string type;
  int no_point = 0;
  float tmp = 0;
  while (ifs >> type) {
    ifs >> tmp >> tmp >> tmp >> no_point;
    std::shared_ptr<Object> obj(new Object());
    obj->cloud->resize(no_point);
    for (int j = 0; j < no_point; ++j) {
      ifs >> obj->cloud->points[j].x >> obj->cloud->points[j].y >>
          obj->cloud->points[j].z >> obj->cloud->points[j].intensity;
    }
    (*objects).push_back(obj);
  }
  return true;
}

TEST_F(HmObjectTrackerTest, Track) {
  // test initialization of hm tracker
  EXPECT_TRUE(hm_tracker_->Init());
  // test tracking via hm tracker
  std::string data_path = "modules/perception/data/hm_tracker_test/";
  std::vector<std::string> seg_filenames;
  common::util::GetFileNamesInFolderById(data_path, ".seg", &seg_filenames);
  std::vector<std::string> pose_filenames;
  common::util::GetFileNamesInFolderById(data_path, ".pose", &pose_filenames);
  int frame_id = -1;
  double time_stamp = 0.0;
  EXPECT_GT(seg_filenames.size(), 0);
  EXPECT_EQ(seg_filenames.size(), pose_filenames.size());
  Eigen::Vector3d global_offset(0, 0, 0);
  for (size_t i = 0; i < seg_filenames.size(); ++i) {
    // read pose
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    if (!ReadPoseFile(data_path + pose_filenames[i], &pose, &frame_id,
                      &time_stamp)) {
      AERROR << "failed to read pose";
      return;
    }
    if (i == 0) {
      global_offset(0) = pose(0, 3);
      global_offset(1) = pose(1, 3);
      global_offset(2) = pose(2, 3);
    }
    pose(0, 3) -= global_offset(0);
    pose(1, 3) -= global_offset(1);
    pose(2, 3) -= global_offset(2);
    // read segments
    std::vector<std::shared_ptr<Object>> objects;
    if (!ConstructObjects(data_path + seg_filenames[i], &objects)) {
      AERROR << "failed to read segments";
      return;
    }
    // build objects
    object_builder_->Build(object_builder_options_, &objects);
    // test tracking
    *(tracker_options_.velodyne_trans) = pose;
    std::vector<std::shared_ptr<Object>> result_objects;
    // assert tracking successfully
    EXPECT_TRUE(hm_tracker_->Track(objects, time_stamp, tracker_options_,
                                   &result_objects));
    // assert reports completely
    EXPECT_TRUE(result_objects.size() >= objects.size());
    std::map<int, int> id_pool;
    for (size_t j = 0; j < result_objects.size(); ++j) {
      int track_id = result_objects[j]->track_id;
      // assert no duplicated track id in the same frame
      EXPECT_TRUE(id_pool.find(track_id) == id_pool.end());
      id_pool[track_id] = 1;
    }
  }
}

}  // namespace perception
}  // namespace apollo
