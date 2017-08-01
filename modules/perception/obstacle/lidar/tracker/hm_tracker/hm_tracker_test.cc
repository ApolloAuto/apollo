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
#include <fstream>
#include <map>
#include "modules/common/log.h"
#include "modules/perception/common/perception_gflags.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/lib/pcl_util/pcl_types.h"
#include "modules/perception/obstacle/common/file_system_util.h"
#include "modules/perception/obstacle/common/pose_util.h"
#include "modules/perception/obstacle/lidar/object_builder/min_box/min_box.h"
#include "modules/perception/obstacle/lidar/tracker/hm_tracker/hm_tracker.h"

namespace apollo {
namespace perception {

class HmObjectTrackerTest : public testing::Test {
 protected:
  HmObjectTrackerTest() : config_manager_(NULL) {}
  virtual ~HmObjectTrackerTest() {}
  void SetUp() {
    RegisterFactoryHmObjectTracker();
    FLAGS_work_root = "modules/perception";
    FLAGS_config_manager_path = "conf/config_manager.config";
    config_manager_ = Singleton<ConfigManager>::Get();
    if (config_manager_ == NULL) {
      AERROR << "failed to get ConfigManager instance.";
      return;
    }
    if (!config_manager_->Init()) {
      AERROR << "failed to init ConfigManager";
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
    hm_tracker_ = NULL;
    delete object_builder_;
    object_builder_ = NULL;
  }

 protected:
  ConfigManager*        config_manager_;
  HmObjectTracker*      hm_tracker_;
  MinBoxObjectBuilder*  object_builder_;
  ObjectBuilderOptions  object_builder_options_;
  TrackerOptions        tracker_options_;
};

bool ConstructObjects(const std::string filename,
  std::vector<ObjectPtr>* objects) {
  std::ifstream ifs(filename);
  if (!ifs.is_open()) {
    AERROR << "failed to open file" << filename;
    return false;
  }
  std::string type;
  int no_point = 0;
  float tmp = 0;
  while (ifs >> type) {
    ifs >> tmp >> tmp >> no_point;
    ObjectPtr obj(new Object());
    obj->cloud->resize(no_point);
    for (int j = 0; j < no_point; ++j) {
      ifs >> obj->cloud->points[j].x
        >> obj->cloud->points[j].y
        >> obj->cloud->points[j].z
        >> obj->cloud->points[j].intensity;
    }
    (*objects).push_back(obj);
  }
  return true;
}

TEST_F(HmObjectTrackerTest, demo_tracking) {
  // test initialization of hm tracker
  EXPECT_TRUE(hm_tracker_->Init());
  // test tracking via hm tracker
  std::string data_path = "data/hm_tracker_test/";
  std::vector<std::string> pcd_filenames;
  get_file_names_in_folder_by_id(data_path, ".pcd", pcd_filenames);
  std::vector<std::string> pose_filenames;
  get_file_names_in_folder_by_id(data_path, ".pose", pose_filenames);
  int frame_id = -1;
  double time_stamp = 0.0;
  for (int i = 0; i < pcd_filenames.size(); ++i) {
    // read pose
    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    if (!read_pose_file(pose_filenames[i], pose, frame_id, time_stamp)) {
      AERROR << "failed to read pose";
      return;
    }
    // read segments
    std::vector<ObjectPtr> objects;
    if (!ConstructObjects(pcd_filenames[i], &objects)) {
      AERROR << "failed to read segments";
      return;
    }
    // build objects
    object_builder_->Build(object_builder_options_, &objects);
    // test tracking
    *(tracker_options_.velodyne_trans) = pose;
    std::vector<ObjectPtr> result_objects;
    // assert tracking succesfully
    EXPECT_TRUE(hm_tracker_->Track(objects, time_stamp, tracker_options_,
      &result_objects));
    // assert reports completly
    EXPECT_TRUE(result_objects.size() >= objects.size());
    std::map<int, int> id_pool;
    for (size_t j = 0; j < result_objects.size(); ++j) {
      int track_id = result_objects[i]->track_id;
      // assert no duplicated track id in the same frame
      EXPECT_TRUE(id_pool.find(track_id) == id_pool.end());
      id_pool[track_id] = 1;
    }
  }
}

}  // namespace perception
}  // namespace apollo
