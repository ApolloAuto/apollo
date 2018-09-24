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
#include <gtest/gtest.h>
#include <fstream>
#include <vector>
#include "modules/perception/base/object_pool_types.h"
#include "modules/perception/base/object_types.h"
#include "modules/perception/common/io/io_util.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/lib/io/file_util.h"
#include "modules/perception/lidar/common/lidar_log.h"
#include "modules/perception/lidar/lib/interface/base_bipartite_graph_matcher.h"
#include "modules/perception/lidar/lib/object_builder/object_builder.h"
#include "modules/perception/lidar/lib/tracker/common/track_pool_types.h"
#include "modules/perception/lidar/lib/tracker/hm_tracker/hm_multi_target_tracker.h"

namespace apollo {
namespace perception {
namespace lib {
DECLARE_string(work_root);
}

namespace lidar {
class HmMultiTargetTrackerTest : public testing::Test {
 protected:
  typedef std::pair<size_t, size_t> TrackObjectPair;
  void SetUp() {
    char* cybertron_path = "CYBERTRON_PATH=";
    putenv(cybertron_path);
    char* module_path = "MODULE_PATH=";
    putenv(module_path);
    lib::FLAGS_work_root = "./lidar_test_data/lib/tracker/hm_tracker/";
    object_builder_ = new ObjectBuilder();
    hm_tracker_ = new HmMultiTargetTracker();

    config_manager_ = lib::Singleton<lib::ConfigManager>::get_instance();
    CHECK_NOTNULL(config_manager_);
    config_manager_->Reset();
    object_builder_->Init();
  }

  void TearDown() {
    delete object_builder_;
    object_builder_ = nullptr;

    delete hm_tracker_;
    hm_tracker_ = nullptr;
  }

 protected:
  lib::ConfigManager* config_manager_ = nullptr;
  ObjectBuilder* object_builder_ = nullptr;
  HmMultiTargetTracker* hm_tracker_ = nullptr;
};  // class ObjectBuilderTest

bool ConstructObjects(const std::string& pcd_data,
                      std::vector<base::ObjectPtr>* objects) {
  std::ifstream cluster_ifs(pcd_data.c_str(), std::ifstream::in);
  std::string point_buf;
  while (cluster_ifs.good()) {
    getline(cluster_ifs, point_buf);
    std::stringstream ss;
    ss << point_buf;
    int point_num = 0;
    std::string type;
    int tmp;
    ss >> type;
    ss >> tmp;
    ss >> point_num;
    if (point_num <= 0) {
      continue;
    }
    uint64_t intensity;
    base::ObjectPtr object(new base::Object);
    for (int i = 0; i < point_num; ++i) {
      base::PointF p;
      ss >> p.x >> p.y >> p.z >> intensity;
      p.intensity = static_cast<uint8_t>(intensity);
      object->lidar_supplement.cloud.push_back(p);
    }
    object->lidar_supplement.is_background = false;
    objects->push_back(object);
  }
  return true;
}

void ConstructTrackedObjects(const std::vector<base::ObjectPtr>& objects,
                             std::vector<TrackedObjectPtr>* tracked_objects,
                             const Eigen::Affine3d& pose) {
  // Construct tracked objects via necessary transformation & feature computing
  int num_objects = objects.size();
  CHECK(objects.size() == tracked_objects->size());
  // LOG_INFO<< "test1" <<objects.size();
  for (size_t i = 0; i < num_objects; ++i) {
    ((*tracked_objects)[i])->AttachObject(objects[i], pose);
    // compute foreground objects' shape feature
    (*tracked_objects)[i]->histogram_bin_size = 10;
    (*tracked_objects)[i]->ComputeShapeFeatures();
  }
}

TEST_F(HmMultiTargetTrackerTest,
       test_track_with_kalman_filter_with_adaptive_and_boostup) {
  // test Init()
  MultiTargetTrackerInitOptions hm_tracker_init_options;
  EXPECT_TRUE(hm_tracker_->Init(hm_tracker_init_options));
  // test Name()
  EXPECT_STREQ("HmMultiTargetTracker", hm_tracker_->Name().c_str());
  // test Track()
  std::string data_path =
      "./lidar_test_data/lib/tracker/hm_tracker/data/files/";
  std::vector<std::string> pcd_filenames;
  lib::FileUtil::GetFileList(data_path, ".pcd", &pcd_filenames);
  std::vector<std::string> pose_filenames;
  lib::FileUtil::GetFileList(data_path, ".pose", &pose_filenames);
  int frame_id = -1;
  double time_stamp = 0.0;
  assert(pcd_filenames.size() > 0);
  assert(pcd_filenames.size() = pose_filenames.size());
  for (size_t i = 0; i < pcd_filenames.size(); ++i) {
    // LOG_INFO<< "file id " <<i;
    LidarFrame* frame(new LidarFrame);
    // read pose
    Eigen::Affine3d pose = Eigen::Affine3d::Identity();
    CHECK(
        common::ReadPoseFile(pose_filenames[i], &pose, &frame_id, &time_stamp));
    // read segments
    CHECK(frame != nullptr);
    std::vector<base::ObjectPtr>& objects = frame->segmented_objects;
    CHECK(ConstructObjects(pcd_filenames[i], &objects));
    // build objects
    ObjectBuilderOptions object_builder_options;
    // velodyne height
    object_builder_options.ref_center = Eigen::Vector3d(0, 0, -1.7);
    object_builder_->Build(object_builder_options, frame);
    // test tracking
    frame->lidar2world_pose = pose;
    frame->timestamp = time_stamp;
    // assert tracking succesfully
    MultiTargetTrackerOptions hm_tracker_options;
    EXPECT_TRUE(hm_tracker_->Track(hm_tracker_options, frame));
    std::vector<base::ObjectPtr>& result_objects = frame->tracked_objects;
    // assert tracking completly
    // LOG_INFO<< "input and result object size " <<
    //    result_objects.size() << " " <<objects.size();
    EXPECT_TRUE(result_objects.size() >= objects.size());
    // assert tracking without duplication
    std::map<size_t, int> track_id_pool;
    for (size_t j = 0; j < result_objects.size(); ++j) {
      int track_id = result_objects[j]->track_id;
      // LOG_INFO<< "track id " << track_id;
      EXPECT_TRUE(track_id_pool.find(track_id) == track_id_pool.end());
      track_id_pool[track_id] = 1;
    }
    // if (frame != nullptr) {
    //     delete frame;
    //     frame = nullptr;
    // }
    delete frame;
  }
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
