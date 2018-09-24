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
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/lidar/common/lidar_log.h"
#include "modules/perception/lidar/lib/interface/base_bipartite_graph_matcher.h"
#include "modules/perception/lidar/lib/object_builder/object_builder.h"
#include "modules/perception/lidar/lib/tracker/common/track_data.h"
#include "modules/perception/lidar/lib/tracker/common/track_pool_types.h"
#include "modules/perception/lidar/lib/tracker/common/tracked_object.h"
#include "modules/perception/lidar/lib/tracker/hm_tracker/object_track_matcher.h"
#include "modules/perception/lidar/lib/tracker/hm_tracker/track_object_distance.h"

namespace apollo {
namespace perception {
namespace lib {
DECLARE_string(work_root);
}

namespace lidar {

class ObjectTrackMatcherTest : public testing::Test {
 protected:
  typedef std::pair<size_t, size_t> TrackObjectPair;
  void SetUp() {
    char* cybertron_path = "CYBERTRON_PATH=";
    putenv(cybertron_path);
    char* module_path = "MODULE_PATH=";
    putenv(module_path);
    lib::FLAGS_work_root = "./lidar_test_data/lib/tracker/hm_tracker";
    object_builder_ = new ObjectBuilder();
    config_manager_ = lib::Singleton<lib::ConfigManager>::get_instance();
    CHECK_NOTNULL(config_manager_);
    config_manager_->Reset();
    object_builder_->Init();
  }
  void TearDown() {
    delete object_builder_;
    object_builder_ = nullptr;
  }

 protected:
  lib::ConfigManager* config_manager_ = nullptr;
  ObjectBuilder* object_builder_ = nullptr;
};  // class ObjectBuilderTest

bool ConstructPointCloud(std::vector<base::ObjectPtr>* objects) {
  std::string pcd_data(
      "./lidar_test_data/lib/tracker/hm_tracker/data/objects.pcd");
  std::ifstream cluster_ifs(pcd_data.c_str(), std::ifstream::in);
  std::string point_buf;
  while (cluster_ifs.good()) {
    getline(cluster_ifs, point_buf);
    std::stringstream ss;
    ss << point_buf;
    int point_num = 0;
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
    object->lidar_supplement.is_background = true;
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

TEST_F(ObjectTrackMatcherTest, foreground_matcher) {
  LidarFrame* frame(new LidarFrame);
  CHECK(frame != nullptr);
  std::vector<base::ObjectPtr>& objects = frame->segmented_objects;

  ConstructPointCloud(&objects);
  ObjectBuilderOptions builder_options;
  builder_options.ref_center << 0.0, 0.0, 0.0;
  EXPECT_EQ(6, objects.size());
  EXPECT_TRUE(object_builder_->Build(builder_options, frame));

  // LOG_INFO<< "to string" <<objects[0]->ToString();
  Eigen::Affine3d pose(Eigen::Affine3d::Identity());
  // pose.AffinePart = Eigen::Matrix3d::Identity();
  // pose.LinearPart = Eigen::vector3d::Zero();
  std::vector<TrackedObjectPtr> tracked_objects;
  TrackedObjectPool::Instance().BatchGet(frame->segmented_objects.size(),
                                         &tracked_objects);
  ConstructTrackedObjects(objects, &tracked_objects, pose);
  // LOG_INFO<< "tracked object size " <<tracked_objects.size();
  std::vector<TrackDataPtr> tracks_data;
  std::vector<TrackedObjectPtr> foreground_tracked_objects;
  std::vector<Eigen::VectorXf> foreground_predicts;

  TrackDataPtr track1(new TrackData());
  TrackDataPtr track2(new TrackData());
  track1->PushTrackedObjectToTrack(tracked_objects[0], 0.0);
  track2->PushTrackedObjectToTrack(tracked_objects[1], 0.0);
  tracks_data.push_back(track1);
  tracks_data.push_back(track2);
  foreground_tracked_objects.push_back(tracked_objects[2]);
  foreground_tracked_objects.push_back(tracked_objects[3]);

  Eigen::VectorXf foreground_predict_track1;
  foreground_predict_track1.resize(6);
  foreground_predict_track1(0) = 1.5;
  foreground_predict_track1(1) = 0.5;
  foreground_predict_track1(2) = 0.5;
  foreground_predict_track1(3) = 1;
  foreground_predict_track1(4) = 0;
  foreground_predict_track1(5) = 0;

  Eigen::VectorXf foreground_predict_track2;
  foreground_predict_track2.resize(6);
  foreground_predict_track2(0) = 105.5;
  foreground_predict_track2(1) = 100.5;
  foreground_predict_track2(2) = 0.5;
  foreground_predict_track2(3) = 5;
  foreground_predict_track2(4) = 0;
  foreground_predict_track2(5) = 0;

  foreground_predicts.push_back(foreground_predict_track1);
  foreground_predicts.push_back(foreground_predict_track2);

  ObjectTrackMatcherInitOptions foreground_options;
  foreground_options.is_background = false;
  foreground_options.matcher_name = "MultiHmBipartiteGraphMatcher";
  std::shared_ptr<ObjectTrackMatcher> foreground_matcher;
  foreground_matcher.reset(new ObjectTrackMatcher);
  CHECK_NOTNULL(foreground_matcher.get());
  EXPECT_TRUE(foreground_matcher->Init(foreground_options));

  ObjectTrackMatcherOptions options;
  std::vector<TrackObjectPair> foreground_assignments;
  std::vector<size_t> foreground_unassigned_tracks;
  std::vector<size_t> foreground_unassigned_objects;
  foreground_matcher->Match(options, foreground_tracked_objects, tracks_data,
                            foreground_predicts, 1.0, &foreground_assignments,
                            &foreground_unassigned_tracks,
                            &foreground_unassigned_objects);
  // foreground_assignments are (0, 0), (1, 1)
  EXPECT_EQ(2, foreground_assignments.size());
  EXPECT_EQ(0, foreground_unassigned_tracks.size());
  EXPECT_EQ(0, foreground_unassigned_objects.size());
  EXPECT_EQ(0, foreground_assignments[0].first);
  EXPECT_EQ(0, foreground_assignments[0].second);
  EXPECT_EQ(1, foreground_assignments[1].first);
  EXPECT_EQ(1, foreground_assignments[1].second);
  // if (frame != nullptr) {
  //    delete frame;
  //    frame = nullptr;
  //  }
  delete frame;
}

TEST_F(ObjectTrackMatcherTest, background_matcher) {
  LidarFrame* frame(new LidarFrame);
  CHECK(frame != nullptr);
  std::vector<base::ObjectPtr>& objects = frame->segmented_objects;
  ConstructPointCloud(&objects);

  ObjectBuilderOptions builder_options;
  builder_options.ref_center << 0.0, 0.0, 0.0;
  EXPECT_EQ(6, objects.size());
  EXPECT_TRUE(object_builder_->Build(builder_options, frame));

  Eigen::Affine3d pose = Eigen::Affine3d::Identity();
  std::vector<TrackedObjectPtr> tracked_objects;
  TrackedObjectPool::Instance().BatchGet(frame->segmented_objects.size(),
                                         &tracked_objects);
  ConstructTrackedObjects(objects, &tracked_objects, pose);

  std::vector<TrackDataPtr> tracks_data;
  std::vector<TrackedObjectPtr> background_tracked_objects;
  std::vector<Eigen::VectorXf> background_predicts;
  assert(tracked_objects.size() != 6);
  TrackDataPtr track1(new TrackData());
  TrackDataPtr track2(new TrackData());
  track1->PushTrackedObjectToTrack(tracked_objects[0], 0.0);
  track2->PushTrackedObjectToTrack(tracked_objects[1], 0.0);
  tracks_data.push_back(track1);
  tracks_data.push_back(track2);
  background_tracked_objects.push_back(tracked_objects[4]);
  background_tracked_objects.push_back(tracked_objects[5]);

  Eigen::VectorXf background_predict_track1 = Eigen::VectorXf::Zero(6);
  background_predict_track1.resize(6);
  Eigen::VectorXf background_predict_track2 = Eigen::VectorXf::Zero(6);
  background_predict_track2.resize(6);
  background_predicts.push_back(background_predict_track1);
  background_predicts.push_back(background_predict_track2);

  ObjectTrackMatcherInitOptions background_options;
  background_options.is_background = true;
  background_options.matcher_name = "GnnBipartiteGraphMatcher";

  std::shared_ptr<ObjectTrackMatcher> background_matcher;
  background_matcher.reset(new ObjectTrackMatcher);

  EXPECT_TRUE(background_matcher->Init(background_options));

  ObjectTrackMatcherOptions options;
  // background assignments are (0, 1), (1, 0)
  std::vector<TrackObjectPair> background_assignments;
  std::vector<size_t> background_unassigned_tracks;
  std::vector<size_t> background_unassigned_objects;
  background_matcher->Match(options, background_tracked_objects, tracks_data,
                            background_predicts, 1.0, &background_assignments,
                            &background_unassigned_tracks,
                            &background_unassigned_objects);

  EXPECT_EQ(2, background_assignments.size());
  EXPECT_EQ(0, background_unassigned_tracks.size());
  EXPECT_EQ(0, background_unassigned_objects.size());
  EXPECT_EQ(0, background_assignments[0].first);
  EXPECT_EQ(1, background_assignments[0].second);
  EXPECT_EQ(1, background_assignments[1].first);
  EXPECT_EQ(0, background_assignments[1].second);
  // if (frame != nullptr) {
  //   delete frame;
  //   frame = nullptr;
  // }
  delete frame;
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
