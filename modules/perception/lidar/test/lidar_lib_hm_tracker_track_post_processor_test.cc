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
#include <random>
#include <vector>
#include "modules/perception/base/object_pool_types.h"
#include "modules/perception/base/object_types.h"
#include "modules/perception/common/io/io_util.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/lib/io/file_util.h"
#include "modules/perception/lidar/common/lidar_log.h"
#include "modules/perception/lidar/lib/common/track_pool_types.h"
#include "modules/perception/lidar/lib/interface/base_bipartite_graph_matcher.h"
#include "modules/perception/lidar/lib/object_builder/object_builder.h"
#include "modules/perception/lidar/lib/tracker/hm_tracker/track_post_processor.h"

namespace apollo {
namespace perception {
namespace lib {
DECLARE_string(work_root);
}

namespace lidar {
class TrackPostProcessorTest : public testing::Test {
 protected:
  typedef std::pair<size_t, size_t> TrackObjectPair;

  void SetUp() {
    char *cybertron_path = "CYBERTRON_PATH=";
    putenv(cybertron_path);
    char *module_path = "MODULE_PATH=";
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
  lib::ConfigManager *config_manager_ = nullptr;
  ObjectBuilder *object_builder_ = nullptr;
};  // class ObjectBuilderTest

void ConstructTrackedObjects(const std::vector<base::ObjectPtr> &objects,
                             std::vector<TrackedObjectPtr> *tracked_objects,
                             const Eigen::Affine3d &pose) {
  // Construct tracked objects via necessary transformation & feature computing
  int num_objects = objects.size();
  CHECK(objects.size() == tracked_objects->size());
  // LOG_INFO<< "test1" <<objects.size();
  for (size_t i = 0; i < num_objects; ++i) {
    ((*tracked_objects)[i])->AttachObject(objects[i], pose);
    // compute foreground objects' shape feature
    (*tracked_objects)[i]->histogram_bin_size = 10;
    (*tracked_objects)[i]->ComputeShapeFeatures();
    (*tracked_objects)[i]->belief_velocity = Eigen::Vector3d(5.0, 0.0, 0.0);
    (*tracked_objects)[i]->output_velocity = Eigen::Vector3d(5.0, 0.0, 0.0);
    (*tracked_objects)[i]->converged = false;
  }
}

void ConstructObjects(const std::vector<Eigen::Vector3d> &centers,
                      const std::vector<Eigen::Vector3d> &sizes,
                      std::vector<base::ObjectPtr> *objects) {
  assert(centers.size() == sizes.size());
  uint8_t intensity = 127;
  for (size_t j = 0; j < centers.size(); ++j) {
    Eigen::Vector3d size = sizes[j];
    Eigen::Vector3d center = centers[j];
    double l = size(0);
    double w = size(1);
    double h = size(2);
    base::ObjectPtr object;
    object.reset(new base::Object);
    Eigen::Matrix<double, 3, 8> corners;
    corners << l / 2, l / 2, -l / 2, -l / 2, l / 2, l / 2, -l / 2, -l / 2,
        w / 2, -w / 2, -w / 2, w / 2, w / 2, -w / 2, -w / 2, w / 2, 0, 0, 0, 0,
        h, h, h, h;
    for (int i = 0; i < 8; ++i) {
      base::PointF p;
      p.x = center(0) + corners(0, i);
      p.y = center(1) + corners(1, i);
      p.z = center(2) + corners(2, i);
      p.intensity = intensity;
      object->lidar_supplement.cloud.push_back(p);
      p.x = center(0) + corners(0, i);
    }
    object->lidar_supplement.is_background = false;
    objects->push_back(object);
  }
}

TEST_F(TrackPostProcessorTest, psot_processor_test) {
  LidarFrame *frame(new LidarFrame);
  CHECK(frame != nullptr);
  // construct track data
  std::vector<Eigen::Vector3d> centers;
  centers.resize(3);
  centers[0] = Eigen::Vector3d(1.0, 0.5, 0.0);
  centers[1] = Eigen::Vector3d(6.0, 0.5, 0.0);
  centers[2] = Eigen::Vector3d(11.0, 0.5, 0.0);
  std::vector<Eigen::Vector3d> sizes(3, Eigen::Vector3d(2, 1, 1));
  double ref_timestamp = 1505524892.1844;

  std::vector<base::ObjectPtr> &objects = frame->segmented_objects;
  ConstructObjects(centers, sizes, &objects);

  ObjectBuilderOptions object_builder_options;
  // velodyne height
  object_builder_options.ref_center = Eigen::Vector3d(0, 0, 0);
  object_builder_->Build(object_builder_options, frame);

  std::vector<TrackedObjectPtr> tracked_objects;
  TrackedObjectPool::Instance().BatchGet(objects.size(), &tracked_objects);
  Eigen::Affine3d pose = Eigen::Affine3d::Identity();
  ConstructTrackedObjects(objects, &tracked_objects, pose);
  TrackDataPtr track_data(new TrackData);
  for (size_t i = 0; i < tracked_objects.size(); ++i) {
    double timestamp = ref_timestamp + 0.1 * i;
    track_data->PushTrackedObjectToTrack(tracked_objects[i], timestamp);
  }

  std::pair<double, TrackedObjectPtr> timestamp_object_pair =
      track_data->GetLatestObject();
  TrackedObjectPtr latest_object = timestamp_object_pair.second;
  ;
  TrackPostProcessorPtr post_processor(new TrackPostProcessor);

  TrackDataPtr track_data2(new TrackData);
  for (size_t i = 0; i < 1; ++i) {
    double timestamp = ref_timestamp + 0.1 * i;
    track_data2->PushTrackedObjectToTrack(tracked_objects[i], timestamp);
  }
  post_processor->PostProcess(track_data2);

  // test keep motion, type is not PEDESTRIAN
  latest_object->belief_velocity_gain(0) = 5;
  latest_object->belief_velocity(0) = 10;
  latest_object->output_velocity(0) = 10;
  latest_object->type = base::ObjectType::VEHICLE;
  post_processor->PostProcess(track_data);
  EXPECT_DOUBLE_EQ(5.0, latest_object->output_velocity(0));
  // test smooth orientation
  EXPECT_DOUBLE_EQ(11.0, latest_object->output_center(0));
  EXPECT_DOUBLE_EQ(0.5, latest_object->output_center(1));
  EXPECT_DOUBLE_EQ(0.0, latest_object->output_center(2));
  EXPECT_DOUBLE_EQ(2.0, latest_object->output_size(0));
  EXPECT_DOUBLE_EQ(1.0, latest_object->output_size(1));
  EXPECT_DOUBLE_EQ(1.0, latest_object->output_size(2));

  // test keep motion, type is PEDESTRIAN
  latest_object->converged = false;
  latest_object->belief_velocity_gain(0) = 0.6;
  latest_object->belief_velocity(0) = 5.6;
  latest_object->output_velocity(0) = 5.6;
  latest_object->type = base::ObjectType::PEDESTRIAN;
  post_processor->PostProcess(track_data);
  EXPECT_DOUBLE_EQ(5.0, latest_object->output_velocity(0));

  // test motion score
  latest_object->belief_velocity_gain(0) = 0.0;
  latest_object->belief_velocity(0) = 2.0;
  latest_object->output_velocity(0) = 2.0;
  track_data->motion_state_ = MotionState::STATIC;
  latest_object->motion_score = Eigen::Vector3d::Zero();
  post_processor->PostProcess(track_data);
  EXPECT_DOUBLE_EQ(2.0, latest_object->output_velocity(0));

  // for push dummy objects
  for (size_t i = 0; i < 5; ++i) {
    double timestamp = ref_timestamp - 0.1 * i;
    track_data->PushTrackedObjectToTrack(tracked_objects[0], timestamp);
  }
  latest_object->belief_velocity_gain(0) = 0.0;
  latest_object->belief_velocity(0) = 2.0;
  latest_object->output_velocity(0) = 2.0;
  track_data->motion_state_ = MotionState::STATIC;
  track_data->should_check_velocity_consistency_ = true;
  latest_object->motion_score = Eigen::Vector3d::Zero();
  post_processor->PostProcess(track_data);
  EXPECT_DOUBLE_EQ(2.0, latest_object->output_velocity(0));

  latest_object->belief_velocity_gain(0) = 0.0;
  latest_object->belief_velocity(0) = 2.0;
  latest_object->output_velocity(0) = 2.0;
  track_data->motion_state_ = MotionState::STATIC;
  track_data->should_check_velocity_consistency_ = true;
  latest_object->motion_score = Eigen::Vector3d::Zero();
  track_data->continuous_motion_frames_ = 4;
  post_processor->PostProcess(track_data);
  EXPECT_DOUBLE_EQ(2.0, latest_object->output_velocity(0));

  latest_object->belief_velocity_gain(0) = 0.0;
  latest_object->belief_velocity(0) = 2.0;
  latest_object->output_velocity(0) = 2.0;
  track_data->motion_state_ = MotionState::STATIC;
  track_data->should_check_velocity_consistency_ = true;
  latest_object->motion_score = Eigen::Vector3d(1, 1, 1);
  track_data->pub_remain_frames_ = 0;
  post_processor->PostProcess(track_data);
  EXPECT_DOUBLE_EQ(2.0, latest_object->output_velocity(0));

  latest_object->belief_velocity_gain(0) = 0.0;
  latest_object->belief_velocity(0) = 2.0;
  latest_object->output_velocity(0) = 2.0;
  track_data->motion_state_ = MotionState::STATIC;
  track_data->should_check_velocity_consistency_ = true;
  latest_object->motion_score = Eigen::Vector3d(1, 1, 1);
  track_data->pub_remain_frames_ = 0;
  post_processor->PostProcess(track_data);
  EXPECT_DOUBLE_EQ(2.0, latest_object->output_velocity(0));

  latest_object->belief_velocity_gain(0) = 0.0;
  latest_object->belief_velocity(0) = 2.0;
  latest_object->output_velocity(0) = 2.0;
  track_data->motion_state_ = MotionState::STATIC;
  track_data->should_check_velocity_consistency_ = false;
  latest_object->motion_score = Eigen::Vector3d(1, 1, 1);
  track_data->pub_remain_frames_ = 0;
  post_processor->PostProcess(track_data);
  EXPECT_DOUBLE_EQ(0.0, latest_object->output_velocity(0));

  latest_object->belief_velocity_gain(0) = 0.0;
  latest_object->belief_velocity(0) = 2.0;
  latest_object->output_velocity(0) = 2.0;
  track_data->motion_state_ = MotionState::STATIC;
  track_data->should_check_velocity_consistency_ = true;
  latest_object->motion_score = Eigen::Vector3d(1, 1, 1);
  track_data->pub_remain_frames_ = 2;
  post_processor->PostProcess(track_data);
  EXPECT_DOUBLE_EQ(2.0, latest_object->output_velocity(0));

  latest_object->belief_velocity_gain(0) = 0.0;
  latest_object->belief_velocity(0) = 2.0;
  latest_object->output_velocity(0) = 2.0;
  track_data->motion_state_ = MotionState::SKEPTICAL_MOVE;
  latest_object->motion_score = Eigen::Vector3d::Zero();
  post_processor->PostProcess(track_data);
  EXPECT_DOUBLE_EQ(2.0, latest_object->output_velocity(0));

  latest_object->belief_velocity_gain(0) = 0.0;
  latest_object->belief_velocity(0) = 2.0;
  latest_object->output_velocity(0) = 2.0;
  track_data->motion_state_ = MotionState::SKEPTICAL_MOVE;
  latest_object->motion_score = Eigen::Vector3d::Zero();
  track_data->continuous_motion_frames_ = 9;
  post_processor->PostProcess(track_data);
  EXPECT_DOUBLE_EQ(2.0, latest_object->output_velocity(0));

  latest_object->belief_velocity_gain(0) = 0.0;
  latest_object->belief_velocity(0) = 2.0;
  latest_object->output_velocity(0) = 2.0;
  track_data->motion_state_ = MotionState::SKEPTICAL_MOVE;
  latest_object->motion_score = Eigen::Vector3d(1, 1, 1);
  track_data->continuous_static_frames_ = 0;
  post_processor->PostProcess(track_data);
  EXPECT_DOUBLE_EQ(2.0, latest_object->output_velocity(0));

  latest_object->belief_velocity_gain(0) = 0.0;
  latest_object->belief_velocity(0) = 2.0;
  latest_object->output_velocity(0) = 2.0;
  track_data->motion_state_ = MotionState::SKEPTICAL_MOVE;
  latest_object->motion_score = Eigen::Vector3d(1, 1, 1);
  track_data->continuous_static_frames_ = 4;
  post_processor->PostProcess(track_data);
  EXPECT_DOUBLE_EQ(0.0, latest_object->output_velocity(0));

  latest_object->belief_velocity_gain(0) = 0.0;
  latest_object->belief_velocity(0) = 2.0;
  latest_object->output_velocity(0) = 2.0;
  track_data->motion_state_ = MotionState::TRUSTED_MOVE;
  post_processor->PostProcess(track_data);
  EXPECT_DOUBLE_EQ(2.0, latest_object->output_velocity(0));

  // test is static hypothesis, velocity_noise_level_is_0
  latest_object->belief_velocity_gain(0) = 0.0;
  latest_object->belief_velocity(0) = 0.1;
  latest_object->output_velocity(0) = 0.1;
  latest_object->type = base::ObjectType::VEHICLE;
  post_processor->PostProcess(track_data);
  EXPECT_DOUBLE_EQ(0.0, latest_object->output_velocity(0));
  // test smooth orientation
  EXPECT_DOUBLE_EQ(11.0, latest_object->output_center(0));
  EXPECT_DOUBLE_EQ(0.5, latest_object->output_center(1));
  EXPECT_DOUBLE_EQ(0.0, latest_object->output_center(2));
  EXPECT_DOUBLE_EQ(2.0, latest_object->output_size(0));
  EXPECT_DOUBLE_EQ(1.0, latest_object->output_size(1));
  EXPECT_DOUBLE_EQ(1.0, latest_object->output_size(2));

  // test is static hypothesis, velocity_noise_level_is_1
  // angle change is 0
  latest_object->belief_velocity_gain(0) = 0.0;
  latest_object->belief_velocity(0) = 0.2;
  latest_object->output_velocity(0) = 0.2;
  latest_object->type = base::ObjectType::PEDESTRIAN;
  post_processor->PostProcess(track_data);
  EXPECT_DOUBLE_EQ(0.2, latest_object->output_velocity(0));
  // type is vehicle, velocity is expected 0.2
  latest_object->belief_velocity_gain(0) = 0.0;
  latest_object->belief_velocity(0) = 0.2;
  latest_object->output_velocity(0) = 0.2;
  latest_object->type = base::ObjectType::VEHICLE;
  post_processor->PostProcess(track_data);
  EXPECT_DOUBLE_EQ(0.0, latest_object->output_velocity(0));

  // test is static hypothesis, velocity_noise_level_is_1
  // angle change is 0.707
  latest_object->belief_velocity_gain(0) = 0.0;
  latest_object->belief_velocity(0) = 0.15;
  latest_object->belief_velocity(1) = 0.15;
  latest_object->output_velocity(0) = 0.15;
  latest_object->output_velocity(1) = 0.15;
  post_processor->PostProcess(track_data);
  EXPECT_DOUBLE_EQ(0.0, latest_object->output_velocity(0));

  // test smooth track orientation ouput_direction in small speed
  // and angle change scene
  EXPECT_NEAR(1.0, latest_object->output_direction(0), 0.1);
  EXPECT_NEAR(0.0, latest_object->output_direction(1), 0.1);

  // test smooth track orientation ouput_direction in large speed
  // and angle change scene
  latest_object->belief_velocity(0) = 10;
  latest_object->belief_velocity(1) = 10;
  latest_object->output_velocity(0) = 10;
  latest_object->output_velocity(1) = 10;
  latest_object->type = base::ObjectType::VEHICLE;
  post_processor->PostProcess(track_data);
  EXPECT_DOUBLE_EQ(1.0, latest_object->output_direction(0));
  EXPECT_DOUBLE_EQ(0.0, latest_object->output_direction(1));

  // if (frame != nullptr) {
  //   delete frame;
  //   frame = nullptr;
  // }
  delete frame;
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
