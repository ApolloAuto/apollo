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
#include "modules/perception/lidar/lib/interface/base_bipartite_graph_matcher.h"
#include "modules/perception/lidar/lib/object_builder/object_builder.h"
#include "modules/perception/lidar/lib/tracker/common/track_pool_types.h"
#include "modules/perception/lidar/lib/tracker/hm_tracker/hm_multi_target_tracker.h"
#include "modules/perception/lidar/lib/tracker/hm_tracker/kalman_filter.h"

namespace apollo {
namespace perception {
namespace lib {
DECLARE_string(work_root);
}

namespace lidar {

class KalmanFilterTest : public testing::Test {
 protected:
  typedef std::pair<size_t, size_t> TrackObjectPair;
  void SetUp() {
    char *cybertron_path = "CYBERTRON_PATH=";
    putenv(cybertron_path);
    char *module_path = "MODULE_PATH=";
    putenv(module_path);
    lib::FLAGS_work_root = "./lidar_test_data/lib/tracker/hm_tracker";
    object_builder_ = new ObjectBuilder();
    filter_.reset(new KalmanFilter);
    config_manager_ = lib::Singleton<lib::ConfigManager>::get_instance();
    CHECK_NOTNULL(config_manager_);
    config_manager_->Reset();
    object_builder_->Init();
  }
  void TearDown() {
    delete object_builder_;
    object_builder_ = nullptr;
    filter_.reset();
  }

 protected:
  lib::ConfigManager *config_manager_ = nullptr;
  ObjectBuilder *object_builder_ = nullptr;
  KalmanFilterPtr filter_;
  FilterOption filter_option_;
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
  }
}
void ConstructObjectsWithConstantSpeedModel(
    const std::vector<Eigen::Vector3d> &centers,
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

TEST_F(KalmanFilterTest, set_params_test) {
  KalmanFilter::SetUseAdaptive(true);
  EXPECT_TRUE(KalmanFilter::s_use_adaptive_);
  KalmanFilter::SetUseConvergenceBoostup(true);
  EXPECT_TRUE(KalmanFilter::s_use_convergence_boostup_);
  KalmanFilter::SetConvergedConfidenceMinimum(-0.2);
  EXPECT_EQ(0.0, KalmanFilter::s_converged_confidence_minimum_);
  KalmanFilter::SetConvergedConfidenceMinimum(2.0);
  EXPECT_EQ(1.0, KalmanFilter::s_converged_confidence_minimum_);
  KalmanFilter::SetParams(0.1, 0.2, 0.3, 0.4);
  EXPECT_FLOAT_EQ(0.1, KalmanFilter::s_centroid_measurement_noise_);
  EXPECT_FLOAT_EQ(0.2, KalmanFilter::s_centroid_init_velocity_variance_);
  EXPECT_FLOAT_EQ(0.3, KalmanFilter::s_propagation_variance_xy_);
  EXPECT_FLOAT_EQ(0.4, KalmanFilter::s_propagation_variance_z_);
  KalmanFilter::SetBoostupHistorySizeMinmax(3);
  EXPECT_EQ(3, KalmanFilter::s_boostup_history_size_minimum_);
  EXPECT_EQ(6, KalmanFilter::s_boostup_history_size_maximum_);
}

TEST_F(KalmanFilterTest, use_constant_speed_model) {
  // test Track()
  size_t object_num = 12;
  Eigen::Vector3d center;
  center << 1.0, 0.5, 0.5;
  Eigen::Vector3d size;
  size << 2.0, 1.0, 1.0;
  Eigen::Vector3d velocity;
  velocity << 5.0, 0.0, 0.0;
  double ref_timestamp = 1505524892.1844;
  std::vector<Eigen::Vector3d> current_centers;
  std::vector<Eigen::Vector3d> sizes;
  current_centers.resize(object_num);
  for (size_t j = 0; j < object_num; ++j) {
    for (size_t k = 0; k < 3; ++k) {
      current_centers[j](k) = center(k) + 0.1 * j * velocity(k);
    }
    sizes.push_back(size);
  }

  LidarFrame *frame(new LidarFrame);
  Eigen::Affine3d pose = Eigen::Affine3d::Identity();

  // read segments
  std::vector<base::ObjectPtr> &objects = frame->segmented_objects;
  ConstructObjectsWithConstantSpeedModel(current_centers, sizes, &objects);
  // build objects
  ObjectBuilderOptions object_builder_options;
  // velodyne height
  object_builder_options.ref_center = Eigen::Vector3d(0, 0, 0);
  object_builder_->Build(object_builder_options, frame);
  // build tracked object
  std::vector<TrackedObjectPtr> tracked_objects;
  TrackedObjectPool::Instance().BatchGet(objects.size(), &tracked_objects);
  ConstructTrackedObjects(objects, &tracked_objects, pose);
  TrackDataPtr track_data(new TrackData);
  MeasurementComputerPtr measure_computer(new MeasurementComputer);
  KalmanFilter::SetUseAdaptive(false);
  KalmanFilter::SetUseConvergenceBoostup(false);
  EXPECT_TRUE(filter_->Init(filter_option_));
  for (size_t i = 0; i < object_num - 1; ++i) {
    double timestamp = ref_timestamp + 0.1 * i;
    if (i == object_num - 2) {
      track_data->motion_state_ = MotionState::TRUSTED_MOVE;
    }
    if (i == 0) {
      filter_->UpdateWithObject(track_data, tracked_objects[i], timestamp);
      track_data->Reset(tracked_objects[i], timestamp, 0);
    } else {
      measure_computer->ComputeMeasurment(track_data, tracked_objects[i],
                                          timestamp);
      filter_->UpdateWithObject(track_data, tracked_objects[i], timestamp);
      track_data->PushTrackedObjectToTrack(tracked_objects[i], timestamp);
    }
  }
  // constant speed model, use adaptive is false,
  // use convergence boost up is false
  TrackedObjectPtr new_object = (track_data->GetLatestObject()).second;
  EXPECT_DOUBLE_EQ(1.0, new_object->update_quality);
  float breakdown_threshold = filter_->ComputeBreakdownThreshold();
  double point_epsilon = 0.1;
  double velocity_epsilon = 0.05;
  double acc_epsilon = 0.05;
  EXPECT_FLOAT_EQ(0.3, breakdown_threshold);
  EXPECT_LE(new_object->belief_velocity_gain(0), 0.3);
  EXPECT_NEAR(5.0, new_object->belief_velocity(0), velocity_epsilon);
  EXPECT_NEAR(0.0, new_object->belief_velocity(1), velocity_epsilon);
  EXPECT_NEAR(0.0, new_object->belief_acceleration(0), acc_epsilon);
  EXPECT_NEAR(6.0, new_object->belief_anchor_point(0), point_epsilon);
  EXPECT_NEAR(0.5, new_object->belief_anchor_point(1), point_epsilon);
  EXPECT_LE(new_object->belief_velocity_gain(0), 0.3);
  EXPECT_LE(new_object->belief_acceleration(0), 2.0);

  if (frame != nullptr) {
    delete frame;
    frame = nullptr;
  }
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
