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

#include "modules/perception/common/sensor_manager/sensor_manager.h"
#include "modules/perception/fusion/base/base_init_options.h"
#include "modules/perception/fusion/base/sensor.h"
#include "modules/perception/fusion/base/sensor_frame.h"
#include "modules/perception/fusion/base/sensor_object.h"
#include "modules/perception/fusion/base/track.h"

#define private public
#define protected public
#include "modules/perception/fusion/lib/data_fusion/tracker/pbf_tracker/pbf_tracker.h"

namespace apollo {
namespace perception {
namespace fusion {

/* TODO(all): Initialize() not compiling. to be fixed
TEST(PbfTrackerTest, test_initialize) {
  FLAGS_work_root = "/apollo/modules/perception/testdata/fusion/pbf_tracker";
  FLAGS_obs_sensor_meta_path = "./data/sensor_meta.pt";
  FLAGS_obs_sensor_intrinsic_path =
      "/apollo/modules/perception/testdata/fusion/pbf_tracker/params";

  EXPECT_TRUE(PbfTracker::InitParams());

  PbfTracker tracker;
  TrackPtr track(new Track());
  EXPECT_EQ(tracker.Name(), "PbfTracker");

  base::SensorInfo lidar_sensor_info;
  lidar_sensor_info.type = base::SensorType::VELODYNE_64;
  lidar_sensor_info.name = "velodyne64";
  SensorPtr lidar_sensor(new Sensor(lidar_sensor_info));

  Sensor::SetMaxCachedFrameNumber(2);

  double timestamp = 123456789.1;
  Eigen::Affine3d sensor2world_pose = Eigen::Affine3d::Identity();
  base::ObjectPtr base_obj_1(new base::Object());
  base_obj_1->latest_tracked_time = timestamp;
  base::FramePtr base_frame_1(new base::Frame());
  base_frame_1->timestamp = timestamp;
  base_frame_1->sensor2world_pose = sensor2world_pose;
  base_frame_1->objects.emplace_back(base_obj_1);
  base_frame_1->sensor_info = lidar_sensor_info;
  SensorFramePtr frame_1(new SensorFrame());
  //frame_1->Initialize(base_frame_1, lidar_sensor);

  SensorObjectPtr sensor_obj = frame_1->GetForegroundObjects()[0];
  EXPECT_TRUE(tracker.Init(track, sensor_obj));

  PbfTracker::s_type_fusion_method_ = "unknown";
  EXPECT_FALSE(tracker.Init(track, sensor_obj));
  EXPECT_FALSE(tracker.InitMethods());

  PbfTracker::s_type_fusion_method_ = "DstTypeFusion";
  PbfTracker::s_motion_fusion_method_ = "unknown";
  EXPECT_FALSE(tracker.InitMethods());

  PbfTracker::s_motion_fusion_method_ = "KalmanMotionFusion";
  PbfTracker::s_existance_fusion_method_ = "unknown";
  EXPECT_FALSE(tracker.InitMethods());

  PbfTracker::s_existance_fusion_method_ = "DstExistanceFusion";
  PbfTracker::s_shape_fusion_method_ = "unknown";
  EXPECT_FALSE(tracker.InitMethods());

  PbfTracker::s_shape_fusion_method_ = "PbfShapeFusion";
  PbfTracker::s_type_fusion_method_ = "unknown";
  EXPECT_FALSE(tracker.InitMethods());
}
*/

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
