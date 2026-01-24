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
#include "gtest/gtest.h"

#include "cyber/common/log.h"
#include "modules/perception/common/algorithm/sensor_manager/sensor_manager.h"
#include "modules/perception/multi_sensor_fusion/base/sensor_data_manager.h"
#include "modules/perception/multi_sensor_fusion/common/camera_util.h"

namespace apollo {
namespace perception {
namespace fusion {

TEST(CameraUtilTest, test_is_pt_in_frustum) {
  double width = 1080;
  double height = 920;
  float x[3] = {-1, 500, 2000};
  float y[3] = {-1, 500, 2000};
  bool result[3] = {false, true, false};
  for (size_t i = 0; i < 3; i++) {
    for (size_t j = 0; j < 3; j++) {
      EXPECT_EQ(IsPtInFrustum<Eigen::Vector2f>(Eigen::Vector2f(x[i], y[j]),
                                               width, height),
                result[i] & result[j]);
    }
  }
}

TEST(CameraUtilTest, test_object_in_camera_view_and_is_behind_camera) {
  FLAGS_work_root =
      "/apollo/modules/perception/data/params";
  FLAGS_obs_sensor_meta_file = "sensor_meta.pb.txt";
  FLAGS_obs_sensor_intrinsic_path =
      "/apollo/modules/perception/data/params";

  // create a lidar sensor object
  base::ObjectPtr base_lidar_object(new base::Object());
  base_lidar_object->center = Eigen::Vector3d(0, 0, 15);
  base_lidar_object->size = Eigen::Vector3f(1, 1, 1);
  base_lidar_object->direction = Eigen::Vector3f(1, 0, 0);
  base::FramePtr lidar_frame(new base::Frame());
  lidar_frame->sensor_info.name = "velodyne64";
  lidar_frame->sensor_info.type = base::SensorType::VELODYNE_64;

  SensorPtr lidar_sensor(new Sensor(lidar_frame->sensor_info));
  lidar_sensor->AddFrame(lidar_frame);

  SensorFramePtr lidar_sensor_frame(new SensorFrame());
  lidar_sensor_frame->Initialize(lidar_frame, lidar_sensor);
  SensorObjectPtr lidar_sensor_object(
      new SensorObject(base_lidar_object, lidar_sensor_frame));

  Eigen::Affine3d pose(Eigen::Affine3d::Identity());
  std::string sensor_id = "front_6mm";
  base::BaseCameraModelPtr camera_model =
      algorithm::SensorManager::Instance()->GetUndistortCameraModel(sensor_id);
  EXPECT_NE(camera_model, nullptr);

  bool flag1 = IsObjectEightVerticesAllBehindCamera(
      base_lidar_object, pose.matrix(), camera_model);
  EXPECT_FALSE(flag1);

  float view_ratio1 = ObjectInCameraView(lidar_sensor_object, camera_model,
                                         pose, 0.0, 100, false, false);
  float view_ratio2 = ObjectInCameraView(lidar_sensor_object, camera_model,
                                         pose, 0.0, 100, true, false);
  float view_ratio3 = ObjectInCameraView(lidar_sensor_object, camera_model,
                                         pose, 0.0, 100, false, true);
  float view_ratio4 = ObjectInCameraView(lidar_sensor_object, camera_model,
                                         pose, 0.0, 100, true, true);
  EXPECT_NEAR(view_ratio1, 1.0, 1e-1);
  EXPECT_NEAR(view_ratio2, 1.0, 1e-1);
  EXPECT_NEAR(view_ratio3, 1.0, 1e-1);
  EXPECT_NEAR(view_ratio4, 1.0, 1e-1);

  base_lidar_object->lidar_supplement.cloud_world.resize(1);
  base_lidar_object->lidar_supplement.cloud_world[0].x = 0.0;
  base_lidar_object->lidar_supplement.cloud_world[0].y = 0.0;
  base_lidar_object->lidar_supplement.cloud_world[0].z = 15;
  float view_ratio5 = ObjectInCameraView(lidar_sensor_object, camera_model,
                                         pose, 0.0, 100, false, false);
  float view_ratio6 = ObjectInCameraView(lidar_sensor_object, camera_model,
                                         pose, 0.0, 100, true, false);
  float view_ratio7 = ObjectInCameraView(lidar_sensor_object, camera_model,
                                         pose, 0.0, 100, false, true);
  float view_ratio8 = ObjectInCameraView(lidar_sensor_object, camera_model,
                                         pose, 0.0, 100, true, true);
  EXPECT_NEAR(view_ratio5, 1.0, 1e-1);
  EXPECT_NEAR(view_ratio6, 1.0, 1e-1);
  EXPECT_NEAR(view_ratio7, 1.0, 1e-1);
  EXPECT_NEAR(view_ratio8, 1.0, 1e-1);

  base_lidar_object->lidar_supplement.cloud_world[0].x = 100000.0;
  base_lidar_object->lidar_supplement.cloud_world[0].y = 100000.0;
  base_lidar_object->lidar_supplement.cloud_world[0].z = 0.0;
  float view_ratio9 = ObjectInCameraView(lidar_sensor_object, camera_model,
                                         pose, 0.0, 100, false, false);
  EXPECT_NEAR(view_ratio9, 0.0, 1e-1);

  base_lidar_object->lidar_supplement.cloud_world.clear();
  float view_ratio10 = ObjectInCameraView(lidar_sensor_object, camera_model,
                                          pose, 0.0, 100, false, false);
  EXPECT_NEAR(view_ratio10, 1.0, 1e-1);

  base_lidar_object->center = Eigen::Vector3d(10000.0, 100000.0, 0.0);
  float view_ratio11 = ObjectInCameraView(lidar_sensor_object, camera_model,
                                          pose, 0.0, 100, true, false);
  EXPECT_NEAR(view_ratio11, 0.0, 1e-1);

  base_lidar_object->size = Eigen::Vector3f(0, 1, 1);
  float view_ratio12 = ObjectInCameraView(lidar_sensor_object, camera_model,
                                          pose, 0.0, 100, true, false);
  EXPECT_NEAR(view_ratio12, 0.0, 1e-1);

  base_lidar_object->center = Eigen::Vector3d(-10000, -10000, -10000);
  bool flag2 = IsObjectEightVerticesAllBehindCamera(
      base_lidar_object, pose.matrix(), camera_model);
  EXPECT_TRUE(flag2);
}

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
