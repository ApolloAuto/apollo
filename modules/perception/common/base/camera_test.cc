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

#include "modules/perception/common/base/camera.h"

namespace apollo {
namespace perception {
namespace base {

TEST(CameraModelTest, camera_model_get_set_test) {
  PinholeCameraModel camera_model;
  EXPECT_EQ(camera_model.get_width(), 0);
  EXPECT_EQ(camera_model.get_height(), 0);

  camera_model.set_width(1920);
  camera_model.set_height(1080);
  EXPECT_EQ(camera_model.get_width(), 1920);
  EXPECT_EQ(camera_model.get_height(), 1080);

  EXPECT_EQ(camera_model.name(), "PinholeCameraModel");
}

TEST(PinholeCameraModelTest, camera_model_project_test) {
  PinholeCameraModel camera_model;
  camera_model.set_width(1080);
  camera_model.set_height(720);
  Eigen::Matrix3f intrinsic_params = Eigen::Matrix3f::Zero();
  intrinsic_params(0, 0) = 1460;
  intrinsic_params(0, 2) = 508;
  intrinsic_params(1, 1) = 1480;
  intrinsic_params(1, 2) = 364;
  intrinsic_params(2, 2) = 1;

  camera_model.set_intrinsic_params(intrinsic_params);

  Eigen::Matrix3f ret_intrinsic = camera_model.get_intrinsic_params();
  for (size_t i = 0; i < 3; ++i) {
    for (size_t j = 0; j < 3; ++j) {
      EXPECT_LT(fabs(intrinsic_params(i, j) - ret_intrinsic(i, j)), 1.0e-6);
    }
  }
  Eigen::Vector3f pt3d(10, 20, 50);
  Eigen::Vector2f pt2d(800, 956);
  Eigen::Vector2f proj2d = camera_model.Project(pt3d);

  EXPECT_LT((pt2d - proj2d).norm(), 1.0e-6);

  Eigen::Vector3f uproj3d = camera_model.UnProject(pt2d);
  EXPECT_TRUE(
      (uproj3d - Eigen::Vector3f(pt3d[0] / pt3d[2], pt3d[1] / pt3d[2], 1.0f))
          .norm() < 1.0e-6);
}

}  //  namespace base
}  //  namespace perception
}  //  namespace apollo
