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

#include "modules/perception/base/distortion_model.h"

namespace apollo {
namespace perception {
namespace base {

TEST(DistortionModelTest, distortion_model_get_set_test) {
  BrownCameraDistortionModel distortion_model;
  EXPECT_EQ(distortion_model.name(), "BrownCameraDistortionModel");

  Eigen::VectorXf params;
  size_t w = 1920;
  size_t h = 1080;

  EXPECT_FALSE(distortion_model.set_params(w, h, params));
  params.resize(9);
  EXPECT_FALSE(distortion_model.set_params(w, h, params));

  Eigen::Matrix3f intrinsic_params = Eigen::Matrix3f::Zero();
  intrinsic_params(0, 0) = 1460;
  intrinsic_params(0, 2) = 508;
  intrinsic_params(1, 1) = 1480;
  intrinsic_params(1, 2) = 364;
  intrinsic_params(2, 2) = 1;

  params.resize(14);
  params(0) = 1460;
  params(1) = 0;
  params(2) = 508;
  params(3) = 0;
  params(4) = 1480;
  params(5) = 364;
  params(6) = 0;
  params(7) = 0;
  params(8) = 1;
  params(9) = 0.001f;
  params(10) = 0.001f;
  params(11) = 0.1f;
  params(12) = 0.001f;
  params(13) = 0.002f;

  EXPECT_TRUE(distortion_model.set_params(w, h, params));

  std::shared_ptr<BaseCameraModel> camera_model =
      distortion_model.get_camera_model();
  EXPECT_NE(camera_model, nullptr);
  std::shared_ptr<PinholeCameraModel> pinhole_camera_model =
      std::static_pointer_cast<PinholeCameraModel>(camera_model);
  EXPECT_NE(pinhole_camera_model, nullptr);
  EXPECT_EQ(pinhole_camera_model->get_width(), w);
  EXPECT_EQ(pinhole_camera_model->get_height(), h);
  Eigen::Matrix3f ret_intrinsic_mat =
      pinhole_camera_model->get_intrinsic_params();
  for (size_t i = 0; i < 3; ++i) {
    for (size_t j = 0; j < 3; ++j) {
      EXPECT_LT(fabs(intrinsic_params(i, j) - ret_intrinsic_mat(i, j)), 1.0e-6);
    }
  }
}

TEST(DistortionModelTest, distortion_model_project_test) {
  BrownCameraDistortionModel distortion_model;
  EXPECT_EQ(distortion_model.name(), "BrownCameraDistortionModel");

  Eigen::VectorXf params;
  size_t w = 1080;
  size_t h = 720;

  params.resize(14);
  params(0) = 1460;
  params(1) = 0;
  params(2) = 508;
  params(3) = 0;
  params(4) = 1480;
  params(5) = 364;
  params(6) = 0;
  params(7) = 0;
  params(8) = 1;
  params(9) = 0.f;
  params(10) = 0.f;
  params(11) = 0.f;
  params(12) = 0.f;
  params(13) = 0.f;

  EXPECT_TRUE(distortion_model.set_params(w, h, params));

  std::shared_ptr<BaseCameraModel> camera_model =
      distortion_model.get_camera_model();
  EXPECT_NE(camera_model, nullptr);
  std::shared_ptr<PinholeCameraModel> pinhole_camera_model =
      std::static_pointer_cast<PinholeCameraModel>(camera_model);
  EXPECT_NE(pinhole_camera_model, nullptr);
  EXPECT_EQ(pinhole_camera_model->get_width(), w);
  EXPECT_EQ(pinhole_camera_model->get_height(), h);

  Eigen::Vector3f pt3d(10, 20, 50);
  Eigen::Vector2f pt2d(800, 956);
  // distort_params all zeros, equal to PinholeCameraModel::Project()
  Eigen::Vector2f proj2d = camera_model->Project(pt3d);
  Eigen::Vector2f proj2d_distort = distortion_model.Project(pt3d);
  EXPECT_EQ(proj2d[0], proj2d_distort[0]);
  EXPECT_EQ(proj2d[1], proj2d_distort[1]);

  params(9) = 0.2f;
  params(10) = 0.1f;
  params(11) = 0.002f;
  params(12) = -0.001f;
  params(13) = 0.f;

  EXPECT_TRUE(distortion_model.set_params(w, h, params));
  proj2d_distort = distortion_model.Project(pt3d);
  EXPECT_NEAR(proj2d[0], proj2d_distort[0], 30);
  EXPECT_NEAR(proj2d[1], proj2d_distort[1], 30);
}

}  //  namespace base
}  //  namespace perception
}  //  namespace apollo
