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
#include <memory>

#include "modules/perception/base/omnidirectional_model.h"

namespace apollo {
namespace perception {
namespace base {

TEST(DistortionModelTest, omnidirectional_model_get_set_test) {
  OmnidirectionalCameraDistortionModel distortion_model;
  EXPECT_EQ(distortion_model.name(), "OmnidirectionalCameraDistortionModel");

  Eigen::VectorXf params;
  size_t w = 1920;
  size_t h = 1080;

  EXPECT_FALSE(distortion_model.set_params(w, h, params));
  params.resize(9);
  EXPECT_FALSE(distortion_model.set_params(w, h, params));
  Eigen::Matrix3f intrinsic_params = Eigen::Matrix3f::Zero();
  intrinsic_params(0, 0) = 300;
  intrinsic_params(0, 2) = 960;
  intrinsic_params(1, 1) = 300;
  intrinsic_params(1, 2) = 540;
  intrinsic_params(2, 2) = 1;

  params.resize(28);
  params(0) = 559.892944;  // center["x"]
  params(1) = 943.844226;  // center["y"]
  params(2) = 0.998701;    // affine["c"]
  params(3) = -0.000375;   // affine["d"]
  params(4) = -0.000110;   // affine["e"]
  params(5) = 300;         // focallength
  params(6) = 960;         // principal["x"]
  params(7) = 540;         // principal["y"]
  params(8) = 5;           // cam2world order
  params(9) = -4.703335e+02;
  params(10) = 0.000000e+00;
  params(11) = 9.691211e-04;
  params(12) = -9.422097e-07;
  params(13) = 6.870575e-10;
  params(14) = 13;  // world2cam order
  params(15) = 842.297583;
  params(16) = 663.510034;
  params(17) = -2.267809;
  params(18) = -93.711075;
  params(19) = 153.354928;
  params(20) = 125.704918;
  params(21) = -255.000729;
  params(22) = -217.999819;
  params(23) = 324.729462;
  params(24) = 619.810504;
  params(25) = 417.824966;
  params(26) = 133.771693;
  params(27) = 17.074414;

  EXPECT_TRUE(distortion_model.set_params(w, h, params));
  std::shared_ptr<BaseCameraModel> camera_model =
      distortion_model.get_camera_model();
  EXPECT_TRUE(camera_model != nullptr);
  std::shared_ptr<PinholeCameraModel> pinhole_camera_model =
      std::static_pointer_cast<PinholeCameraModel>(camera_model);
  EXPECT_TRUE(pinhole_camera_model != nullptr);
  EXPECT_TRUE(pinhole_camera_model->get_width() == w);
  EXPECT_TRUE(pinhole_camera_model->get_height() == h);
  Eigen::Matrix3f ret_intrinsic_mat =
      pinhole_camera_model->get_intrinsic_params();
  for (size_t i = 0; i < 3; ++i) {
    for (size_t j = 0; j < 3; ++j) {
      EXPECT_TRUE(fabs(intrinsic_params(i, j) - ret_intrinsic_mat(i, j)) <
                  1.0e-6);
    }
  }
}

TEST(DistortionModelTest, distortion_model_project_test) {
  OmnidirectionalCameraDistortionModel distortion_model;
  EXPECT_EQ(distortion_model.name(), "OmnidirectionalCameraDistortionModel");

  Eigen::VectorXf params;
  size_t w = 1920;
  size_t h = 1080;

  params.resize(28);
  params(0) = 559.892944;  // center["x"]
  params(1) = 943.844226;  // center["y"]
  params(2) = 0.998701;    // affine["c"]
  params(3) = -0.000375;   // affine["d"]
  params(4) = -0.000110;   // affine["e"]
  params(5) = 300;         // focallength
  params(6) = 960;         // principal["x"]
  params(7) = 540;         // principal["y"]
  params(8) = 5;           // cam2world order
  params(9) = -4.703335e+02;
  params(10) = 0.000000e+00;
  params(11) = 9.691211e-04;
  params(12) = -9.422097e-07;
  params(13) = 6.870575e-10;
  params(14) = 13;  // world2cam order
  params(15) = 842.297583;
  params(16) = 663.510034;
  params(17) = -2.267809;
  params(18) = -93.711075;
  params(19) = 153.354928;
  params(20) = 125.704918;
  params(21) = -255.000729;
  params(22) = -217.999819;
  params(23) = 324.729462;
  params(24) = 619.810504;
  params(25) = 417.824966;
  params(26) = 133.771693;
  params(27) = 17.074414;

  EXPECT_TRUE(distortion_model.set_params(w, h, params));

  std::shared_ptr<BaseCameraModel> camera_model =
      distortion_model.get_camera_model();
  EXPECT_TRUE(camera_model != nullptr);
  std::shared_ptr<PinholeCameraModel> pinhole_camera_model =
      std::static_pointer_cast<PinholeCameraModel>(camera_model);
  EXPECT_TRUE(pinhole_camera_model != nullptr);
  EXPECT_TRUE(pinhole_camera_model->get_width() == w);
  EXPECT_TRUE(pinhole_camera_model->get_height() == h);

  Eigen::Vector3f pt3d(0, 0, 50);
  Eigen::Vector2f pt2d(943.844226, 559.892944);

  Eigen::Vector2f proj2d_distort = distortion_model.Project(pt3d);
  EXPECT_NEAR(pt2d[0], proj2d_distort[0], 1e-2);
  EXPECT_NEAR(pt2d[1], proj2d_distort[1], 1e-2);
}

}  //  namespace base
}  //  namespace perception
}  //  namespace apollo
