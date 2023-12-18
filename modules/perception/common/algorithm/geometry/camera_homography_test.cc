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
#include "modules/perception/common/algorithm/geometry/camera_homography.h"

#include "gtest/gtest.h"

#include "modules/perception/common/base/camera.h"

namespace apollo {
namespace perception {
namespace algorithm {

TEST(CameraHomographyTest, is_camera_overlap_test) {
  base::PinholeCameraModel camera1;
  camera1.set_width(1080);
  camera1.set_height(720);
  Eigen::Matrix3f intrinsic_params = Eigen::Matrix3f::Zero();
  intrinsic_params(0, 0) = 1460;
  intrinsic_params(0, 2) = 508;
  intrinsic_params(1, 1) = 1480;
  intrinsic_params(1, 2) = 364;
  intrinsic_params(2, 2) = 1;
  camera1.set_intrinsic_params(intrinsic_params);

  base::PinholeCameraModel camera2;
  camera2.set_width(5);
  camera2.set_height(10);
  Eigen::Matrix3f intrinsic_params2 = Eigen::Matrix3f::Zero();
  intrinsic_params2(0, 0) = 1460;
  intrinsic_params2(0, 2) = 508;
  intrinsic_params2(1, 1) = 1480;
  intrinsic_params2(1, 2) = 364;
  intrinsic_params2(2, 2) = 1;
  camera2.set_intrinsic_params(intrinsic_params2);

  Eigen::Vector2d up_left;
  Eigen::Vector2d low_right;
  Eigen::Matrix4d extrinsic;
  extrinsic << 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
      0.0, 0.0, 1.0;
  bool flag =
      IsCamerasFieldOverlap(camera1, camera2, extrinsic, &up_left, &low_right);
  EXPECT_TRUE(flag);

  Eigen::Matrix4d extrinsic2;
  extrinsic2 << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0;
  bool flag2 =
      IsCamerasFieldOverlap(camera1, camera2, extrinsic2, &up_left, &low_right);
  EXPECT_FALSE(flag2);
}

}  //  namespace algorithm
}  //  namespace perception
}  //  namespace apollo
