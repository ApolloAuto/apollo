/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
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

#ifndef MODULES_PERCEPTION_OBSTACLE_LIDAR_VISUALIZER_CAMERA_H_
#define MODULES_PERCEPTION_OBSTACLE_LIDAR_VISUALIZER_CAMERA_H_

#include <Eigen/Dense>

namespace apollo {
namespace perception {

class Camera {
 public:
  Camera();
  ~Camera() = default;

  void SetPosition(Eigen::Vector3d world_pos) {
    t_c2w_ = world_pos;
  }
  void SetUpDirection(Eigen::Vector3d world_vec);
  void SetFov(double fov_deg) {
    fov_ = fov_deg;
  }
  void SetScreenWidthHeight(int width, int height);

  Eigen::Vector3d Position() {
    return t_c2w_;
  }
  Eigen::Vector3d UpDir();

  void LookAt(Eigen::Vector3d world_tgt);

  Eigen::Matrix4d GetViewMat();
  Eigen::Matrix4d GetProjectionMat();

  Eigen::Vector3d PointOnScreen(Eigen::Vector3d point);

 private:
  Eigen::Matrix3d rot_c2w_;
  Eigen::Vector3d t_c2w_;
  double fov_;
  int screen_width_;
  int screen_height_;
  Eigen::Matrix4d view_mat_;
  Eigen::Matrix4d proj_mat_;
};

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_LIDAR_VISUALIZER_CAMERA_H_
