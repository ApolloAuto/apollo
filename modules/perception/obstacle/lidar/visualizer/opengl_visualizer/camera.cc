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

#include "modules/perception/obstacle/lidar/visualizer/opengl_visualizer/camera.h"
#include <GL/glut.h>
#include <math.h>

namespace apollo {
namespace perception {

Camera::Camera()
    : rot_c2w_(Eigen::Matrix3d::Identity()),
      t_c2w_(0, 0, 0),
      fov_(45.0),
      screen_width_(600),
      screen_height_(600),
      view_mat_(Eigen::Matrix4d::Identity()),
      proj_mat_(Eigen::Matrix4d::Zero()) {}

void Camera::SetUpDirection(Eigen::Vector3d world_vec) {
  Eigen::Vector3d up_vec_cam_frame = rot_c2w_.inverse() * world_vec;
  Eigen::Vector3d up_vec_proj_in_cam_frame(up_vec_cam_frame(0),
                                           up_vec_cam_frame(1), 0);
  up_vec_proj_in_cam_frame.normalize();
  Eigen::Vector3d cam_y_axis = Eigen::Vector3d(0, 1, 0);
  Eigen::Vector3d align_axis = cam_y_axis.cross(up_vec_proj_in_cam_frame);
  align_axis
      .normalize();  // the axis for Eigen::AngleAxisd must be a unit vector
  double align_ang = acos(cam_y_axis.dot(up_vec_proj_in_cam_frame));
  Eigen::Matrix3d align_rot_mat =
      Eigen::AngleAxisd(align_ang, align_axis).toRotationMatrix();

  rot_c2w_ = rot_c2w_ * align_rot_mat;
}

void Camera::SetScreenWidthHeight(int width, int height) {
  screen_width_ = width;
  screen_height_ = height;
}

Eigen::Vector3d Camera::UpDir() {
  Eigen::Vector3d up_dir = rot_c2w_ * Eigen::Vector3d(0, 1, 0);
  return up_dir;
}

void Camera::LookAt(Eigen::Vector3d world_tgt) {
  Eigen::Vector3d view_dir = world_tgt - Position();
  view_dir.normalize();
  Eigen::Vector3d right_dir = view_dir.cross(UpDir());
  right_dir.normalize();
  Eigen::Vector3d up_dir = right_dir.cross(view_dir);
  up_dir.normalize();

  rot_c2w_.col(0) = right_dir;
  rot_c2w_.col(1) = up_dir;
  rot_c2w_.col(2) = -view_dir;  // view_dir == minor z axis
}

Eigen::Matrix4d Camera::GetViewMat() {
  Eigen::Matrix4d view_mat = Eigen::Matrix4d::Identity();
  view_mat.topLeftCorner(3, 3) = rot_c2w_;
  view_mat.topRightCorner(3, 1) = t_c2w_;
  view_mat_ = view_mat.inverse();

  return view_mat_;
}

Eigen::Matrix4d Camera::GetProjectionMat() {
  double z_near = 1.0;
  double z_far = 1000.0;
  GLdouble aspect_ratio = static_cast<GLdouble>(screen_width_) /
                          static_cast<GLdouble>(screen_height_);
  double fov_arc = fov_ * M_PI / 180.0;
  double f = 1.0 / tan(fov_arc / 2.0);
  proj_mat_(0, 0) = f / aspect_ratio;
  proj_mat_(1, 1) = f;
  proj_mat_(2, 2) = (z_near + z_far) / (z_near - z_far);
  proj_mat_(3, 2) = -1.0;
  proj_mat_(2, 3) = 2.0 * z_near * z_far / (z_near - z_far);

  return proj_mat_;
}

Eigen::Vector3d Camera::PointOnScreen(Eigen::Vector3d point) {
  GLdouble x = 0.0;
  GLdouble y = 0.0;
  GLdouble z = 0.0;
  static GLint viewport[4];
  viewport[0] = 0;
  viewport[1] = screen_height_;
  viewport[2] = screen_width_;
  viewport[3] =
      -screen_height_;  // origin of opengl frame is at left-bottom corner

  GLdouble view_mat[16] = {
      view_mat_(0, 0), view_mat_(1, 0), view_mat_(2, 0), view_mat_(3, 0),
      view_mat_(0, 1), view_mat_(1, 1), view_mat_(2, 1), view_mat_(3, 1),
      view_mat_(0, 2), view_mat_(1, 2), view_mat_(2, 2), view_mat_(3, 2),
      view_mat_(0, 3), view_mat_(1, 3), view_mat_(2, 3), view_mat_(3, 3)};

  GLdouble proj_mat[16] = {
      proj_mat_(0, 0), proj_mat_(1, 0), proj_mat_(2, 0), proj_mat_(3, 0),
      proj_mat_(0, 1), proj_mat_(1, 1), proj_mat_(2, 1), proj_mat_(3, 1),
      proj_mat_(0, 2), proj_mat_(1, 2), proj_mat_(2, 2), proj_mat_(3, 2),
      proj_mat_(0, 3), proj_mat_(1, 3), proj_mat_(2, 3), proj_mat_(3, 3)};
  // here the view_mat should be view_mode_mat acutally
  gluProject(point(0), point(1), point(2), view_mat, proj_mat, viewport, &x, &y,
             &z);

  return Eigen::Vector3d(x, y, z);
}

}  // namespace perception
}  // namespace apollo
