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

#pragma once

#include <QtGui/QMatrix4x4>

class AbstractCamera {
 public:
  enum class CameraMode { PerspectiveMode, OrthoMode };

  static const QVector3D UP;
  static float Radians(float degrees) {
    return degrees * static_cast<float>(0.01745329251994329576923690768489);
  }

  static float Degrees(float radians) {
    return radians * static_cast<float>(57.295779513082320876798154814105);
  }

  static QMatrix4x4 YawPitchRoll(float yawInDegrees, float picthInDegrees,
                                 float rollInDegrees);

  AbstractCamera(void);
  virtual ~AbstractCamera() {}

  virtual void UpdateWorld(void) = 0;  // update modelview

  CameraMode camera_mode(void) const { return camera_mode_; }
  void set_camera_mode(CameraMode cm) { camera_mode_ = cm; }

  const QMatrix4x4& projection_matrix(void) const { return projection_mat_; }
  const QMatrix4x4& model_view_matrix(void) const { return model_view_mat_; }

  float near_plane_height(void) const { return near_plane_height_; }
  float near_plane_width(void) const { return near_plane_width_; }
  void set_near_plane_height(const float npHeight) {
    near_plane_height_ = npHeight;
  }
  void set_near_plane_width(const float npWidth) {
    near_plane_width_ = npWidth;
  }

  float fov(void) const { return fov_; }
  void set_fov(const float fovInDegrees) { fov_ = fovInDegrees; }

  float near_plane(void) const { return near_plane_; }
  void set_near_plane(float n) { near_plane_ = n; }

  float far_plane(void) const { return far_plane_; }
  void set_far_plane(float f) { far_plane_ = f; }

  void SetUpProjection(float fovInDegrees, float nearPlaneWidth,
                       float nearPlaneHeight, float near = 0.1f,
                       float far = 1000.f) {
    fov_ = fovInDegrees;
    near_plane_width_ = nearPlaneWidth;
    near_plane_height_ = nearPlaneHeight;
    near_plane_ = near;
    far_plane_ = far;
  }

  float x(void) const { return position_[0]; }
  float y(void) const { return position_[1]; }
  float z(void) const { return position_[2]; }

  void set_x(float x) { position_[0] = x; }
  void set_y(float y) { position_[1] = y; }
  void set_z(float z) { position_[2] = z; }

  const QVector3D& position(void) const { return position_; }
  void set_position(const QVector3D& pos) { position_ = pos; }
  void set_position(float x, float y, float z) {
    position_.setX(x);
    position_.setY(y);
    position_.setZ(z);
  }

  float yaw(void) const { return attitude_[0]; }
  void set_yaw(float yInDegrees) { attitude_[0] = yInDegrees; }

  float pitch(void) const { return attitude_[1]; }
  void set_pitch(float pInDegrees) { attitude_[1] = pInDegrees; }

  float roll(void) const { return attitude_[2]; }
  void set_roll(float rInDegrees) { attitude_[2] = rInDegrees; }

  const QVector3D& attitude(void) const { return attitude_; }

  void SetAttitude(float yawInDegrees, float pitchInDegrees,
                   float rollInDegrees) {
    attitude_[0] = yawInDegrees;
    attitude_[1] = pitchInDegrees;
    attitude_[2] = rollInDegrees;
  }

  const QVector3D& look(void) const { return look_; }

  void UpdateProjection(void) {
    projection_mat_.setToIdentity();
    if (camera_mode() == CameraMode::PerspectiveMode) {
      projection_mat_.perspective(fov_, near_plane_width_ / near_plane_height_,
                                  near_plane_, far_plane_);
    } else {
      projection_mat_.ortho(-near_plane_width_ / 2.0f, near_plane_width_ / 2.0f,
                            -near_plane_height_ / 2.0f,
                            near_plane_height_ / 2.0f, 0.0f, 0.0f);
    }
  }

  void Update(void) {
    UpdateWorld();
    UpdateProjection();
  }

 protected:
  CameraMode camera_mode_;

  float fov_;  // in degrees

  float near_plane_width_;
  float near_plane_height_;

  float near_plane_;  // in look direction
  float far_plane_;   // in look direction

  QVector3D position_;  // x, y, z
  QVector3D attitude_;  // 0:yaw, 1:pitch, 2:roll , in degrees

  QVector3D look_;
  QVector3D up_;
  QVector3D right_;

  QMatrix4x4 projection_mat_;
  QMatrix4x4 model_view_mat_;
};
