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

#include "modules/tools/visualizer/abstract_camera.h"
#include <cmath>

const QVector3D AbstractCamera::UP{0.0f, 1.0f, 0.0f};

QMatrix4x4 AbstractCamera::YawPitchRoll(float yaw, float pitch, float roll) {
  yaw = Radians(yaw);
  pitch = Radians(pitch);
  roll = Radians(roll);

  register float tmpCy = std::cos(yaw);
  register float tmpSy = std::sin(yaw);
  register float tmpCp = std::cos(pitch);
  register float tmpSp = std::sin(pitch);
  register float tmpCr = std::cos(roll);
  register float tmpSr = std::sin(roll);

  QMatrix4x4 Result;
  Result(0, 0) = tmpCy * tmpCr + tmpSy * tmpSp * tmpSr;
  Result(1, 0) = tmpSr * tmpCp;
  Result(2, 0) = -tmpSy * tmpCr + tmpCy * tmpSp * tmpSr;
  Result(3, 0) = 0.0f;
  Result(0, 1) = -tmpCy * tmpSr + tmpSy * tmpSp * tmpCr;
  Result(1, 1) = tmpCr * tmpCp;
  Result(2, 1) = tmpSr * tmpSy + tmpCy * tmpSp * tmpCr;
  Result(3, 1) = 0.0f;
  Result(0, 2) = tmpSy * tmpCp;
  Result(1, 2) = -tmpSp;
  Result(2, 2) = tmpCy * tmpCp;
  Result(3, 2) = 0.0f;
  Result(0, 3) = 0.0f;
  Result(1, 3) = 0.0f;
  Result(2, 3) = 0.0f;
  Result(3, 3) = 1.0f;
  return Result;
}

AbstractCamera::AbstractCamera()
    : camera_mode_(CameraMode::PerspectiveMode),
      fov_(45.0f),
      near_plane_width_(1.0f),
      near_plane_height_(1.0f),
      near_plane_(0.1f),
      far_plane_(1000.0f),
      position_(0.0f, 0.0f, 0.0f),
      attitude_(),
      look_(0.0f, 0.0f, 1.0f),
      up_(UP),
      right_(1.0f, 0.0f, 0.0f),
      projection_mat_(),
      model_view_mat_() {}
