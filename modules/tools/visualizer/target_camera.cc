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

#include "modules/tools/visualizer/target_camera.h"

TargetCamera::TargetCamera()
    : AbstractCamera(), target_pos_(0.0, 0.0, 0.0), distance_(10.0) {}

void TargetCamera::UpdateWorld() {
  QMatrix4x4 R = YawPitchRoll(yaw(), pitch(), roll());
  QVector3D T{0, 0, distance_};
  T = QVector3D(R * QVector4D(T, 0.0f));
  position_ = target_pos_ + T;
  look_ = target_pos_ - position_;
  look_.normalize();

  up_ = QVector3D(R * QVector4D(UP, 0.0f));
  right_ = QVector3D::crossProduct(look_, up_);

  model_view_mat_.setToIdentity();
  model_view_mat_.lookAt(position_, target_pos_, up_);
}
