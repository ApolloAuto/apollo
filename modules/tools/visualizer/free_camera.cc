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

#include "modules/tools/visualizer/free_camera.h"

FreeCamera::FreeCamera(void)
    : AbstractCamera(), translation_(0.0, 0.0f, 0.0f) {}

void FreeCamera::UpdateWorld(void) {
  QMatrix4x4 R = YawPitchRoll(attitude_[0], attitude_[1], attitude_[2]);

  position_ += translation_;
  translation_.setX(0.0f);
  translation_.setY(0.0f);
  translation_.setZ(0.0f);

  look_ = QVector3D(R * QVector4D(0.0f, 0.0f, 1.0f, 0.0f));
  up_ = QVector3D(R * QVector4D(0.0f, 1.0f, 0.0f, 0.0f));
  right_ = QVector3D::crossProduct(look_, up_);

  QVector3D tgt = position_ + look_;
  model_view_mat_.setToIdentity();
  model_view_mat_.lookAt(position_, tgt, up_);
}
