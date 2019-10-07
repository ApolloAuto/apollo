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

#include "modules/tools/visualizer/abstract_camera.h"

class TargetCamera : public AbstractCamera {
 public:
  TargetCamera();

  virtual void UpdateWorld();

  const QVector3D& target_pos(void) const { return target_pos_; }
  void set_target_pos(float x, float y, float z) {
    target_pos_.setX(x);
    target_pos_.setY(y);
    target_pos_.setZ(z);

    distance_ = position_.distanceToPoint(target_pos_);
  }

  void set_target_pos(const QVector3D& tgt) {
    target_pos_ = tgt;
    distance_ = position_.distanceToPoint(target_pos_);
  }

  float distance(void) const { return distance_; }
  void set_distance(float distance) {
    if (distance < 0.0f) {
      distance = 0.0f;
    }
    distance_ = distance;
  }

  void Rotate(float xRotateDegrees, float yRotateDegrees,
              float zRotateDegrees) {
    SetAttitude(yRotateDegrees, xRotateDegrees, zRotateDegrees);
  }

 private:
  QVector3D target_pos_;

  float distance_;
};
