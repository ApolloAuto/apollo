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

#include <string>

#include "modules/perception/lidar/lib/interface/base_multi_target_tracker.h"

namespace apollo {
namespace perception {
namespace lidar {

class DummyMultiTargetTracker : public BaseMultiTargetTracker {
 public:
  DummyMultiTargetTracker() = default;

  virtual ~DummyMultiTargetTracker() = default;

  bool Init(const MultiTargetTrackerInitOptions& options =
                MultiTargetTrackerInitOptions()) override;

  // @brief: track segmented objects, and estimate motion
  // @param [in]: options
  // @param [in/out]: tracked object
  // tracked objects should be filled, required,
  bool Track(const MultiTargetTrackerOptions& options,
             LidarFrame* frame) override;

  std::string Name() const override { return "DummyMultiTargetTracker"; }

 private:
  int id_ = 0;
};  // class DummyMultiTargetTracker

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
