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

#include "cyber/common/macros.h"
#include "modules/perception/common/lib/interface/base_init_options.h"
#include "modules/perception/common/lib/registerer/registerer.h"
#include "modules/perception/common/lidar/common/lidar_frame.h"

namespace apollo {
namespace perception {
namespace lidar {

using apollo::perception::BaseInitOptions;

struct MultiTargetTrackerInitOptions : public BaseInitOptions {};

struct MultiTargetTrackerOptions {};

class BaseMultiTargetTracker {
 public:
  BaseMultiTargetTracker() = default;

  virtual ~BaseMultiTargetTracker() = default;

  /**
   * @brief Init multi target tarcker
   *
   * @param options
   * @return true
   * @return false
   */
  virtual bool Init(const MultiTargetTrackerInitOptions& options =
                        MultiTargetTrackerInitOptions()) = 0;

  /**
   * @brief Track segmented objects and estimate motion and shape
   *
   * @param options
   * @param frame location to save tracked object
   * @return true
   * @return false
   */
  virtual bool Track(const MultiTargetTrackerOptions& options,
                     LidarFrame* frame) = 0;

  /**
   * @brief Get class name
   *
   * @return std::string
   */
  virtual std::string Name() const = 0;

 private:
  DISALLOW_COPY_AND_ASSIGN(BaseMultiTargetTracker);
};  // class BaseMultiTargetTracker

PERCEPTION_REGISTER_REGISTERER(BaseMultiTargetTracker);
#define PERCEPTION_REGISTER_MULTITARGET_TRACKER(name) \
  PERCEPTION_REGISTER_CLASS(BaseMultiTargetTracker, name)

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
