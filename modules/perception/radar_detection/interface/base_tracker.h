/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#pragma once

#include <string>

#include "cyber/common/macros.h"
#include "modules/perception/common/base/frame.h"
#include "modules/perception/common/lib/interface/base_init_options.h"
#include "modules/perception/common/lib/registerer/registerer.h"

namespace apollo {
namespace perception {
namespace radar {

struct TrackerInitOptions : public BaseInitOptions {};

struct TrackerOptions {};

class BaseTracker {
 public:
  /**
   * @brief Construct a new Base Tracker object
   *
   */
  BaseTracker() : name_("BaseTracker") {}
  virtual ~BaseTracker() = default;

  /**
   * @brief Init base tracker config
   *
   * @param options init options
   * @return true
   * @return false
   */
  virtual bool Init(const TrackerInitOptions &options) = 0;

  /**
   * @brief Tracking objects.
   *
   * @param detected_frame current object frame.
   * @param options options.
   * @param tracked_frame current tracked objects frame.
   * @return true
   * @return false
   */
  virtual bool Track(const base::Frame &detected_frame,
                     const TrackerOptions &options,
                     base::FramePtr tracked_frame) = 0;

  /**
   * @brief The name of the radar base Tracker
   *
   * @return std::string
   */
  virtual std::string Name() { return name_; }

 protected:
  std::string name_;

 private:
  DISALLOW_COPY_AND_ASSIGN(BaseTracker);
};

PERCEPTION_REGISTER_REGISTERER(BaseTracker);
#define PERCEPTION_REGISTER_TRACKER(name) \
  PERCEPTION_REGISTER_CLASS(BaseTracker, name)
}  // namespace radar
}  // namespace perception
}  // namespace apollo
