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
// SAMPLE CODE:
//
// class MyTracker : public BaseTracker {
// public:
//     MyTracker() : BaseTracker() {}
//     virtual ~MyTracker() {}
//
//     virtual bool Init() override {
//         // Do something.
//         return true;
//     }
//
//     virtual bool Track(
//             const base::Frame& detected_frame,
//              const TrackerOptions& options,
//              base::FramePtr tracked_frame) override {
//          // Do something.
//          return true;
//      }
//
//      virtual std::string Name() const override {
//          return "MyTracker";
//      }
//
// };
//
// // Register plugin.
// PERCEPTION_REGISTER_TRACKER(MyTracker);
////////////////////////////////////////////////////////
// USING CODE:
//
// BaseTracker* tracker =
//          BaseTrackerRegisterer::get_instance_by_name("MyTracker");
// using tracker to do somethings.
// ////////////////////////////////////////////////////
#pragma once

#include <string>

#include "Eigen/Core"

#include "cyber/common/log.h"
#include "cyber/common/macros.h"

#include "modules/perception/base/frame.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/lib/registerer/registerer.h"

namespace apollo {
namespace perception {
namespace radar {
struct TrackerOptions {};
class BaseTracker {
 public:
  BaseTracker() : name_("BaseTracker") {}
  virtual ~BaseTracker() = default;
  virtual bool Init() = 0;
  // @brief: tracking objects.
  // @param [in]: current object frame.
  // @param [in]: options.
  // @param [out]: current tracked objects frame.
  virtual bool Track(const base::Frame &detected_frame,
                     const TrackerOptions &options,
                     base::FramePtr tracked_frame) = 0;
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
