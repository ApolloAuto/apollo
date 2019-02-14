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
// class DefaultDetector : public BaseDetector {
//  public:
//   DefaultDetector() : BaseDetector() {}
//   virtual ~DefaultDetector() {}
//
//   virtual bool Init() override {
//     // Do something.
//     return true;
//   }
//
//   virtual bool Detect(
//           const drivers::ContiRadar& corrected_obstacles,
//           const DetectorOptions& options,
//           base::FramePtr detected_frame) override {
//      // Do something.
//     return true;
//   }
//
//   virtual std::string Name() const override {
//        return "DefaultDetector";
//   }
//
// };
//
// // Register plugin.
// PERCEPTION_REGISTER_DETECTOR(DefaultDetector);
////////////////////////////////////////////////////////
// USING CODE:
//
// BaseDetector* detector =
//    BaseDetectorRegisterer::GetInstanceByName("DefaultDetector");
// using detector to do somethings.
// ////////////////////////////////////////////////////

#pragma once

#include <string>

#include "Eigen/Core"

#include "cyber/common/log.h"
#include "cyber/common/macros.h"

#include "modules/drivers/proto/conti_radar.pb.h"
#include "modules/perception/base/frame.h"
#include "modules/perception/common/geometry/roi_filter.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/lib/registerer/registerer.h"
#include "modules/perception/radar/common/types.h"

namespace apollo {
namespace perception {
namespace radar {

struct DetectorOptions {
  Eigen::Matrix4d* radar2world_pose = nullptr;
  Eigen::Matrix4d* radar2novatel_trans = nullptr;
  Eigen::Vector3f car_linear_speed = Eigen::Vector3f::Zero();
  Eigen::Vector3f car_angular_speed = Eigen::Vector3f::Zero();
  base::HdmapStructPtr roi = nullptr;
};

class BaseDetector {
 public:
  BaseDetector() = default;
  virtual ~BaseDetector() = default;

  virtual bool Init() = 0;

  // @brief: detect the objects from the corrected obstacles
  // @param [in]: corrected obstacles.
  // @param [in]: options.
  // @param [out]: detected objects.
  virtual bool Detect(const drivers::ContiRadar& corrected_obstacles,
                      const DetectorOptions& options,
                      base::FramePtr detected_frame) = 0;

  virtual std::string Name() const = 0;

 private:
  DISALLOW_COPY_AND_ASSIGN(BaseDetector);
};

PERCEPTION_REGISTER_REGISTERER(BaseDetector);
#define PERCEPTION_REGISTER_DETECTOR(name) \
  PERCEPTION_REGISTER_CLASS(BaseDetector, name)

}  // namespace radar
}  // namespace perception
}  // namespace apollo
