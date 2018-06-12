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

// @brief: base class of lane post-processor
//
// SAMPLE CODE:
//
// class DefaultLanePostProcessor : public BaseLanePostProcessor {
// public:
//     DefaultLanePostProcessor() : BaseLanePostProcessor() {}
//     virtual ~DefaultLanePostProcessor() {}
//
//     virtual bool Init() override {
//         // Do something.
//         return true;
//     }
//
//      virtual std::string name() const override {
//          return "DefaultLanePostProcessor";
//      }
//
// };
//
// // Register plugin.
// REGISTER_CAMERA_POST_PROCESSOR(DefaultLanePostProcessor);
////////////////////////////////////////////////////////
// USING CODE:
//
// BaseCameraLanePostProcessor* camera_lane_post_processor =
//    BaseCameraLanePostProcessorRegistar::get_instance_by_name("DefaultCameraLanePostProcessor");
// using camera_detector to do somethings.
// ////////////////////////////////////////////////////

#ifndef MODULES_PERCEPTION_OBSTACLE_CAMERA_INTERFACE_BASE_LANE_POST_PROC_H_
#define MODULES_PERCEPTION_OBSTACLE_CAMERA_INTERFACE_BASE_LANE_POST_PROC_H_

#include <string>
#include <vector>

#include "Eigen/Core"
#include "opencv2/opencv.hpp"

#include "modules/common/macro.h"
#include "modules/perception/lib/base/registerer.h"
#include "modules/perception/obstacle/camera/lane_post_process/common/type.h"
#include "modules/perception/obstacle/base/object_supplement.h"

namespace apollo {
namespace perception {

struct CameraLanePostProcessOptions {
  double timestamp;
  bool use_lane_history = false;
  int lane_history_size = 0;
  VehicleStatus vehicle_status;
  void SetMotion(const VehicleStatus &vs) {
    vehicle_status = vs;
  }
};

class BaseCameraLanePostProcessor {
 public:
  BaseCameraLanePostProcessor() = default;
  virtual ~BaseCameraLanePostProcessor() = default;

  virtual bool Init() = 0;

  /*
  // @brief: lane pixel label map -> lane objects
  // @param [in]: lane pixel label map
  // @param [in]: options
  // @param [out]: lane objects
  virtual bool Process(const cv::Mat& lane_map,
                       const CameraLanePostProcessOptions& options,
                       LaneObjectsPtr lane_instances) = 0;
  */
  virtual bool Process(const cv::Mat& lane_map,
                       const CameraLanePostProcessOptions& options,
                       LaneObjectsPtr* lane_instances) = 0;

  virtual std::string name() const = 0;

 private:
  DISALLOW_COPY_AND_ASSIGN(BaseCameraLanePostProcessor);
};

REGISTER_REGISTERER(BaseCameraLanePostProcessor);
#define REGISTER_CAMERA_LANE_POST_PROCESSOR(name) \
  REGISTER_CLASS(BaseCameraLanePostProcessor, name)

}  // namespace perception
}  // namespace apollo

#endif  // MODULES_PERCEPTION_OBSTACLE_CAMERA_INTERFACE_BASE_LANE_POST_PROC_H_
