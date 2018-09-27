// Copyright 2018 Baidu Inc. All Rights Reserved.
// @author: Chongchong Li (lichongchong@baidu.com)
// @file: base_detector.h
// @brief: radar detector interface.

#ifndef RADAR_LIB_INTERFACE_BASE_DETECTOR_H_
#define RADAR_LIB_INTERFACE_BASE_DETECTOR_H_

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
//           const ContiRadar& corrected_obstacles,
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

#include <string>
#include <vector>
#include "Eigen/Core"
#include "modules/drivers/proto/conti_radar.pb.h"
#include "modules/perception/base/frame.h"
#include "cybertron/common/log.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/lib/registerer/registerer.h"
#include "modules/perception/common/geometry/roi_filter.h"
#include "modules/perception/radar/common/types.h"

namespace apollo {
namespace perception {
namespace radar {

using apollo::common::Header;
using apollo::drivers::ContiRadarObs;
using apollo::drivers::ContiRadar;

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
  virtual bool Detect(
          const ContiRadar& corrected_obstacles,
          const DetectorOptions& options,
          base::FramePtr detected_frame) = 0;

  virtual std::string Name() const = 0;

 private:
  BaseDetector(const BaseDetector&) = delete;
  BaseDetector& operator=(const BaseDetector&) = delete;
};

PERCEPTION_REGISTER_REGISTERER(BaseDetector);
#define PERCEPTION_REGISTER_DETECTOR(name) \
  PERCEPTION_REGISTER_CLASS(BaseDetector, name)

}  // namespace radar
}  // namespace perception
}  // namespace apollo

#endif  // RADAR_LIB_INTERFACE_BASE_DETECTOR_H_
