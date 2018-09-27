// Copyright 2018 Baidu Inc. All Rights Reserved.
// @author: Chongchong Li (lichongchong@baidu.com)
// @file: conti_ars_detector.h
// @brief: detector for Continental Ars 408-21

#ifndef RADAR_LIB_DETECTOR_CONTI_ARS_DETECTOR_CONTI_ARS_DETECTOR_H_
#define RADAR_LIB_DETECTOR_CONTI_ARS_DETECTOR_CONTI_ARS_DETECTOR_H_

#include <string>
#include "modules/perception/radar/lib/interface/base_detector.h"
#include "modules/perception/radar/common/radar_util.h"

namespace apollo {
namespace perception {
namespace radar {

class ContiArsDetector : public BaseDetector {
 public:
  ContiArsDetector() : BaseDetector() {}
  virtual ~ContiArsDetector() {}

  bool Init() override;

  bool Detect(
       const ContiRadar& corrected_obstacles,
       const DetectorOptions& options,
       base::FramePtr detected_frame) override;

  std::string Name() const override;

 private:
  void RawObs2Frame(
       const ContiRadar& corrected_obstacles,
       const DetectorOptions& options,
       base::FramePtr radar_frame);
  ContiArsDetector(const ContiArsDetector&) = delete;
  ContiArsDetector& operator=(const ContiArsDetector&) = delete;
};

}  // namespace radar
}  // namespace perception
}  // namespace apollo

#endif  // RADAR_LIB_DETECTOR_CONTI_ARS_DETECTOR_CONTI_ARS_DETECTOR_H_
