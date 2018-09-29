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
