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
#include "modules/perception/radar/lib/interface/base_detector.h"
#include "modules/perception/radar/lib/interface/base_preprocessor.h"
#include "modules/perception/radar/lib/interface/base_roi_filter.h"
// #include "modules/perception/radar/lib/interface/base_matcher.h"
// #include "modules/perception/radar/lib/interface/base_filter.h"
// #include "modules/perception/radar/lib/interface/base_tracker.h"

namespace apollo {
namespace perception {
namespace radar {

class DummyPreprocessor : public BasePreprocessor {
 public:
  DummyPreprocessor() : BasePreprocessor() {}
  virtual ~DummyPreprocessor() = default;

  bool Init() override;
  bool Preprocess(const drivers::ContiRadar& raw_obstacles,
                  const PreprocessorOptions& options,
                  drivers::ContiRadar* corrected_obstacles) override;
  std::string Name() const override;

 private:
  DISALLOW_COPY_AND_ASSIGN(DummyPreprocessor);
};

class DummyDetector : public BaseDetector {
 public:
  DummyDetector() : BaseDetector() {}
  virtual ~DummyDetector() = default;

  bool Init() override;
  bool Detect(const drivers::ContiRadar& corrected_obstacles,
              const DetectorOptions& options,
              base::FramePtr detected_frame) override;
  std::string Name() const override;

 private:
  void ContiObs2Frame(const drivers::ContiRadar& corrected_obstacles,
                      base::FramePtr radar_frame);

  DISALLOW_COPY_AND_ASSIGN(DummyDetector);
};

class DummyRoiFilter : public BaseRoiFilter {
 public:
  DummyRoiFilter() : BaseRoiFilter() {}
  virtual ~DummyRoiFilter() = default;

  bool Init() override;
  bool RoiFilter(const RoiFilterOptions& options,
                 base::FramePtr radar_frame) override;
  std::string Name() const override;

 private:
  DISALLOW_COPY_AND_ASSIGN(DummyRoiFilter);
};

}  // namespace radar
}  // namespace perception
}  // namespace apollo
