// Copyright 2018 Baidu Inc. All Rights Reserved.
// @author: Chongchong Li (lichongchong@baidu.com)
// @file: dummy_algorithms.h
// @brief: dummy algorithms

#ifndef RADAR_LIB_DUMMY_DUMMY_ALGORITHMS_H_
#define RADAR_LIB_DUMMY_DUMMY_ALGORITHMS_H_

#include <string>
#include "modules/perception/radar/lib/interface/base_preprocessor.h"
#include "modules/perception/radar/lib/interface/base_detector.h"
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
  bool Preprocess(
       const ContiRadar& raw_obstacles,
       const PreprocessorOptions& options,
       ContiRadar* corrected_obstacles) override;
  std::string Name() const override;

 private:
  DummyPreprocessor(const DummyPreprocessor&) = delete;
  DummyPreprocessor& operator=(const DummyPreprocessor&) = delete;
};

class DummyDetector : public BaseDetector {
 public:
  DummyDetector() : BaseDetector() {}
  virtual ~DummyDetector() = default;

  bool Init() override;
  bool Detect(
       const ContiRadar& corrected_obstacles,
       const DetectorOptions& options,
       base::FramePtr detected_frame) override;
  std::string Name() const override;

 private:
  void ContiObs2Frame(const ContiRadar& corrected_obstacles,
                      base::FramePtr radar_frame);
  DummyDetector(const DummyDetector&) = delete;
  DummyDetector& operator=(const DummyDetector&) = delete;
};

class DummyRoiFilter : public BaseRoiFilter {
 public:
  DummyRoiFilter() : BaseRoiFilter() {}
  virtual ~DummyRoiFilter() = default;

  bool Init() override;
  bool RoiFilter(
       const RoiFilterOptions& options,
       base::FramePtr radar_frame) override;
  std::string Name() const override;

 private:
  DummyRoiFilter(const DummyRoiFilter&) = delete;
  DummyRoiFilter& operator=(const DummyRoiFilter&) = delete;
};

}  // namespace radar
}  // namespace perception
}  // namespace apollo

#endif  // RADAR_LIB_DUMMY_DUMMY_ALGORITHMS_H_
