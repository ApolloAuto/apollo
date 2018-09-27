// Copyright 2018 Baidu Inc. All Rights Reserved.
// @author: Chongchong Li (lichongchong@baidu.com)
// @file: conti_ars_preprocessor.h
// @brief: preprocessor for Continental ARS 408-21

#ifndef RADAR_LIB_PREPROCESSOR_CONTI_ARS_PREPROCESSOR_CONTI_ARS_PREPROCESSOR_H_
#define RADAR_LIB_PREPROCESSOR_CONTI_ARS_PREPROCESSOR_CONTI_ARS_PREPROCESSOR_H_

#include <string>
#include <map>
#include "modules/perception/radar/lib/interface/base_preprocessor.h"

namespace apollo {
namespace perception {
namespace radar {

class ContiArsPreprocessor : public BasePreprocessor {
 public:
  ContiArsPreprocessor() : BasePreprocessor(),
                           delay_time_(0.0) {}
  virtual ~ContiArsPreprocessor() {}

  bool Init() override;

  bool Preprocess(
       const ContiRadar& raw_obstacles,
       const PreprocessorOptions& options,
       ContiRadar* corrected_obstacles) override;

  std::string Name() const override;

  inline double GetDelayTime() {
    return delay_time_;
  }

 private:
  void SkipObjects(const ContiRadar& raw_obstacles,
                   ContiRadar* corrected_obstacles);
  void ExpandIds(ContiRadar* corrected_obstacles);
  void CorrectTime(ContiRadar* corrected_obstacles);
  int GetNextId();
  float delay_time_;
  static int current_idx_;
  static int local2global_[ORIGIN_CONTI_MAX_ID_NUM];
  ContiArsPreprocessor(const ContiArsPreprocessor&) = delete;
  ContiArsPreprocessor& operator=(const ContiArsPreprocessor&) = delete;

  friend class ContiArsPreprocessorTest;
};

}  // namespace radar
}  // namespace perception
}  // namespace apollo

#endif  // RADAR_LIB_PREPROCESSOR_CONTI_ARS_PREPROCESSOR_CONTI_ARS_PREPROCESSOR_H_
