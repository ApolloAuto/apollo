// Copyright 2018 Baidu Inc. All Rights Reserved.
// @author: Chongchong Li (lichongchong@baidu.com)
// @file: base_preprocessor.h
// @brief: radar preprocessor interface.

#ifndef RADAR_LIB_INTERFACE_BASE_PREPROCESSOR_H_
#define RADAR_LIB_INTERFACE_BASE_PREPROCESSOR_H_
// SAMPLE CODE:
//
// class DefaultPreprocessor : public BasePreprocessor {
//  public:
//   DefaultPreprocessor() : BasePreprocessor() {}
//   virtual ~DefaultPreprocessor() {}
//
//   virtual bool Init() override {
//     // Do something.
//     return true;
//   }
//
//   virtual bool Preprocess(
//           const ContiRadar& raw_obstacles,
//           const PreprocessorOptions& options,
//           ContiRadar* corrected_obstacles) override {
//      // Do something.
//      return true;
//    }
//
//    virtual std::string Name() const override {
//        return "DefaultPreprocessor";
//    }
//
// };
//
// // Register plugin.
// PERCEPTION_REGISTER_PREPROCESSOR(DefaultPreprocessor);
////////////////////////////////////////////////////////
// USING CODE:
//
// BasePreprocessor* preprocessor =
//    BasePreprocessorRegisterer::GetInstanceByName("DefaultPreprocessor");
// using preprocessor to do somethings.
// ////////////////////////////////////////////////////

#include <string>
#include <vector>
#include "Eigen/Core"
#include "modules/drivers/proto/conti_radar.pb.h"
#include "modules/perception/lib/registerer/registerer.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/lib/singleton/singleton.h"
#include "modules/perception/base/frame.h"
#include "cybertron/common/log.h"
#include "modules/perception/radar/common/types.h"

namespace apollo {
namespace perception {
namespace radar {

using apollo::common::Header;
using apollo::drivers::ContiRadarObs;
using apollo::drivers::ContiRadar;

struct PreprocessorOptions {
  // reserved
};

class BasePreprocessor {
 public:
  BasePreprocessor() = default;
  virtual ~BasePreprocessor() = default;

  virtual bool Init() = 0;

  // @brief: correct radar raw obstacles.
  // @param [in]: raw obstacles from radar driver.
  // @param [in]: options.
  // @param [out]: corrected radar obstacles
  virtual bool Preprocess(
          const ContiRadar& raw_obstacles,
          const PreprocessorOptions& options,
          ContiRadar* corrected_obstacles) = 0;

  virtual std::string Name() const = 0;

 private:
  BasePreprocessor(const BasePreprocessor&) = delete;
  BasePreprocessor& operator=(const BasePreprocessor&) = delete;
};

PERCEPTION_REGISTER_REGISTERER(BasePreprocessor);
#define PERCEPTION_REGISTER_PREPROCESSOR(name) \
  PERCEPTION_REGISTER_CLASS(BasePreprocessor, name)

}  // namespace radar
}  // namespace perception
}  // namespace apollo

#endif  // RADAR_LIB_INTERFACE_BASE_PREPROCESSOR_H_
