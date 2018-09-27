// Copyright 2018 Baidu Inc. All Rights Reserved.
// @author: Chongchong Li (lichongchong@baidu.com)
// @file: base_roi_filter.h
// @brief: radar roi filter interface.

#ifndef RADAR_LIB_INTERFACE_BASE_ROI_FILTER_H_
#define RADAR_LIB_INTERFACE_BASE_ROI_FILTER_H_

// SAMPLE CODE:
//
// class DefaultRoiFilter : public BaseRoiFilter {
//  public:
//   DefaultRoiFilter() : BaseRoiFilter() {}
//   virtual ~DefaultRoiFilter() {}
//
//   virtual bool Init() override {
//     // Do something.
//     return true;
//   }
//
//   virtual bool RoiFilter(
//           const RoiFilterOptions& options,
//           base::FramePtr radar_frame) override {
//      // Do something.
//     return true;
//   }
//
//   virtual std::string Name() const override {
//        return "DefaultRoiFilter";
//   }
//
// };
//
// // Register plugin.
// PERCEPTION_REGISTER_ROIFITLER(DefaultRoiFilter);
////////////////////////////////////////////////////////
// USING CODE:
//
// BaseRoiFilter* roi_filter =
//    BaseRoiFilterRegisterer::GetInstanceByName("DefaultRoiFilter");
// using roi_filter to do somethings.
// ////////////////////////////////////////////////////

#include <string>
#include <vector>
#include "Eigen/Core"
#include "modules/perception/base/frame.h"
#include "cybertron/common/log.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/lib/registerer/registerer.h"
#include "modules/perception/common/geometry/roi_filter.h"

namespace apollo {
namespace perception {
namespace radar {

struct RoiFilterOptions {
  base::HdmapStructPtr roi = nullptr;
};

class BaseRoiFilter {
 public:
  BaseRoiFilter() = default;
  virtual ~BaseRoiFilter() = default;

  virtual bool Init() = 0;

  // @brief: fliter the objects outside the ROI
  // @param [in]: options.
  // @param [in / out]: origin total objects / the objects in the ROI.
  virtual bool RoiFilter(
          const RoiFilterOptions& options,
          base::FramePtr radar_frame) = 0;

  virtual std::string Name() const = 0;

 private:
  BaseRoiFilter(const BaseRoiFilter&) = delete;
  BaseRoiFilter& operator=(const BaseRoiFilter&) = delete;
};

PERCEPTION_REGISTER_REGISTERER(BaseRoiFilter);
#define PERCEPTION_REGISTER_ROI_FILTER(name) \
  PERCEPTION_REGISTER_CLASS(BaseRoiFilter, name)

}  // namespace radar
}  // namespace perception
}  // namespace apollo

#endif  // RADAR_LIB_INTERFACE_BASE_ROI_FILTER_H_
