// Copyright 2018 Baidu Inc. All Rights Reserved.
// @author: Chongchong Li (lichongchong@baidu.com)
// @file: hdmap_radar_roi_filter.h
// @brief: roi_filter for Continental ARS 408-21

#ifndef RADAR_LIB_ROI_FILTER_HDMAP_RADAR_ROI_FILTER_HDMAP_RADAR_ROI_FILTER_H_
#define RADAR_LIB_ROI_FILTER_HDMAP_RADAR_ROI_FILTER_HDMAP_RADAR_ROI_FILTER_H_

#include <string>
#include "modules/perception/radar/lib/interface/base_roi_filter.h"

namespace apollo {
namespace perception {
namespace radar {

class HdmapRadarRoiFilter : public BaseRoiFilter {
 public:
  HdmapRadarRoiFilter() : BaseRoiFilter() {}
  virtual ~HdmapRadarRoiFilter() {}

  bool Init() override { return true;}

  bool RoiFilter(
       const RoiFilterOptions& options,
       base::FramePtr radar_frame) override;

  std::string Name() const override;

 private:
  HdmapRadarRoiFilter(const HdmapRadarRoiFilter&) = delete;
  HdmapRadarRoiFilter& operator=(const HdmapRadarRoiFilter&) = delete;
};

}  // namespace radar
}  // namespace perception
}  // namespace apollo

#endif  // RADAR_LIB_ROI_FILTER_HDMAP_RADAR_ROI_FILTER_HDMAP_RADAR_ROI_FILTER_H_
