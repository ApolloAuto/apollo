// Copyright 2018 Baidu Inc. All Rights Reserved.
// @author: Chongchong Li (lichongchong@baidu.com)
// @file: hdmap_radar_roi_filter.h
// @brief: roi_filter for Continental ARS408-21.


#include <vector>
#include "modules/perception/radar/lib/roi_filter/hdmap_radar_roi_filter/hdmap_radar_roi_filter.h"

namespace apollo {
namespace perception {
namespace radar {

bool HdmapRadarRoiFilter::RoiFilter(
          const RoiFilterOptions& options,
          base::FramePtr radar_frame) {
  std::vector<base::ObjectPtr> origin_objects = radar_frame->objects;
  common::ObjectInRoiCheck(options.roi,
                           origin_objects,
                           &radar_frame->objects);
  return true;
}

std::string HdmapRadarRoiFilter::Name() const {
  return "HdmapRadarRoiFilter";
}

PERCEPTION_REGISTER_ROI_FILTER(HdmapRadarRoiFilter);

}  // namespace radar
}  // namespace perception
}  // namespace apollo

