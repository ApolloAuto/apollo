/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
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
#include "modules/perception/radar4d_detection/interface/base_roi_filter.h"

namespace apollo {
namespace perception {
namespace radar4d {

class HdmapRadarRoiFilter : public BaseRoiFilter {
 public:
  HdmapRadarRoiFilter() : BaseRoiFilter() {}
  virtual ~HdmapRadarRoiFilter() {}

  /**
   * @brief Init base roi filter config
   *
   * @param options init options
   * @return true
   * @return false
   */
  bool Init(const RoiFilterInitOptions& options) override { return true; }

  /**
   * @brief Filter the objects outside the ROI
   *
   * @param options roi filter options
   * @param radar_frame origin total objects / the objects in the ROI.
   * @return true
   * @return false
   */
  bool RoiFilter(const RoiFilterOptions& options,
                 RadarFrame* radar_frame) override;

  /**
   * @brief The name of the HdmapRadarRoiFilter
   *
   * @return std::string
   */
  std::string Name() const override { return "HdmapRadarRoiFilter"; }

 private:
  DISALLOW_COPY_AND_ASSIGN(HdmapRadarRoiFilter);
};

}  // namespace radar4d
}  // namespace perception
}  // namespace apollo
