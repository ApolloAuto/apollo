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
#include "modules/perception/common/algorithm/geometry/roi_filter.h"
#include "modules/perception/common/base/frame.h"
#include "modules/perception/common/lib/interface/base_init_options.h"
#include "modules/perception/common/lib/registerer/registerer.h"

namespace apollo {
namespace perception {
namespace radar {

struct RoiFilterInitOptions : public BaseInitOptions {};

struct RoiFilterOptions {
  base::HdmapStructPtr roi = nullptr;
};

class BaseRoiFilter {
 public:
  /**
   * @brief Construct a new Base Roi Filter object
   *
   */
  BaseRoiFilter() = default;
  virtual ~BaseRoiFilter() = default;

  /**
   * @brief Init base roi filter config
   *
   * @param options init options
   * @return true
   * @return false
   */
  virtual bool Init(const RoiFilterInitOptions& options) = 0;

  /**
   * @brief Filter the objects outside the ROI
   *
   * @param options roi filter options
   * @param radar_frame origin total objects / the objects in the ROI.
   * @return true
   * @return false
   */
  virtual bool RoiFilter(const RoiFilterOptions& options,
                         base::FramePtr radar_frame) = 0;

  /**
   * @brief The name of the radar base Roi Filter
   *
   * @return std::string
   */
  virtual std::string Name() const = 0;

 private:
  DISALLOW_COPY_AND_ASSIGN(BaseRoiFilter);
};

PERCEPTION_REGISTER_REGISTERER(BaseRoiFilter);
#define PERCEPTION_REGISTER_ROI_FILTER(name) \
  PERCEPTION_REGISTER_CLASS(BaseRoiFilter, name)

}  // namespace radar
}  // namespace perception
}  // namespace apollo
