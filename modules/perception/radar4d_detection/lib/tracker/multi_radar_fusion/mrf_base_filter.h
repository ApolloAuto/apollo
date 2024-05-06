/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#pragma once

#include <string>

#include "cyber/common/macros.h"
#include "modules/perception/common/lib/interface/base_init_options.h"
#include "modules/perception/common/lib/registerer/registerer.h"
#include "modules/perception/radar4d_detection/lib/tracker/common/mrf_track_data.h"
#include "modules/perception/radar4d_detection/lib/tracker/common/tracked_object.h"

namespace apollo {
namespace perception {
namespace radar4d {

using apollo::perception::BaseInitOptions;

struct MrfFilterInitOptions : public BaseInitOptions {};

struct MrfFilterOptions {};

class MrfBaseFilter {
 public:
  MrfBaseFilter() = default;
  virtual ~MrfBaseFilter() = default;

  /**
   * @brief Init mrf fitler
   *
   * @param options
   * @return true
   * @return false
   */
  virtual bool Init(
      const MrfFilterInitOptions& options = MrfFilterInitOptions()) = 0;

  /**
   * @brief Interface for updating filter with object
   *
   * @param options for updating
   * @param track_data track data, not include new object
   * @param new_object new object for updating
   */
  virtual void UpdateWithObject(const MrfFilterOptions& options,
                                const MrfTrackDataConstPtr& track_data,
                                TrackedObjectPtr new_object) = 0;

  /**
   * @brief Interface for updating filter without object
   *
   * @param options for updating
   * @param timestamp current timestamp
   * @param track_data track data to be updated
   */
  virtual void UpdateWithoutObject(const MrfFilterOptions& options,
                                   double timestamp,
                                   MrfTrackDataPtr track_data) = 0;
  /**
   * @brief Get class name
   *
   * @return std::string
   */
  virtual std::string Name() const = 0;

 private:
  DISALLOW_COPY_AND_ASSIGN(MrfBaseFilter);
};  // class MrfBaseFilter

PERCEPTION_REGISTER_REGISTERER(MrfBaseFilter);
#define PERCEPTION_REGISTER_MRFFILTER(name) \
  PERCEPTION_REGISTER_CLASS(MrfBaseFilter, name)

}  // namespace radar4d
}  // namespace perception
}  // namespace apollo
