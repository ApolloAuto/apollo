/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
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
#include "modules/perception/lib/registerer/registerer.h"
#include "modules/perception/lidar/lib/tracker/common/mlf_track_data.h"
#include "modules/perception/lidar/lib/tracker/common/tracked_object.h"

namespace apollo {
namespace perception {
namespace lidar {

struct MlfFilterInitOptions {};

struct MlfFilterOptions {};

class MlfBaseFilter {
 public:
  MlfBaseFilter() = default;
  virtual ~MlfBaseFilter() = default;

  virtual bool Init(
      const MlfFilterInitOptions& options = MlfFilterInitOptions()) = 0;

  // @brief: interface for updating filter with object
  // @params [in]: options for updating
  // @params [in]: track data, not include new object
  // @params [in/out]: new object for updating
  virtual void UpdateWithObject(const MlfFilterOptions& options,
                                const MlfTrackDataConstPtr& track_data,
                                TrackedObjectPtr new_object) = 0;

  // @brief: interface for updating filter without object
  // @params [in]: options for updating
  // @params [in]: current timestamp
  // @params [in/out]: track data to be updated
  virtual void UpdateWithoutObject(const MlfFilterOptions& options,
                                   double timestamp,
                                   MlfTrackDataPtr track_data) = 0;

  virtual std::string Name() const = 0;

 private:
  DISALLOW_COPY_AND_ASSIGN(MlfBaseFilter);
};  // class MlfBaseFilter

PERCEPTION_REGISTER_REGISTERER(MlfBaseFilter);
#define PERCEPTION_REGISTER_MLFFILTER(name) \
  PERCEPTION_REGISTER_CLASS(MlfBaseFilter, name)

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
