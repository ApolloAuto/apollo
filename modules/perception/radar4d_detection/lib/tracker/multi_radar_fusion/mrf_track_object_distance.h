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

#include <map>
#include <string>
#include <vector>

#include "modules/perception/common/lib/interface/base_init_options.h"
#include "modules/perception/common/util.h"
#include "modules/perception/radar4d_detection/lib/tracker/common/mrf_track_data.h"
#include "modules/perception/radar4d_detection/lib/tracker/common/tracked_object.h"

namespace apollo {
namespace perception {
namespace radar4d {

using apollo::perception::BaseInitOptions;

struct MrfTrackObjectDistanceInitOptions : public BaseInitOptions {};

class MrfTrackObjectDistance {
 public:
  MrfTrackObjectDistance() = default;
  ~MrfTrackObjectDistance() = default;

  /**
   * @brief Init mrf track object distance
   *
   * @param options
   * @return true
   * @return false
   */
  bool Init(const MrfTrackObjectDistanceInitOptions& options =
                MrfTrackObjectDistanceInitOptions());

  /**
   * @brief Compute object track distance
   *
   * @param object
   * @param track track data
   * @return float distance
   */
  float ComputeDistance(const TrackedObjectConstPtr& object,
                        const MrfTrackDataConstPtr& track) const;

  /**
   * @brief Get class name
   *
   * @return std::string
   */
  std::string Name() const { return "MrfTrackObjectDistance"; }

 protected:
  std::map<std::string, std::vector<float>> foreground_weight_table_;
  std::map<std::string, std::vector<float>> background_weight_table_;

  static const std::vector<float> kForegroundDefaultWeight;
  static const std::vector<float> kBackgroundDefaultWeight;

  double background_object_match_threshold_ = 4.0;
};  // class MrfTrackObjectDistance

}  // namespace radar4d
}  // namespace perception
}  // namespace apollo
