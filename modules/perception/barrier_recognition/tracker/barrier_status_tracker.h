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

#include <deque>
#include <map>
#include <string>
#include <vector>

#include "cyber/common/macros.h"
#include "modules/perception/common/lib/interface/base_init_options.h"

namespace apollo {
namespace perception {
namespace lidar {

struct StatusTrackerInitOptions : public BaseInitOptions {};

class BarrierStatusTracker{
public:
    BarrierStatusTracker() = default;
    virtual ~BarrierStatusTracker() = default;

    /**
     * @brief Init BarrierStatusTracker config
     *
     * @param options
     * @return true
     * @return false
     */
    bool Init(const StatusTrackerInitOptions& options = StatusTrackerInitOptions());

    /**
     * @brief Tracking status.
     *
     * @param open_percent current barrier open percentage.
     * @param status_id status id, including unkown / completely open / completely closed / opening / closing
     * @return true
     * @return false
     */
    bool Track(double timestamp, const float open_percent, int& status_id);

private:
    double first_timestamp_ = 0.;
    double s_tracking_time_win_;
    double status_change_threshold_;
    double open_threshold_;
    double close_threshold_;

    std::deque<std::pair<float, float>> prev_status_;

    DISALLOW_COPY_AND_ASSIGN(BarrierStatusTracker);
};

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
