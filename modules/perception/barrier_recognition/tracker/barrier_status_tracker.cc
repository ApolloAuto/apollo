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
#include <cstdlib>

#include "cyber/common/file.h"
#include "cyber/common/log.h"

#include "modules/perception/barrier_recognition/tracker/barrier_status_tracker.h"
#include "modules/perception/barrier_recognition/tracker/proto/barrier_tracker_config.pb.h"
#include "modules/perception/common/algorithm/i_lib/geometry/i_line.h"
#include "modules/perception/common/util.h"


namespace apollo {
namespace perception {
namespace lidar {

bool BarrierStatusTracker::Init(const StatusTrackerInitOptions &options) {
    std::string config_file = GetConfigFile(options.config_path, options.config_file);

    BarrierTrackerConfig config;
    ACHECK(cyber::common::GetProtoFromFile(config_file, &config));

    s_tracking_time_win_ = config.tracking_time_window();
    status_change_threshold_ = config.status_change_threshold();
    open_threshold_ = config.open_threshold();
    close_threshold_ = config.close_threshold();
    return true;
}

bool BarrierStatusTracker::Track(double timestamp, const float open_percent, int& status_id) {
    if (first_timestamp_ < 1e-3) {
      first_timestamp_ = timestamp;
    }
    double relative_timestamp = timestamp - first_timestamp_;
    while (!prev_status_.empty() &&
        relative_timestamp - prev_status_.front().first > s_tracking_time_win_) {
        prev_status_.pop_front();
    }
    prev_status_.push_back(std::make_pair(relative_timestamp, open_percent));
    float theta = 0.;
    // check cached status
    if (prev_status_.size() == 1) {
        AINFO << "No cached open_percent status, return unkown.";
        status_id = 0;
    } else if (open_percent < close_threshold_) {
        status_id = 1;
    } else {
        std::vector<float> status_samples(static_cast<int>(prev_status_.size() * 2), 0);
        for (size_t i = 0; i < prev_status_.size(); i++) {
            auto& status = prev_status_.at(i);
            status_samples[i*2] = status.first;
            status_samples[i*2+1] = status.second;
        }

        float fit_line[3] = {0};
        algorithm::ILineFit2dTotalLeastSquare(status_samples.data(), 
                                              fit_line, 
                                              prev_status_.size());
        theta = std::atan(-fit_line[0] / (fit_line[1] + 1e-6)); //  -PI/2 ~ PI/2

        if (std::abs(theta) < status_change_threshold_) {
            if (open_percent > open_threshold_) {
                status_id = 3;
            } else if (open_percent < close_threshold_) {
               status_id = 1;
            } else {
                status_id = 0;
            }
        } else if (theta > 0.) {
            status_id = 4;
        } else {
           status_id = 2;
        }
    }

    return true;
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
