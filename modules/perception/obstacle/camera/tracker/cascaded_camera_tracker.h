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

#ifndef ADU_PERCEPTION_OBSTACLE_MIX_CAMERA_TRACKER_H
#define ADU_PERCEPTION_OBSTACLE_MIX_CAMERA_TRACKER_H

#include <map>
#include <vector>
#include <algorithm>

#include "obstacle/camera/interface/base_camera_tracker.h"
#include "obstacle/camera/tracker/common/kalman_filter.h"

#include "obstacle/camera/tracker/mix_camera_tracker/mix_camera_tracker_util.h"
#include "obstacle/camera/tracker/mix_camera_tracker/base_affinity_tracker.h"
#include "obstacle/camera/tracker/mix_camera_tracker/cs2d/cs2d_affinity_tracker.h"
#include "obstacle/camera/tracker/mix_camera_tracker/dlf/dlf_affinity_tracker.h"
#include "obstacle/camera/tracker/mix_camera_tracker/dat/dat_affinity_tracker.h"
#include "obstacle/camera/tracker/mix_camera_tracker/kcf/kcf_affinity_tracker.h"

namespace adu {
namespace perception {
namespace obstacle {

class CascadedCameraTracker : public BaseCameraTracker {
public:

    CascadedCameraTracker() : BaseCameraTracker() {}

    virtual ~CascadedCameraTracker() {}

    virtual bool init() override;

    virtual bool associate(const cv::Mat &frame,
                           const std::vector<VisualObjectPtr> &objects,
                           double timestamp,
                           const CameraTrackerOptions &options,
                           std::vector<VisualObjectPtr> *tracked_objects) override;

    virtual std::string name() const override;

private:
    // Verbose to XLOG(INFO)
    bool _verbose = false;
    bool _with_dl_feature = true;

    // Trackers from different stages
    CS2DAffinityTracker _cs2d_tracker;
    DLFAffinityTracker _dlf_tracker;
    KCFAffinityTracker _kcf_tracker;

    // Tracking and ID management
    std::vector<Tracked> _tracked;  // All tracked objects, extended up to max frame latent space
    int _next_tracked_id = 0;
    int _frame_cnt = 0;
    int _max_kep_frame_cnt = 10;

    DISALLOW_COPY_AND_ASSIGN(MixCameraTracker);
};

}  // namespace obstacle
}  // namespace perception
}  // namespace adu

#endif //ADU_PERCEPTION_OBSTACLE_MIX_CAMERA_TRACKER_H
