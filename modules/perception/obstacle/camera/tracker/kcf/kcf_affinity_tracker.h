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

 #ifndef ADU_PERCEPTION_OBSTACLE_CAMERA_KCF_AFFINITY_TRACKER_H
#define ADU_PERCEPTION_OBSTACLE_CAMERA_KCF_AFFINITY_TRACKER_H

#include <vector>
#include <limits>
#include <unordered_map>

#include "obstacle/camera/tracker/mix_camera_tracker/base_affinity_tracker.h"
#include "obstacle/camera/tracker/mix_camera_tracker/mix_camera_tracker_util.h"
#include "kcf_components.h"

namespace adu {
namespace perception {
namespace obstacle {

class KCFAffinityTracker : public BaseAffinityTracker {
public:
    KCFAffinityTracker() : BaseAffinityTracker() {}

    virtual ~KCFAffinityTracker() {}

    virtual bool init() override;

    virtual bool get_affinity_matrix(const cv::Mat &img,
                                     const std::vector<Tracked> &tracked,
                                     const std::vector<Detected> &detected,
                                     std::vector<std::vector<float > > &affinity_matrix) override;

    virtual bool update_tracked(const cv::Mat &img, const std::vector<Detected> &detected,
                                std::vector<Tracked> &tracked) override;
private:
    // z_f for all detected objects
    std::unordered_map<int, std::vector<cv::Mat>> _detected_features;

    // KCF module used
    KCFComponents _kcf_component;

    // HACK: Should save it globally somewhere
    cv::Mat _prev_img;

    float _keep_threshold = 0.0f; // 0.4f, 0.0f
};

}  // namespace obstacle
}  // namespace perception
}  // namespace adu

#endif //ADU_PERCEPTION_OBSTACLE_CAMERA_KCF_AFFINITY_TRACKER_H
