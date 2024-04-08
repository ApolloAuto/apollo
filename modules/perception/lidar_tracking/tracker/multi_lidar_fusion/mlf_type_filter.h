/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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

#include <memory>
#include <string>

#include "gtest/gtest_prod.h"

#include "modules/perception/common/lidar/common/lidar_frame.h"
#include "modules/perception/common/lidar/common/object_sequence.h"
#include "modules/perception/lidar_tracking/tracker/type_fusion/type_fusion_interface.h"
#include "modules/perception/lidar_tracking/tracker/multi_lidar_fusion/mlf_base_filter.h"

namespace apollo {
namespace perception {
namespace lidar {

class MlfTypeFilter : public MlfBaseFilter {
 public:
    MlfTypeFilter() = default;
    ~MlfTypeFilter() = default;

    bool Init(const MlfFilterInitOptions& options =
        MlfFilterInitOptions()) override;

    void UpdateWithObject(
            const MlfFilterOptions& options,
            const MlfTrackDataConstPtr& track_data,
            TrackedObjectPtr new_object) override;

    void UpdateWithoutObject(const MlfFilterOptions& options, double timestamp,
        MlfTrackDataPtr track_data) override;

    std::string Name() const override {
        return "MlfTypeFilter";
    }

 private:
    ObjectSequence sequence_;
    double temporal_window_ = 20.0;
    bool enable_temporal_fusion_ = true;
    bool use_tracked_objects_ = true;
    bool print_type_filter_log_ = false;

    std::string one_shot_fusion_method_;
    std::string sequence_fusion_method_;

    BaseSingleShotTypeFusion* one_shot_fuser_;
    BaseMultiShotTypeFusion* sequence_fuser_;

    std::unique_ptr<BaseSingleShotTypeFusion> one_shot_fuser_ptr_;
    std::unique_ptr<BaseMultiShotTypeFusion> sequence_fuser_ptr_;

    TypeFilterOption option_;
    TypeFilterInitOption init_option_;

    MlfFilterInitOptions fused_classifier_config_;
};  // class MlfTypeFilter

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
