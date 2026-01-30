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

#include "modules/perception/lidar_tracking/tracker/multi_lidar_fusion/mlf_type_filter.h"

#include <vector>

#include "cyber/common/file.h"
#include "modules/perception/common/util.h"
#include "modules/perception/lidar_tracking/tracker/type_fusion/ccrf_type_fusion.h"
#include "modules/perception/lidar_tracking/tracker/multi_lidar_fusion/proto/multi_lidar_fusion_config.pb.h"

namespace apollo {
namespace perception {
namespace lidar {

using ObjectPtr = std::shared_ptr<apollo::perception::base::Object>;
using apollo::perception::base::ObjectType;

bool MlfTypeFilter::Init(const MlfFilterInitOptions& options) {
    std::string config_file = "mlf_type_filter.conf";
    if (!options.config_file.empty()) {
        config_file = options.config_file;
    }
    MlfTypeFilterConfig config;
    config_file = GetConfigFile(options.config_path, config_file);
    ACHECK(cyber::common::GetProtoFromFile(config_file, &config));

    temporal_window_ = config.temporal_window();
    enable_temporal_fusion_ = config.enable_temporal_fusion();
    use_tracked_objects_ = config.use_tracked_objects();
    print_type_filter_log_ = config.print_type_filter_log();
    one_shot_fusion_method_ = config.one_shot_fusion_method();
    sequence_fusion_method_ = config.sequence_fusion_method();
    one_shot_fuser_ = BaseSingleShotTypeFusionRegisterer::GetInstanceByName(
        one_shot_fusion_method_);

    bool init_success = true;
    init_option_.config_path = options.config_path;
    init_option_.debug_log = print_type_filter_log_;
    CHECK_NOTNULL(one_shot_fuser_);
    ACHECK(one_shot_fuser_->Init(init_option_));
    sequence_fuser_ = BaseMultiShotTypeFusionRegisterer::GetInstanceByName(
        sequence_fusion_method_);
    CHECK_NOTNULL(sequence_fuser_);
    ACHECK(sequence_fuser_->Init(init_option_));

    return init_success;
}

void MlfTypeFilter::UpdateWithObject(
        const MlfFilterOptions& options,
        const MlfTrackDataConstPtr& track_data,
        TrackedObjectPtr new_object) {
    // todo: merge fore-ground and background -> fore/back TRACK
    if (new_object->object_ptr->lidar_supplement.is_background) {
        new_object->type_probs.assign(
            static_cast<int>(ObjectType::MAX_OBJECT_TYPE), 0);
        new_object->type = ObjectType::UNKNOWN_UNMOVABLE;
        new_object->type_probs[static_cast<int>(ObjectType::UNKNOWN_UNMOVABLE)]
             = 1.0;
        return;
    }

    std::vector<TrackedObjectConstPtr> tracked_objects;
    track_data->GetObjectsInIntervalByOrder(
        new_object->timestamp - temporal_window_, &tracked_objects);
    // [ToDo] use_tracked_objects_:
    // cur: use original centerpoint model results
    // should: store latest smooth type-probs with new time to calculate final
    // std::vector<ObjectPtr>* objects = use_tracked_objects_ ?
    //   &(frame->tracked_objects) : &(frame->segmented_objects);
    if (enable_temporal_fusion_) {
        if (!sequence_fuser_->TypeFusion(option_, tracked_objects,
            new_object)) {
            AERROR << "Failed to fuse types, so break.";
            return;
        }
    } else {
        if (!one_shot_fuser_->TypeFusion(option_, new_object)) {
            AERROR << "Failed to fuse types, so continue.";
        }
    }

    // model is unknown: final is unknown -> trafficcone, otherwise -> unknown
    if (new_object->object_ptr->type == base::ObjectType::UNKNOWN) {
        if (!new_object->object_ptr->lidar_supplement.is_clustered &&
            new_object->type == base::ObjectType::UNKNOWN) {
            new_object->object_ptr->sub_type =
                base::ObjectSubType::TRAFFICCONE;
        } else {
            new_object->object_ptr->sub_type = base::ObjectSubType::UNKNOWN;
        }
    }

    if (print_type_filter_log_) {
        std::stringstream sstr;
        sstr << "[TypeFilter] track_id: " << track_data->track_id_ << " age_: "
             << track_data->age_ << " [Cur] type: "
             << static_cast<size_t>(new_object->object_ptr->type)
             << " confidence: " << new_object->object_ptr->confidence;
        for (const auto& obj : tracked_objects) {
            sstr << " [history] type: "
                 << static_cast<size_t>(obj->object_ptr->type)
                 << " confidence: " << obj->object_ptr->confidence;
        }
        sstr << " [Final]: " << new_object->type_probs[0] << ", "
             << new_object->type_probs[1] << ", " << new_object->type_probs[2]
             << ", " << new_object->type_probs[3] << ", "
             << new_object->type_probs[4] << ", " << new_object->type_probs[5]
             << " type = " << static_cast<size_t>(new_object->type);
        AINFO << sstr.str();
    }
    return;
}

void MlfTypeFilter::UpdateWithoutObject(const MlfFilterOptions& options,
    double timestamp, MlfTrackDataPtr track_data) {
    // todo: predict
}

PERCEPTION_REGISTER_MLFFILTER(MlfTypeFilter);

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
