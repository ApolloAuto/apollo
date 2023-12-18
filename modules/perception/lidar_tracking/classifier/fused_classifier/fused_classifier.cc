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

#include "modules/perception/lidar_tracking/classifier/fused_classifier/fused_classifier.h"

#include <vector>

#include "cyber/common/file.h"
#include "modules/perception/common/util.h"
#include "modules/perception/lidar_tracking/classifier/fused_classifier/ccrf_type_fusion.h"

namespace apollo {
namespace perception {
namespace lidar {

using ObjectPtr = std::shared_ptr<apollo::perception::base::Object>;
using apollo::perception::base::ObjectType;

bool FusedClassifier::Init(const ClassifierInitOptions& options) {
  std::string config_file =
      GetConfigFile(options.config_path, options.config_file);
  FusedClassifierConfig config;
  ACHECK(cyber::common::GetProtoFromFile(config_file, &config));
  temporal_window_ = config.temporal_window();
  enable_temporal_fusion_ = config.enable_temporal_fusion();
  use_tracked_objects_ = config.use_tracked_objects();
  one_shot_fusion_method_ = config.one_shot_fusion_method();
  sequence_fusion_method_ = config.sequence_fusion_method();
  one_shot_fuser_ = BaseOneShotTypeFusionRegisterer::GetInstanceByName(
      one_shot_fusion_method_);

  bool init_success = true;
  init_option_.config_path = options.config_path;
  CHECK_NOTNULL(one_shot_fuser_);
  ACHECK(one_shot_fuser_->Init(init_option_));
  sequence_fuser_ = BaseSequenceTypeFusionRegisterer::GetInstanceByName(
      sequence_fusion_method_);
  CHECK_NOTNULL(sequence_fuser_);
  ACHECK(sequence_fuser_->Init(init_option_));
  return init_success;
}

bool FusedClassifier::Classify(const ClassifierOptions& options,
                               LidarFrame* frame) {
  if (frame == nullptr) {
    return false;
  }
  std::vector<ObjectPtr>* objects = use_tracked_objects_
                                        ? &(frame->tracked_objects)
                                        : &(frame->segmented_objects);
  if (enable_temporal_fusion_ && frame->timestamp > 0.0) {
    // sequence fusion
    AINFO << "Combined classifier, temporal fusion";
    sequence_.AddTrackedFrameObjects(*objects, frame->timestamp);
    ObjectSequence::TrackedObjects tracked_objects;
    for (auto& object : *objects) {
      if (object->lidar_supplement.is_background) {
        object->type_probs.assign(static_cast<int>(ObjectType::MAX_OBJECT_TYPE),
                                  0);
        object->type = ObjectType::UNKNOWN_UNMOVABLE;
        object->type_probs[static_cast<int>(ObjectType::UNKNOWN_UNMOVABLE)] =
            1.0;
        continue;
      }
      const int track_id = object->track_id;
      sequence_.GetTrackInTemporalWindow(track_id, &tracked_objects,
                                         temporal_window_);
      if (tracked_objects.empty()) {
        AERROR << "Find zero-length track, so skip.";
        continue;
      }
      if (object != tracked_objects.rbegin()->second) {
        AERROR << "There must exist some timestamp in disorder, so skip.";
        continue;
      }
      if (!sequence_fuser_->TypeFusion(option_, &tracked_objects)) {
        AERROR << "Failed to fuse types, so break.";
        break;
      }
    }
  } else {
    // one shot fusion
    AINFO << "Combined classifier, one shot fusion";
    for (auto& object : *objects) {
      if (object->lidar_supplement.is_background) {
        object->type_probs.assign(static_cast<int>(ObjectType::MAX_OBJECT_TYPE),
                                  0);
        object->type = ObjectType::UNKNOWN_UNMOVABLE;
        object->type_probs[static_cast<int>(ObjectType::UNKNOWN_UNMOVABLE)] =
            1.0;
        continue;
      }
      if (!one_shot_fuser_->TypeFusion(option_, object)) {
        AERROR << "Failed to fuse types, so continue.";
      }
    }
  }
  return true;
}

PERCEPTION_REGISTER_CLASSIFIER(FusedClassifier);

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
