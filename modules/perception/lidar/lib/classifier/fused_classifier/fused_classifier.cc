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
#include "modules/perception/lidar/lib/classifier/fused_classifier/fused_classifier.h"
#include <string>
#include <vector>
#include "modules/perception/lib/io/file_util.h"
#include "modules/perception/lib/io/protobuf_util.h"
#include "modules/perception/lidar/lib/classifier/fused_classifier/proto/fused_classifier_config.pb.h"

namespace apollo {
namespace perception {
namespace lidar {
using base::ObjectType;
bool FusedClassifier::Init(const ClassifierInitOptions& options) {
  lib::ConfigManager* config_manager =
      lib::Singleton<lib::ConfigManager>::get_instance();
  CHECK_NOTNULL(config_manager);
  const lib::ModelConfig* model_config = nullptr;
  CHECK(config_manager->GetModelConfig(Name(), &model_config));
  const std::string work_root = config_manager->work_root();
  std::string config_file;
  std::string root_path;
  CHECK(model_config->get_value("root_path", &root_path));
  config_file = lib::FileUtil::GetAbsolutePath(work_root, root_path);
  config_file =
      lib::FileUtil::GetAbsolutePath(config_file, "fused_classifier.conf");
  FusedClassifierConfig config;
  CHECK(lib::ParseProtobufFromFile(config_file, &config));
  temporal_window_ = config.temporal_window();
  enable_temporal_fusion_ = config.enable_temporal_fusion();
  use_tracked_objects_ = config.use_tracked_objects();
  one_shot_fusion_method_ = config.one_shot_fusion_method();
  sequence_fusion_method_ = config.sequence_fusion_method();
  one_shot_fuser_.reset(BaseOneShotTypeFusionRegisterer::GetInstanceByName(
      one_shot_fusion_method_));
  bool init_success = true;
  CHECK_NOTNULL(one_shot_fuser_.get());
  CHECK(one_shot_fuser_->Init(init_option_));
  sequence_fuser_.reset(BaseSequenceTypeFusionRegisterer::GetInstanceByName(
      sequence_fusion_method_));
  CHECK_NOTNULL(sequence_fuser_.get());
  CHECK(sequence_fuser_->Init(init_option_));
  return init_success;
}

bool FusedClassifier::Classify(const ClassifierOptions& options,
                               LidarFrame* frame) {
  if (frame == nullptr) {
    return false;
  }
  std::vector<base::ObjectPtr>* objects = use_tracked_objects_
                                              ? &(frame->tracked_objects)
                                              : &(frame->segmented_objects);
  if (enable_temporal_fusion_ && frame->timestamp > 0.0) {
    // sequence fusion
    LOG_INFO << "Combined classifier, temporal fusion";
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
      const int& track_id = object->track_id;
      sequence_.GetTrackInTemporalWindow(track_id, &tracked_objects,
                                         temporal_window_);
      if (tracked_objects.size() == 0) {
        LOG_ERROR << "Find zero-length track, so skip.";
        continue;
      }
      if (object != tracked_objects.rbegin()->second) {
        LOG_ERROR << "There must exist some timestamp in disorder, so skip.";
        continue;
      }
      if (!sequence_fuser_->TypeFusion(option_, &tracked_objects)) {
        LOG_ERROR << "Failed to fuse types, so break.";
        break;
      }
    }
  } else {
    // one shot fusion
    LOG_INFO << "Combined classifier, one shot fusion";
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
        LOG_ERROR << "Failed to fuse types, so continue.";
      }
    }
  }
  return true;
}

PERCEPTION_REGISTER_CLASSIFIER(FusedClassifier);

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
