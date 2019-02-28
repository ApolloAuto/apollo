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

#include "modules/perception/lidar/lib/tracker/multi_lidar_fusion/mlf_tracker.h"

#include "cyber/common/file.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/lidar/lib/tracker/multi_lidar_fusion/proto/multi_lidar_fusion_config.pb.h"

namespace apollo {
namespace perception {
namespace lidar {

using cyber::common::GetAbsolutePath;

bool MlfTracker::Init(const MlfTrackerInitOptions options) {
  auto config_manager = lib::ConfigManager::Instance();
  const lib::ModelConfig* model_config = nullptr;
  CHECK(config_manager->GetModelConfig(Name(), &model_config));
  const std::string work_root = config_manager->work_root();
  std::string config_file;
  std::string root_path;
  CHECK(model_config->get_value("root_path", &root_path));
  config_file = GetAbsolutePath(work_root, root_path);
  config_file = GetAbsolutePath(config_file, "mlf_tracker.conf");
  MlfTrackerConfig config;
  CHECK(cyber::common::GetProtoFromFile(config_file, &config));

  for (int i = 0; i < config.filter_name_size(); ++i) {
    const auto& name = config.filter_name(i);
    MlfBaseFilter* filter = MlfBaseFilterRegisterer::GetInstanceByName(name);
    CHECK(filter);
    MlfFilterInitOptions filter_init_options;
    CHECK(filter->Init(filter_init_options));
    filters_.push_back(filter);
    AINFO << "MlfTracker add filter: " << filter->Name();
  }

  return true;
}

void MlfTracker::InitializeTrack(MlfTrackDataPtr new_track_data,
                                 TrackedObjectPtr new_object) {
  new_track_data->Reset(new_object, GetNextTrackId());
  new_track_data->is_current_state_predicted_ = false;
}

void MlfTracker::UpdateTrackDataWithObject(MlfTrackDataPtr track_data,
                                           TrackedObjectPtr new_object) {
  // 1. state filter and store belief in new_object
  for (auto& filter : filters_) {
    filter->UpdateWithObject(filter_options_, track_data, new_object);
  }
  // 2. push new_obect to track_data
  track_data->PushTrackedObjectToTrack(new_object);
  track_data->is_current_state_predicted_ = false;
}

void MlfTracker::UpdateTrackDataWithoutObject(double timestamp,
                                              MlfTrackDataPtr track_data) {
  for (auto& filter : filters_) {
    filter->UpdateWithoutObject(filter_options_, timestamp, track_data);
  }
  track_data->is_current_state_predicted_ = true;
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
