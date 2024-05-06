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
#include "modules/perception/radar4d_detection/lib/tracker/multi_radar_fusion/mrf_tracker.h"

#include "cyber/common/file.h"
#include "modules/perception/common/util.h"
#include "modules/perception/radar4d_detection/lib/tracker/multi_radar_fusion/proto/mrf_config.pb.h"

namespace apollo {
namespace perception {
namespace radar4d {

bool MrfTracker::Init(const MrfTrackerInitOptions options) {
  std::string config_file = "mrf_tracker.pb.txt";
  if (!options.config_file.empty()) {
    config_file = options.config_file;
  }
  config_file = GetConfigFile(options.config_path, config_file);
  MrfTrackerConfig config;
  ACHECK(cyber::common::GetProtoFromFile(config_file, &config));

  for (int i = 0; i < config.filter_name_size(); ++i) {
    const auto& name = config.filter_name(i);
    MrfBaseFilter* filter = MrfBaseFilterRegisterer::GetInstanceByName(name);
    ACHECK(filter);
    MrfFilterInitOptions filter_init_options;
    filter_init_options.config_path = options.config_path;
    ACHECK(filter->Init(filter_init_options));
    filters_.push_back(filter);
    AINFO << "MrfTracker add filter: " << filter->Name();
  }

  return true;
}

void MrfTracker::InitializeTrack(MrfTrackDataPtr new_track_data,
                                 TrackedObjectPtr new_object) {
  new_track_data->Reset(new_object, GetNextTrackId());
  new_track_data->is_current_state_predicted_ = false;
}

void MrfTracker::UpdateTrackDataWithObject(MrfTrackDataPtr track_data,
                                           TrackedObjectPtr new_object) {
  // 1. state filter and store belief in new_object
  for (auto& filter : filters_) {
    filter->UpdateWithObject(filter_options_, track_data, new_object);
  }
  // 2. push new_obect to track_data
  track_data->PushTrackedObjectToTrack(new_object);
  track_data->is_current_state_predicted_ = false;
}

void MrfTracker::UpdateTrackDataWithoutObject(double timestamp,
                                              MrfTrackDataPtr track_data) {
  for (auto& filter : filters_) {
    filter->UpdateWithoutObject(filter_options_, timestamp, track_data);
  }
  track_data->is_current_state_predicted_ = true;
}

}  // namespace radar4d
}  // namespace perception
}  // namespace apollo
