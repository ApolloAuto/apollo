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
#include "modules/perception/lidar_tracking/tracker/multi_lidar_fusion/mlf_tracker.h"

#include "cyber/common/file.h"
#include "modules/perception/common/util.h"
#include "modules/perception/lidar_tracking/tracker/multi_lidar_fusion/proto/multi_lidar_fusion_config.pb.h"

namespace apollo {
namespace perception {
namespace lidar {

bool MlfTracker::Init(const MlfTrackerInitOptions options) {
  std::string config_file = "mlf_tracker.conf";
  if (!options.config_file.empty()) {
    config_file = options.config_file;
  }
  config_file = GetConfigFile(options.config_path, config_file);
  MlfTrackerConfig config;
  ACHECK(cyber::common::GetProtoFromFile(config_file, &config));

  for (int i = 0; i < config.filter_name_size(); ++i) {
    const auto& name = config.filter_name(i);
    MlfBaseFilter* filter = MlfBaseFilterRegisterer::GetInstanceByName(name);
    ACHECK(filter);
    MlfFilterInitOptions filter_init_options;
    filter_init_options.config_path = options.config_path;
    ACHECK(filter->Init(filter_init_options));
    filters_.push_back(filter);
    AINFO << "MlfTracker add filter: " << filter->Name();
  }

  return true;
}

void MlfTracker::InitializeTrack(MlfTrackDataPtr new_track_data,
                                 TrackedObjectPtr new_object) {
  new_track_data->Reset(new_object, GetNextTrackId());
  new_track_data->is_current_state_predicted_ = false;
  new_track_data->is_front_critical_track_ =
      new_object->object_ptr->is_front_critical;
}

void MlfTracker::UpdateTrackDataWithObject(MlfTrackDataPtr track_data,
                                           TrackedObjectPtr new_object) {
  // Update trackable state:
  //   1.change background to foreground if it's matched with foreground track
  //   2.update foreground_track_prob_
  //   3.change bacground obj is_background state if it's MOVING or OBJECT
  track_data->UpdateTrackableState(new_object);
  // 1. state filter and store belief in new_object
  for (auto& filter : filters_) {
    filter->UpdateWithObject(filter_options_, track_data, new_object);
  }
  // 2. push new_obect to track_data
  track_data->PushTrackedObjectToTrack(new_object);
  track_data->is_current_state_predicted_ = false;
  track_data->is_front_critical_track_ =
      new_object->object_ptr->is_front_critical;
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
