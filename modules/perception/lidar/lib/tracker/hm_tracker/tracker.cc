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
#include "modules/perception/lidar/lib/tracker/hm_tracker/tracker.h"

#include "modules/perception/base/log.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/lib/io/file_util.h"
#include "modules/perception/lib/io/protobuf_util.h"
#include "modules/perception/lidar/lib/tracker/hm_tracker/proto/hm_tracker_config.pb.h"

namespace apollo {
namespace perception {
namespace lidar {

Tracker::Tracker()
    : tracker_prepared_(false),
      separate_fore_background_(false),
      current_track_id_(0),
      measure_computer_(nullptr),
      filter_(nullptr),
      post_processor_(nullptr) {}

bool Tracker::Init(const TrackerOption& option) {
  tracker_prepared_ = false;

  lib::ConfigManager* config_manager =
      lib::Singleton<lib::ConfigManager>::get_instance();
  CHECK_NOTNULL(config_manager);
  const lib::ModelConfig* model_config = nullptr;
  CHECK(config_manager->GetModelConfig("Tracker", &model_config))
      << "Failed to get model config: Tracker";

  const std::string& work_root = config_manager->work_root();
  std::string root_path;
  CHECK(model_config->get_value("root_path", &root_path))
      << "Failed to get value of root_path.";
  std::string config_file;
  config_file = lib::FileUtil::GetAbsolutePath(work_root, root_path);
  config_file = lib::FileUtil::GetAbsolutePath(config_file, "tracker.conf");
  // get config params
  TrackerConfig config_params;
  CHECK(lib::ParseProtobufFromFile<TrackerConfig>(config_file, &config_params))
      << "Failed to parse TrackerConfig config file.";

  separate_fore_background_ = config_params.separate_fore_background();
  std::string filter_name = config_params.filter_name();

  current_track_id_ = 0;

  measure_computer_.reset(new MeasurementComputer());

  filter_.reset(BaseFilterRegisterer::GetInstanceByName(filter_name));
  FilterOption filter_option;
  filter_->Init(filter_option);

  post_processor_.reset(new TrackPostProcessor());

  tracker_prepared_ = true;
  return tracker_prepared_;
}

Eigen::VectorXd Tracker::Predict(TrackDataPtr track_data, double timestamp) {
  std::pair<double, TrackedObjectPtr> latest_object_pair =
      track_data->GetLatestObject();

  TrackedObjectPtr latest_object = latest_object_pair.second;
  const Eigen::Vector3d& latest_anchor_point =
      latest_object->belief_anchor_point;
  const Eigen::Vector3d& latest_velocity = latest_object->output_velocity;

  double latest_time = latest_object_pair.first;
  double time_diff = timestamp - latest_time;
  Eigen::VectorXd predict_state;
  predict_state.resize(6);
  predict_state(0) = latest_anchor_point(0) + latest_velocity(0) * time_diff;
  predict_state(1) = latest_anchor_point(1) + latest_velocity(1) * time_diff;
  predict_state(2) = latest_anchor_point(2) + latest_velocity(2) * time_diff;
  predict_state(3) = latest_velocity(0);
  predict_state(4) = latest_velocity(1);
  predict_state(5) = latest_velocity(2);
  return predict_state;
}

void Tracker::UpdateTrackDataWithObject(TrackDataPtr track_data,
                                        TrackedObjectPtr new_object,
                                        double timestamp) {
  if (!track_data || !new_object || timestamp <= 0.0) {
    return;
  }

  // assign track id to new track_data
  if (track_data->age_ <= 0) {
    UpdateNewTrack(track_data, new_object, timestamp);
    return;
  }
  // track background if needed
  if (separate_fore_background_ && new_object->is_background) {
    UpdateBackground(track_data, new_object, timestamp);
    return;
  }
  // track foreground
  UpdateForeground(track_data, new_object, timestamp);
}

void Tracker::UpdateTrackDataWithoutObject(TrackDataPtr track_data,
                                           double timestamp) {
  TrackedObjectPtr faked_object = FakeTrackedObject(track_data, timestamp);
  UpdateTrackDataWithObject(track_data, faked_object, timestamp);
}

void Tracker::UpdateNewTrack(TrackDataPtr track_data,
                             TrackedObjectPtr new_object, double timestamp) {
  // initialize filter infomation in new object
  filter_->UpdateWithObject(track_data, new_object, timestamp);
  int track_id = GetNextTrackId();
  track_data->Reset(new_object, timestamp, track_id);
}

void Tracker::UpdateForeground(TrackDataPtr track_data,
                               TrackedObjectPtr new_object, double timestamp) {
  // update new object measurement information
  measure_computer_->ComputeMeasurment(track_data, new_object, timestamp);
  // update new object filter infomation
  filter_->UpdateWithObject(track_data, new_object, timestamp);
  // push object to track
  track_data->PushTrackedObjectToTrack(new_object, timestamp);
  // update track output information
  post_processor_->PostProcess(track_data);
}

void Tracker::UpdateBackground(TrackDataPtr track_data,
                               TrackedObjectPtr new_object, double timestamp) {
  track_data->PushTrackedObjectToTrack(new_object, timestamp);
}

TrackedObjectPtr Tracker::FakeTrackedObject(TrackDataPtr track_data,
                                            double timestamp) {
  TrackedObjectPtr faked_tracked_object = TrackedObjectPool::Instance().Get();
  base::ObjectPtr faked_object = base::ObjectPool::Instance().Get();
  faked_tracked_object->object_ptr = faked_object;

  std::pair<double, TrackedObjectPtr> latest_Tracked_object_pair =
      track_data->GetLatestObject();
  double latest_time = latest_Tracked_object_pair.first;
  TrackedObjectPtr latest_tracked_object = latest_Tracked_object_pair.second;

  double time_diff = timestamp - latest_time;
  Eigen::Vector3d predict_shift =
      latest_tracked_object->output_velocity * time_diff;

  faked_tracked_object->CopyFrom(latest_tracked_object, true);
  base::PointDCloud& cloud_world = faked_object->lidar_supplement.cloud_world;
  for (size_t i = 0; i < cloud_world.size(); ++i) {
    cloud_world[i].x += predict_shift(0);
    cloud_world[i].y += predict_shift(1);
    cloud_world[i].z += predict_shift(2);
  }

  for (size_t i = 0; i < 4; ++i) {
    faked_tracked_object->corners[i] += predict_shift;
  }
  faked_tracked_object->center += predict_shift;
  faked_tracked_object->barycenter += predict_shift;
  faked_tracked_object->anchor_point += predict_shift;
  faked_tracked_object->belief_anchor_point =
      faked_tracked_object->anchor_point;
  faked_tracked_object->output_center += predict_shift;
  faked_tracked_object->is_fake = true;
  return faked_tracked_object;
}

}  // namespace lidar
}  // namespace perception
}  // namespace apollo
