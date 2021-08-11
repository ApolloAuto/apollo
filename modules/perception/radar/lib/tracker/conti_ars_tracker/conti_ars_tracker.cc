/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the License);
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "modules/perception/radar/lib/tracker/conti_ars_tracker/conti_ars_tracker.h"

namespace apollo {
namespace perception {
namespace radar {

double ContiArsTracker::s_tracking_time_win_ = 0.06;
ContiArsTracker::ContiArsTracker()
    : BaseTracker(), matcher_(nullptr), track_manager_(nullptr) {
  name_ = "ContiArsTracker";
}

ContiArsTracker::~ContiArsTracker() {
  if (matcher_ != nullptr) {
    delete matcher_;
  }
  if (track_manager_ != nullptr) {
    delete track_manager_;
  }
}

bool ContiArsTracker::Init() {
  std::string model_name = name_;
  const lib::ModelConfig *model_config = nullptr;
  bool state = true;
  if (!lib::ConfigManager::Instance()->GetModelConfig(model_name,
                                                      &model_config)) {
    AERROR << "not found model: " << model_name;
    state = false;
  }
  if (!model_config->get_value("tracking_time_window", &s_tracking_time_win_)) {
    AERROR << "track_time_window is not found.";
    state = false;
  }
  if (!model_config->get_value("macher_name", &matcher_name_)) {
    AERROR << "macher_name is not found.";
    state = false;
  }
  std::string chosen_filter;
  if (!model_config->get_value("chosen_filter", &chosen_filter)) {
    AERROR << "chosen_filter is not found.";
    state = false;
  }
  RadarTrack::SetChosenFilter(chosen_filter);
  int tracked_times_threshold;
  if (!model_config->get_value("tracked_times_threshold",
                               &tracked_times_threshold)) {
    AERROR << "tracked_times_threshold is not found.";
    state = false;
  }
  RadarTrack::SetTrackedTimesThreshold(tracked_times_threshold);
  bool use_filter;
  if (!model_config->get_value("use_filter", &use_filter)) {
    AERROR << "use_filter is not found.";
    state = false;
  }
  RadarTrack::SetUseFilter(use_filter);
  // Or use register class instead.
  if (matcher_name_ == "HMMatcher") {
    matcher_ = new HMMatcher();
    matcher_->Init();  //  use proto later
  } else {
    AERROR << "Not supported matcher : " << matcher_name_;
    state = false;
  }

  track_manager_ = new RadarTrackManager();
  ACHECK(track_manager_ != nullptr)
      << "Failed to get RadarTrackManager instance.";
  return state;
}

bool ContiArsTracker::Track(const base::Frame &detected_frame,
                            const TrackerOptions &options,
                            base::FramePtr tracked_frame) {
  TrackObjects(detected_frame);
  CollectTrackedFrame(tracked_frame);
  return true;
}

void ContiArsTracker::TrackObjects(const base::Frame &radar_frame) {
  std::vector<TrackObjectPair> assignments;
  std::vector<size_t> unassigned_tracks;
  std::vector<size_t> unassigned_objects;
  TrackObjectMatcherOptions matcher_options;
  const auto &radar_tracks = track_manager_->GetTracks();
  matcher_->Match(radar_tracks, radar_frame, matcher_options, &assignments,
                  &unassigned_tracks, &unassigned_objects);
  UpdateAssignedTracks(radar_frame, assignments);
  UpdateUnassignedTracks(radar_frame, unassigned_tracks);
  DeleteLostTracks();
  CreateNewTracks(radar_frame, unassigned_objects);
}

void ContiArsTracker::UpdateAssignedTracks(
    const base::Frame &radar_frame, std::vector<TrackObjectPair> assignments) {
  auto &radar_tracks = track_manager_->mutable_tracks();
  for (size_t i = 0; i < assignments.size(); ++i) {
    radar_tracks[assignments[i].first]->UpdataObsRadar(
        radar_frame.objects[assignments[i].second], radar_frame.timestamp);
  }
}

void ContiArsTracker::UpdateUnassignedTracks(
    const base::Frame &radar_frame,
    const std::vector<size_t> &unassigned_tracks) {
  double timestamp = radar_frame.timestamp;
  auto &radar_tracks = track_manager_->mutable_tracks();
  for (size_t i = 0; i < unassigned_tracks.size(); ++i) {
    if (radar_tracks[unassigned_tracks[i]]->GetObs() != nullptr) {
      double radar_time = radar_tracks[unassigned_tracks[i]]->GetTimestamp();
      double time_diff = fabs(timestamp - radar_time);
      if (time_diff > s_tracking_time_win_) {
        radar_tracks[unassigned_tracks[i]]->SetDead();
      }
    } else {
      radar_tracks[unassigned_tracks[i]]->SetDead();
    }
  }
}

void ContiArsTracker::DeleteLostTracks() { track_manager_->RemoveLostTracks(); }

void ContiArsTracker::CreateNewTracks(
    const base::Frame &radar_frame,
    const std::vector<size_t> &unassigned_objects) {
  for (size_t i = 0; i < unassigned_objects.size(); ++i) {
    RadarTrackPtr radar_track;
    radar_track.reset(new RadarTrack(radar_frame.objects[unassigned_objects[i]],
                                     radar_frame.timestamp));
    track_manager_->AddTrack(radar_track);
  }
}

void ContiArsTracker::CollectTrackedFrame(base::FramePtr tracked_frame) {
  if (tracked_frame == nullptr) {
    AERROR << "tracked_frame is nullptr";
    return;
  }
  auto &objects = tracked_frame->objects;
  const auto &radar_tracks = track_manager_->GetTracks();
  for (size_t i = 0; i < radar_tracks.size(); ++i) {
    if (radar_tracks[i]->ConfirmTrack()) {
      base::ObjectPtr object = base::ObjectPtr(new base::Object());
      const base::ObjectPtr &track_object = radar_tracks[i]->GetObs();
      *object = *track_object;
      object->tracking_time = radar_tracks[i]->GetTrackingTime();
      object->track_id = radar_tracks[i]->GetObsId();
      object->latest_tracked_time = radar_tracks[i]->GetTimestamp();
      objects.push_back(object);
    }
  }
}

PERCEPTION_REGISTER_TRACKER(ContiArsTracker);

}  // namespace radar
}  // namespace perception
}  // namespace apollo
