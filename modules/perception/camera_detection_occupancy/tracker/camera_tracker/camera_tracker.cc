/******************************************************************************
 * Copyright 2024 The Apollo Authors. All Rights Reserved.
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
#include "modules/perception/camera_detection_occupancy/tracker/camera_tracker/camera_tracker.h"

#include "modules/perception/camera_detection_occupancy/proto/tracker_config.pb.h"

#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "modules/perception/common/util.h"

namespace apollo {
namespace perception {
namespace camera {

double CameraTracker::s_tracking_time_win_ = 0.06;

CameraTracker::CameraTracker()
    : BaseTracker(), matcher_(nullptr), track_manager_(nullptr) {
  name_ = "CameraTracker";
}

CameraTracker::~CameraTracker() {
  if (track_manager_ != nullptr) {
    delete track_manager_;
  }
}

bool CameraTracker::Init(const TrackerInitOptions &options) {
  std::string config_file =
      GetConfigFile(options.config_path, options.config_file);

  TrackerConfig config;
  ACHECK(cyber::common::GetProtoFromFile(config_file, &config));

  s_tracking_time_win_ = config.tracking_time_window();

  std::string chosen_filter = config.chosen_filter();
  CameraTrack::SetChosenFilter(chosen_filter);

  int tracked_times_threshold = config.tracked_times_threshold();
  CameraTrack::SetTrackedTimesThreshold(tracked_times_threshold);

  bool use_filter = config.use_filter();
  CameraTrack::SetUseFilter(use_filter);

  // Or use register class instead.
  MatcherInitOptions matcher_init_options;
  auto matcher_param = config.matcher_param();
  matcher_init_options.config_path = matcher_param.config_path();
  matcher_init_options.config_file = matcher_param.config_file();
  BaseMatcher *matcher =
      BaseMatcherRegisterer::GetInstanceByName(matcher_param.name());
  CHECK_NOTNULL(matcher);
  matcher_.reset(matcher);
  ACHECK(matcher_->Init(matcher_init_options))
      << "Failed to init Camera preprocessor.";

  track_manager_ = new CameraTrackManager();
  ACHECK(track_manager_ != nullptr)
      << "Failed to get CameraTrackManager instance.";
  return true;
}

bool CameraTracker::Track(const base::Frame &detected_frame,
                          base::FramePtr tracked_frame) {
  TrackObjects(detected_frame);
  CollectTrackedFrame(tracked_frame);
  return true;
}

void CameraTracker::TrackObjects(const base::Frame &camera_frame) {
  std::vector<TrackObjectPair> assignments;
  std::vector<size_t> unassigned_tracks;
  std::vector<size_t> unassigned_objects;
  TrackObjectMatcherOptions matcher_options;
  const auto &camera_tracks = track_manager_->GetTracks();
  matcher_->Match(camera_tracks, camera_frame, matcher_options, &assignments,
                  &unassigned_tracks, &unassigned_objects);
  UpdateAssignedTracks(camera_frame, assignments);
  UpdateUnassignedTracks(camera_frame, unassigned_tracks);
  DeleteLostTracks();
  CreateNewTracks(camera_frame, unassigned_objects);
}

void CameraTracker::UpdateAssignedTracks(
    const base::Frame &camera_frame, std::vector<TrackObjectPair> assignments) {
  auto &camera_tracks = track_manager_->mutable_tracks();
  for (size_t i = 0; i < assignments.size(); ++i) {
    double time_diff = camera_frame.timestamp -
      camera_tracks[assignments[i].first]->GetTimestamp();
    MeasurementBboxCenterVelocity(
        camera_frame.objects[assignments[i].second],
        camera_tracks[assignments[i].first]->GetObs(), time_diff);
    MeasureVelocityRefine(
        camera_frame.objects[assignments[i].second],
        camera_tracks[assignments[i].first]->GetObs(), time_diff);
    camera_tracks[assignments[i].first]->UpdataObsCamera(
        camera_frame.objects[assignments[i].second], camera_frame.timestamp);
  }
}

void CameraTracker::MeasurementBboxCenterVelocity(
        base::ObjectPtr new_object,
        const base::ObjectPtr old_object,
        double time_diff) {
    Eigen::Vector3d old_center = old_object->center;
    Eigen::Vector3d new_center = new_object->center;
    Eigen::Vector3d measured_bbox_center_velocity = (new_center - old_center);
    measured_bbox_center_velocity /= time_diff;
    measured_bbox_center_velocity(2) = 0.0;
    new_object->velocity[0] = measured_bbox_center_velocity[0];
    new_object->velocity[1] = measured_bbox_center_velocity[1];
    new_object->velocity[2] = measured_bbox_center_velocity[2];
}

void CameraTracker::MeasureVelocityRefine(
        base::ObjectPtr new_object,
        const base::ObjectPtr old_object,
        double time_diff) {
    if (old_object->type == base::ObjectType::UNKNOWN
        || old_object->type == base::ObjectType::UNKNOWN_MOVABLE
        || old_object->type == base::ObjectType::UNKNOWN_UNMOVABLE) {
        new_object->velocity[0] = 0.0;
        new_object->velocity[1] = 0.0;
        new_object->velocity[2] = 0.0;
    }
    if (old_object->type == base::ObjectType::VEHICLE) {
        double vel = (new_object->velocity).norm();
        if (vel < 2.0) {
            new_object->velocity[0] = 0.0;
            new_object->velocity[1] = 0.0;
            new_object->velocity[2] = 0.0;
        }
    }
    if (old_object->type == base::ObjectType::BICYCLE) {
        double vel = (new_object->velocity).norm();
        if (vel < 2.0) {
            new_object->velocity[0] = 0.0;
            new_object->velocity[1] = 0.0;
            new_object->velocity[2] = 0.0;
        }
    }
}

void CameraTracker::UpdateUnassignedTracks(
    const base::Frame &camera_frame,
    const std::vector<size_t> &unassigned_tracks) {
  double timestamp = camera_frame.timestamp;
  auto &camera_tracks = track_manager_->mutable_tracks();
  for (size_t i = 0; i < unassigned_tracks.size(); ++i) {
    if (camera_tracks[unassigned_tracks[i]]->GetObs() != nullptr) {
      double camera_time = camera_tracks[unassigned_tracks[i]]->GetTimestamp();
      double time_diff = fabs(timestamp - camera_time);
      if (time_diff > s_tracking_time_win_) {
        camera_tracks[unassigned_tracks[i]]->SetDead();
      }
    } else {
      camera_tracks[unassigned_tracks[i]]->SetDead();
    }
    camera_tracks[unassigned_tracks[i]]->SetUnAssigned();
  }
}

void CameraTracker::DeleteLostTracks() { track_manager_->RemoveLostTracks(); }

void CameraTracker::CreateNewTracks(
    const base::Frame &camera_frame,
    const std::vector<size_t> &unassigned_objects) {
  for (size_t i = 0; i < unassigned_objects.size(); ++i) {
    CameraTrackPtr camera_track;
    camera_track.reset(new CameraTrack(
        camera_frame.objects[unassigned_objects[i]], camera_frame.timestamp));
    track_manager_->AddTrack(camera_track);
  }
}

void CameraTracker::CollectTrackedFrame(base::FramePtr tracked_frame) {
  if (tracked_frame == nullptr) {
    AERROR << "tracked_frame is nullptr";
    return;
  }
  auto &objects = tracked_frame->objects;
  objects.clear();
  const auto &camera_tracks = track_manager_->GetTracks();
  for (size_t i = 0; i < camera_tracks.size(); ++i) {
    if (camera_tracks[i]->ConfirmTrack() && camera_tracks[i]->IsAssigned()) {
      base::ObjectPtr object = base::ObjectPtr(new base::Object());
      const base::ObjectPtr &track_object = camera_tracks[i]->GetObs();
      *object = *track_object;
      object->tracking_time = camera_tracks[i]->GetTrackingTime();
      object->track_id = camera_tracks[i]->GetObsId();
      object->latest_tracked_time = camera_tracks[i]->GetTimestamp();
      objects.push_back(object);
    }
  }
}

PERCEPTION_REGISTER_TRACKER(CameraTracker);

}  // namespace camera
}  // namespace perception
}  // namespace apollo
