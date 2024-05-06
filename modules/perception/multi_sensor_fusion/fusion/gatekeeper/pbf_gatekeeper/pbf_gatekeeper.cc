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
#include "modules/perception/multi_sensor_fusion/fusion/gatekeeper/pbf_gatekeeper/pbf_gatekeeper.h"

#include <algorithm>
#include <limits>

#include "modules/perception/multi_sensor_fusion/proto/pbf_gatekeeper_config.pb.h"

#include "cyber/common/file.h"
#include "modules/perception/common/base/object_types.h"
#include "modules/perception/common/util.h"

namespace apollo {
namespace perception {
namespace fusion {

using cyber::common::GetAbsolutePath;

PbfGatekeeper::PbfGatekeeper() {}

PbfGatekeeper::~PbfGatekeeper() {}

bool PbfGatekeeper::Init(const GatekeeperInitOptions &options) {
  std::string config_file =
      GetConfigFile(options.config_path, options.config_file);

  PbfGatekeeperConfig params;
  if (!cyber::common::GetProtoFromFile(config_file, &params)) {
    AERROR << "Read config failed: " << config_file;
    return false;
  }

  params_.publish_if_has_lidar = params.publish_if_has_lidar();
  params_.publish_if_has_radar = params.publish_if_has_radar();
  params_.publish_if_has_camera = params.publish_if_has_camera();
  params_.use_camera_3d = params.use_camera_3d();
  params_.min_radar_confident_distance = params.min_radar_confident_distance();
  params_.max_radar_confident_angle = params.max_radar_confident_angle();
  params_.min_camera_publish_distance = params.min_camera_publish_distance();
  params_.invisible_period_threshold = params.invisible_period_threshold();
  params_.existence_threshold = params.existence_threshold();
  params_.radar_existence_threshold = params.radar_existence_threshold();
  params_.toic_threshold = params.toic_threshold();
  params_.use_track_time_pub_strategy = params.use_track_time_pub_strategy();
  params_.pub_track_time_thresh = params.pub_track_time_thresh();
  return true;
}

bool PbfGatekeeper::AbleToPublish(const TrackPtr &track) {
  bool invisible_in_lidar = !(track->IsLidarVisible());
  bool invisible_in_radar = !(track->IsRadarVisible());
  bool invisible_in_camera = !(track->IsCameraVisible());
  if (invisible_in_lidar && invisible_in_radar &&
      (!params_.use_camera_3d || invisible_in_camera)) {
    auto sensor_obj = track->GetFusedObject();
    if (sensor_obj != nullptr && sensor_obj->GetBaseObject()->sub_type !=
                                     base::ObjectSubType::TRAFFICCONE) {
      return false;
    }
  }
  time_t rawtime = static_cast<time_t>(track->GetFusedObject()->GetTimestamp());

  // use thread-safe localtime_r instead of localtime
  struct tm timeinfo;
  localtime_r(&rawtime, &timeinfo);
  bool is_night = (timeinfo.tm_hour >= 23);
  if (!LidarAbleToPublish(track) && !RadarAbleToPublish(track, is_night) &&
      !CameraAbleToPublish(track, is_night)) {
    return false;
  }

  track->AddTrackedTimes();
  if (params_.use_track_time_pub_strategy &&
      track->GetTrackedTimes() <=
          static_cast<size_t>(params_.pub_track_time_thresh)) {
    return false;
  }
  return true;
}

bool PbfGatekeeper::LidarAbleToPublish(const TrackPtr &track) {
  bool visible_in_lidar = track->IsLidarVisible();
  if (params_.publish_if_has_lidar && visible_in_lidar) {
    return true;
  }
  return false;
}

bool PbfGatekeeper::RadarAbleToPublish(const TrackPtr &track, bool is_night) {
  if (!params_.publish_if_has_radar) {
    return false;
  }

  bool visible_in_radar = track->IsRadarVisible();
  SensorObjectConstPtr radar_object = track->GetLatestRadarObject();
  if (!visible_in_radar || radar_object == nullptr) {
    return false;
  }

  if (radar_object->GetSensorId() == "radar_front") {
    if (radar_object->GetBaseObject()->radar_supplement.range >
            params_.min_radar_confident_distance &&
        radar_object->GetBaseObject()->radar_supplement.angle <
            params_.max_radar_confident_angle) {
      auto base_obj = track->GetFusedObject()->GetBaseObject();
      double heading_v = std::abs(base_obj->velocity.dot(base_obj->direction));
      auto set_velocity_to_zero = [heading_v, track]() {
        if (heading_v < 0.3) {
          track->GetFusedObject()->GetBaseObject()->velocity.setZero();
        }
      };

      double toic_p = track->GetToicProb();
      if (!is_night) {
        if (toic_p > params_.toic_threshold) {
          set_velocity_to_zero();
          return true;
        }
      } else {
        // the velocity buffer is [-3, +3] m/s
        double v_ct = 4.0;
        double v_slope = 1.0;
        auto heading_v_decision = [](double x, double c, double k) {
          x = x - c;
          return 0.5 + 0.5 * x * k / std::sqrt(1 + x * x * k * k);
        };
        auto fuse_two_probabilities = [](double p1, double p2) {
          double p = (p1 * p2) / (2 * p1 * p2 + 1 - p1 - p2);
          p = std::min(1.0 - std::numeric_limits<float>::epsilon(), p);
          return p;
        };

        double min_toic_p = 0.2;
        toic_p = std::max(min_toic_p, toic_p);
        double v_p = heading_v_decision(heading_v, v_ct, v_slope);
        double p = fuse_two_probabilities(toic_p, v_p);
        if (p > 0.5) {
          set_velocity_to_zero();
          return true;
        }
      }
    }
  } else if (radar_object->GetSensorId() == "radar_rear") {
    ADEBUG << "radar_rear: min_dis: " << params_.min_radar_confident_distance
           << " obj dist: "
           << radar_object->GetBaseObject()->radar_supplement.range
           << " track_id: " << track->GetTrackId()
           << " exist_prob: " << track->GetExistenceProb();
    if (radar_object->GetBaseObject()->radar_supplement.range >
            params_.min_radar_confident_distance &&
        (radar_object->GetBaseObject()->velocity.norm() > 4.0 ||
         track->GetExistenceProb() > params_.radar_existence_threshold)) {
      return true;
    }
  }

  return false;
}

bool PbfGatekeeper::CameraAbleToPublish(const TrackPtr &track, bool is_night) {
  bool visible_in_camera = track->IsCameraVisible();
  SensorId2ObjectMap &camera_objects = track->GetCameraObjects();
  auto iter = camera_objects.find("front_6mm");
  auto iter_narrow = camera_objects.find("front_12mm");
  iter = iter != camera_objects.end() ? iter : iter_narrow;
  if (camera_objects.find("camera_rear") != camera_objects.end()) {
    iter = camera_objects.find("camera_rear");
  } else if (camera_objects.find("camera_front") != camera_objects.end()) {
    iter = camera_objects.find("camera_front");
  }

  if (params_.publish_if_has_camera && visible_in_camera &&
      iter != camera_objects.end() && params_.use_camera_3d && !is_night) {
    SensorObjectConstPtr camera_object = iter->second;
    double range =
        camera_object->GetBaseObject()->camera_supplement.local_center.norm();
    // If sub_type of object is traffic cone publish it regardless of range
    if ((camera_object->GetBaseObject()->sub_type ==
         base::ObjectSubType::TRAFFICCONE) ||
        (range >= params_.min_camera_publish_distance)) {
      double exist_prob = track->GetExistenceProb();
      if (exist_prob > params_.existence_threshold) {
        static int cnt_cam = 1;
        AINFO << "publish camera only object : cnt =  " << cnt_cam;
        cnt_cam++;
        return true;
      }
    }
  }
  return false;
}

PERCEPTION_REGISTER_GATEKEEPER(PbfGatekeeper)

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
