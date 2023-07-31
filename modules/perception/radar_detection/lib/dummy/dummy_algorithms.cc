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
#include "modules/perception/radar_detection/lib/dummy/dummy_algorithms.h"

#include "modules/perception/radar_detection/common/types.h"

namespace apollo {
namespace perception {
namespace radar {

void DummyDetector::ContiObs2Frame(
    const drivers::ContiRadar& corrected_obstacles,
    base::FramePtr radar_frame) {
  for (const auto& radar_obs : corrected_obstacles.contiobs()) {
    base::ObjectPtr radar_object(new base::Object);
    radar_object->id = radar_obs.obstacle_id();
    radar_object->track_id = radar_obs.obstacle_id();
    radar_object->center(0) = radar_obs.longitude_dist();
    radar_object->center(1) = radar_obs.lateral_dist();
    radar_object->center(2) = 0.0;
    radar_object->anchor_point = radar_object->center;

    radar_object->velocity(0) = static_cast<float>(radar_obs.longitude_vel());
    radar_object->velocity(1) = static_cast<float>(radar_obs.lateral_vel());
    radar_object->velocity(2) = 0.0f;

    Eigen::Matrix3d dist_rms;
    dist_rms.setZero();
    Eigen::Matrix3d vel_rms;
    vel_rms.setZero();
    dist_rms(0, 0) = radar_obs.longitude_dist_rms();
    dist_rms(1, 1) = radar_obs.lateral_dist_rms();
    vel_rms(0, 0) = radar_obs.longitude_vel_rms();
    vel_rms(1, 1) = radar_obs.lateral_vel_rms();
    radar_object->center_uncertainty =
        (dist_rms * dist_rms.transpose()).cast<float>();
    radar_object->velocity_uncertainty =
        (vel_rms * vel_rms.transpose()).cast<float>();

    double local_obj_theta = radar_obs.oritation_angle() / 180.0 * PI;
    Eigen::Vector3d direction(cos(local_obj_theta), sin(local_obj_theta), 0);
    radar_object->direction = direction.cast<float>();
    radar_object->theta =
        static_cast<float>(std::atan2(direction(1), direction(0)));
    radar_object->theta_variance =
        static_cast<float>(radar_obs.oritation_angle_rms() / 180.0 * PI);
    radar_object->size(0) = static_cast<float>(radar_obs.length());
    radar_object->size(1) = static_cast<float>(radar_obs.width());
    radar_object->confidence = static_cast<float>(radar_obs.probexist());

    int cls = radar_obs.obstacle_class();
    if (cls == CONTI_CAR || cls == CONTI_TRUCK) {
      radar_object->type = base::ObjectType::VEHICLE;
    } else if (cls == CONTI_PEDESTRIAN) {
      radar_object->type = base::ObjectType::PEDESTRIAN;
    } else if (cls == CONTI_MOTOCYCLE || cls == CONTI_BICYCLE) {
      radar_object->type = base::ObjectType::BICYCLE;
    } else {
      radar_object->type = base::ObjectType::UNKNOWN;
    }
    radar_frame->objects.push_back(radar_object);
  }
}

bool DummyPreprocessor::Init(const PreprocessorInitOptions& options) {
  return true;
}

bool DummyPreprocessor::Preprocess(const drivers::ContiRadar& raw_obstacles,
                                   const PreprocessorOptions& options,
                                   drivers::ContiRadar* corrected_obstacles) {
  if (corrected_obstacles == nullptr) {
    AERROR << "corrected_obstacles is not available";
    return false;
  }
  *corrected_obstacles = raw_obstacles;
  return true;
}

std::string DummyPreprocessor::Name() const { return "DummyPreprocessor"; }

bool DummyDetector::Init(const DetectorInitOptions& options) { return true; }

bool DummyDetector::Detect(const drivers::ContiRadar& corrected_obstacles,
                           const DetectorOptions& options,
                           base::FramePtr detected_frame) {
  ContiObs2Frame(corrected_obstacles, detected_frame);
  return true;
}

std::string DummyDetector::Name() const { return "DummyDetector"; }

bool DummyRoiFilter::Init(const RoiFilterInitOptions& options) { return true; }

bool DummyRoiFilter::RoiFilter(const RoiFilterOptions& options,
                               base::FramePtr radar_frame) {
  return true;
}
std::string DummyRoiFilter::Name() const { return "DummyRoiFilter"; }

PERCEPTION_REGISTER_PREPROCESSOR(DummyPreprocessor);
PERCEPTION_REGISTER_ROI_FILTER(DummyRoiFilter);
PERCEPTION_REGISTER_DETECTOR(DummyDetector);

}  // namespace radar
}  // namespace perception
}  // namespace apollo
